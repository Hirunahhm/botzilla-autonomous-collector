#include "botzilla_coverage_layer/coverage_cost_layer.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace botzilla_coverage_layer
{

void CoverageCostLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("CoverageCostLayer: failed to lock lifecycle node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/coverage_cost_map")));
  node->get_parameter(name_ + ".enabled", enabled_);
  std::string topic;
  node->get_parameter(name_ + ".topic", topic);

  // Always current: this layer only ever adds a soft preference. If the explorer is
  // not running (or not publishing yet) the costmap must still be usable, so the
  // planner is never held up waiting on it.
  current_ = true;

  // Transient-local so a costmap that (re)starts after the explorer's last publish
  // still gets the latest grid, matching how /map itself is delivered.
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic, qos,
    std::bind(&CoverageCostLayer::coverageCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_, "CoverageCostLayer '%s': %s, listening on %s",
    name_.c_str(), enabled_ ? "enabled" : "disabled", topic.c_str());
}

void CoverageCostLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (grid_) {
    dirty_ = extentOf(*grid_);
  }
  grid_.reset();
}

CoverageCostLayer::Extent CoverageCostLayer::extentOf(const nav_msgs::msg::OccupancyGrid & grid)
{
  Extent e;
  if (grid.info.width == 0 || grid.info.height == 0) {
    return e;
  }
  e.min_x = grid.info.origin.position.x;
  e.min_y = grid.info.origin.position.y;
  e.max_x = e.min_x + grid.info.width * grid.info.resolution;
  e.max_y = e.min_y + grid.info.height * grid.info.resolution;
  e.valid = true;
  return e;
}

void CoverageCostLayer::coverageCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  Extent next = extentOf(*msg);
  if (grid_) {
    Extent prev = extentOf(*grid_);
    if (prev.valid && next.valid) {
      next.min_x = std::min(next.min_x, prev.min_x);
      next.min_y = std::min(next.min_y, prev.min_y);
      next.max_x = std::max(next.max_x, prev.max_x);
      next.max_y = std::max(next.max_y, prev.max_y);
    } else if (prev.valid) {
      next = prev;
    }
  }
  if (next.valid) {
    if (dirty_.valid) {
      dirty_.min_x = std::min(dirty_.min_x, next.min_x);
      dirty_.min_y = std::min(dirty_.min_y, next.min_y);
      dirty_.max_x = std::max(dirty_.max_x, next.max_x);
      dirty_.max_y = std::max(dirty_.max_y, next.max_y);
    } else {
      dirty_ = next;
    }
  }
  grid_ = msg;
}

void CoverageCostLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!dirty_.valid) {
    return;
  }
  // Grow the update window over the whole coverage grid when it changes. Everything
  // inside the window is reset and repainted by every layer, so this is also what
  // removes a penalty the new grid no longer carries. Between messages this layer
  // adds nothing and just repaints whatever window the other layers asked for.
  *min_x = std::min(*min_x, dirty_.min_x);
  *min_y = std::min(*min_y, dirty_.min_y);
  *max_x = std::max(*max_x, dirty_.max_x);
  *max_y = std::max(*max_y, dirty_.max_y);
  dirty_.valid = false;
}

void CoverageCostLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = grid_;
  }
  if (!grid || grid->info.width == 0 || grid->info.height == 0) {
    return;
  }

  const double res = grid->info.resolution;
  const double ox = grid->info.origin.position.x;
  const double oy = grid->info.origin.position.y;
  const int gw = static_cast<int>(grid->info.width);
  const int gh = static_cast<int>(grid->info.height);
  // Published 0..100 -> internal 0..252. Capped one below INSCRIBED so this layer
  // can never, by itself, make a cell a collision.
  constexpr int kMaxInternal = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE - 1;

  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      const unsigned char current = master_grid.getCost(i, j);
      if (current == nav2_costmap_2d::NO_INFORMATION ||
        current >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      {
        continue;
      }
      double wx, wy;
      master_grid.mapToWorld(i, j, wx, wy);
      const int gx = static_cast<int>(std::floor((wx - ox) / res));
      const int gy = static_cast<int>(std::floor((wy - oy) / res));
      if (gx < 0 || gy < 0 || gx >= gw || gy >= gh) {
        continue;
      }
      const int8_t value = grid->data[gy * gw + gx];
      if (value <= 0) {
        continue;
      }
      const int cost = std::min(
        kMaxInternal, static_cast<int>(std::lround(std::min<int>(value, 100) * 2.52)));
      if (cost > current) {
        master_grid.setCost(i, j, static_cast<unsigned char>(cost));
      }
    }
  }
}

}  // namespace botzilla_coverage_layer

PLUGINLIB_EXPORT_CLASS(botzilla_coverage_layer::CoverageCostLayer, nav2_costmap_2d::Layer)
