#include "botzilla_straightline_planner/straight_line_planner.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/utils.h"

namespace botzilla_straightline_planner
{

void StraightLinePlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  tf_ = tf;
  name_ = name;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  auto node = node_.lock();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.05));
  node->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  // Sweep waypoints are pre-validated as free by coverage_planning.py against the
  // occupancy grid, but the space between them can still hold NO_INFORMATION cells
  // near the row edges. Default false to match GridBased's allow_unknown=false and
  // fail the plan rather than route confidently through unsensed territory.
  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".allow_unknown", rclcpp::ParameterValue(false));
  node->get_parameter(name_ + ".allow_unknown", allow_unknown_);
}

void StraightLinePlanner::cleanup() {}
void StraightLinePlanner::activate() {}
void StraightLinePlanner::deactivate() {}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> cancel_checker)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = global_frame_;
  path.header.stamp = node_.lock()->now();

  unsigned int mx, my;
  if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx, my)) {
    throw nav2_core::StartOutsideMapBounds(
      "Sweep-leg start (" + std::to_string(start.pose.position.x) + ", " +
      std::to_string(start.pose.position.y) + ") is outside the costmap.");
  }
  if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
    throw nav2_core::GoalOutsideMapBounds(
      "Sweep-leg goal (" + std::to_string(goal.pose.position.x) + ", " +
      std::to_string(goal.pose.position.y) + ") is outside the costmap.");
  }

  const double dx = goal.pose.position.x - start.pose.position.x;
  const double dy = goal.pose.position.y - start.pose.position.y;
  const double distance = std::hypot(dx, dy);
  const double heading = std::atan2(dy, dx);

  const int num_steps = std::max(
    1, static_cast<int>(std::ceil(distance / interpolation_resolution_)));

  for (int i = 0; i <= num_steps; ++i) {
    if (cancel_checker()) {
      throw nav2_core::PlannerCancelled("Sweep-leg straight-line planning was cancelled.");
    }

    const double t = static_cast<double>(i) / static_cast<double>(num_steps);
    const double wx = start.pose.position.x + t * dx;
    const double wy = start.pose.position.y + t * dy;

    if (!costmap_->worldToMap(wx, wy, mx, my)) {
      throw nav2_core::NoValidPathCouldBeFound(
        "Straight line from sweep-leg start to goal leaves the costmap.");
    }
    const unsigned char cost = costmap_->getCost(mx, my);
    const bool lethal = cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
      cost != nav2_costmap_2d::NO_INFORMATION;
    const bool unknown_blocked = cost == nav2_costmap_2d::NO_INFORMATION && !allow_unknown_;
    if (lethal || unknown_blocked) {
      throw nav2_core::NoValidPathCouldBeFound(
        "Straight line from sweep-leg start to goal is blocked at (" +
        std::to_string(wx) + ", " + std::to_string(wy) +
        "), cost=" + std::to_string(static_cast<int>(cost)) +
        " — caller should mark this waypoint unreachable rather than reroute.");
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = global_frame_;
    pose.header.stamp = path.header.stamp;
    pose.pose.position.x = wx;
    pose.pose.position.y = wy;
    if (i == num_steps) {
      // Last pose: honor the caller's requested final orientation.
      pose.pose.orientation = goal.pose.orientation;
    } else {
      pose.pose.orientation.z = std::sin(heading / 2.0);
      pose.pose.orientation.w = std::cos(heading / 2.0);
    }
    path.poses.push_back(pose);
  }

  return path;
}

}  // namespace botzilla_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(botzilla_straightline_planner::StraightLinePlanner, nav2_core::GlobalPlanner)
