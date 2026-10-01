#ifndef BOTZILLA_COVERAGE_LAYER__COVERAGE_COST_LAYER_HPP_
#define BOTZILLA_COVERAGE_LAYER__COVERAGE_COST_LAYER_HPP_

#include <mutex>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace botzilla_coverage_layer
{

// Global-costmap layer that makes ground the camera has already inspected slightly
// more expensive to drive over, so a cost-aware planner (SmacGrid) routes frontier
// legs and sweep transits through floor the Kinect has NOT seen yet — every ordinary
// journey then doubles as a partial coverage pass. See frontier_explorer_node's
// "Coverage cost" docstring section for the policy side.
//
// The cost itself is computed by frontier_explorer_node, which owns the swept mask,
// and published on `topic` as an OccupancyGrid in the /map frame whose values are
// already the published-scale cost (0 = no penalty, 100 = maximum). This layer only
// copies it into the master grid. Deliberately NOT a second StaticLayer instance:
// StaticLayer resizes the master costmap to whatever grid it receives, and a coverage
// message still carrying /map's previous dimensions after RTAB-Map has grown the map
// would resize the master back and wipe the real static layer until the next /map.
// This layer never resizes anything — it samples the coverage grid by world
// coordinate, so a size mismatch just means some cells get no penalty for a tick.
//
// Combination rule is max, and only below INSCRIBED_INFLATED_OBSTACLE: a cell is
// never made cheaper, never turned into a collision, and never touched if unknown.
// With max, every existing threshold in the stack (frontier_explorer's snap cutoff at
// published 75, its self-clearance at 99, the straight-line planner's inscribed
// check) is unchanged whenever the coverage value is below it — which it always is
// at the intended settings (published ~15).
//
// Must be listed AFTER inflation_layer: inflation writes its own costs with max as
// well, and running this first would have inflation treat nothing differently but
// would make the ordering intent unclear to the next reader.
class CoverageCostLayer : public nav2_costmap_2d::Layer
{
public:
  CoverageCostLayer() = default;

  void onInitialize() override;
  void reset() override;
  bool isClearable() override {return false;}

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;

  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

private:
  void coverageCallback(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);

  // World-space extent of a grid: [min_x, min_y, max_x, max_y].
  struct Extent
  {
    double min_x{0.0}, min_y{0.0}, max_x{0.0}, max_y{0.0};
    bool valid{false};
  };
  static Extent extentOf(const nav_msgs::msg::OccupancyGrid & grid);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  std::mutex mutex_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid_;
  // Area that must be repainted on the next update: the union of the previous and
  // the new grid's extent, so a grid that shrank, moved, or went all-zero clears
  // the stale penalty it used to cover.
  Extent dirty_;
};

}  // namespace botzilla_coverage_layer

#endif  // BOTZILLA_COVERAGE_LAYER__COVERAGE_COST_LAYER_HPP_
