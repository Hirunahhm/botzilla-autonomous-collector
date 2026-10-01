#ifndef BOTZILLA_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
#define BOTZILLA_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace botzilla_straightline_planner
{

// Global-planner plugin used only for pre-validated boustrophedon sweep legs.
//
// nav2_navfn_planner (the "GridBased" plugin used everywhere else in this project)
// computes a minimum-cost path over the costmap gradient, which curves away from
// inflation even between two waypoints already known to be free. During the
// camera-coverage sweep that reads as the robot wandering instead of tracing the
// lawnmower rows frontier_explorer_node/coverage_planning laid out. This plugin
// is deliberately dumb: it interpolates a straight line between start and goal and
// fails outright (rather than reroute) if that line crosses anything unsafe — the
// same "fail honestly, let the caller blacklist/replan" philosophy nav2_params.yaml
// documents for GridBased's tolerance/allow_unknown settings. It is only ever
// selected for sweep-leg goals (see frontier_explorer_node's planner_selector
// publish); frontier exploration keeps using GridBased.
class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::string global_frame_;
  std::string name_;

  double interpolation_resolution_{0.05};
  bool allow_unknown_{false};
};

}  // namespace botzilla_straightline_planner

#endif  // BOTZILLA_STRAIGHTLINE_PLANNER__STRAIGHT_LINE_PLANNER_HPP_
