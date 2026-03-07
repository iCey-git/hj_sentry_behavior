#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__FIND_REACHABLE_GOAL_IN_AREA_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__FIND_REACHABLE_GOAL_IN_AREA_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav2_util/robot_utils.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class FindReachableGoalInAreaAction : public BT::SyncActionNode
{
public:
  FindReachableGoalInAreaAction(
    const std::string & name, const BT::NodeConfig & conf,
    const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /// Generate candidate points in a filled disk area
  std::vector<geometry_msgs::msg::Point> generateGridCandidates(
    const geometry_msgs::msg::Point & center, double radius, double step) const;

  /// Filter candidates by costmap cost
  std::vector<geometry_msgs::msg::Point> filterFeasiblePoints(
    const std::vector<geometry_msgs::msg::Point> & candidates,
    const nav_msgs::msg::OccupancyGrid & costmap, int cost_threshold) const;

  /// Select the point nearest to the robot
  geometry_msgs::msg::Point selectNearestPoint(
    const std::vector<geometry_msgs::msg::Point> & feasible_points,
    const geometry_msgs::msg::Point & robot_position) const;

  /// Publish RViz visualization markers
  void publishVisualization(
    const geometry_msgs::msg::Point & center, double radius,
    const geometry_msgs::msg::Point & selected_goal,
    const std::string & frame_id) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_publisher_;
  std::string robot_base_frame_;
  std::string global_frame_;
  bool visualize_;
  double transform_tolerance_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__FIND_REACHABLE_GOAL_IN_AREA_HPP_
