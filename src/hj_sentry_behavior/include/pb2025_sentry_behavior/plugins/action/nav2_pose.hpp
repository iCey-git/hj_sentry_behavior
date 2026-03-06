#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_sentry_behavior
{

class Nav2PoseAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2PoseAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool createClient(const std::string & action_name);
  bool sendGoal(const geometry_msgs::msg::PoseStamped & goal);
  void publishGoalPose(const geometry_msgs::msg::PoseStamped & goal);
  static bool goalsEqual(
    const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs);
  void resetGoalState();

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::string goal_pose_topic_;
  std::chrono::milliseconds server_timeout_;
  std::chrono::milliseconds wait_for_server_timeout_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  geometry_msgs::msg::PoseStamped active_goal_;
  std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_HPP_
