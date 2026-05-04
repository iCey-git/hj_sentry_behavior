#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_THROUGH_POSES_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_THROUGH_POSES_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace pb2025_sentry_behavior
{

class Nav2ThroughPosesAction : public BT::StatefulActionNode
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses =
    rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  Nav2ThroughPosesAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool createClient(const std::string & action_name);
  bool sendGoal(const std::vector<geometry_msgs::msg::PoseStamped> & goals);
  void publishFinalGoalPose(const std::vector<geometry_msgs::msg::PoseStamped> & goals);
  static bool goalsEqual(
    const std::vector<geometry_msgs::msg::PoseStamped> & lhs,
    const std::vector<geometry_msgs::msg::PoseStamped> & rhs);
  void resetGoalState();

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::string goal_pose_topic_;
  std::chrono::milliseconds server_timeout_;
  std::chrono::milliseconds wait_for_server_timeout_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  GoalHandleNavigateThroughPoses::SharedPtr goal_handle_;
  std::vector<geometry_msgs::msg::PoseStamped> active_goals_;
  std::shared_future<GoalHandleNavigateThroughPoses::WrappedResult> result_future_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_THROUGH_POSES_HPP_
