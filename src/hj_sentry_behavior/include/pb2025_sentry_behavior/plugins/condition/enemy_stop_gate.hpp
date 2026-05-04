#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__ENEMY_STOP_GATE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__ENEMY_STOP_GATE_HPP_

#include <memory>
#include <string>

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pb2025_sentry_behavior/omni_perception_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/mode_control.hpp"

namespace pb2025_sentry_behavior
{

class EnemyStopGate : public BT::ConditionNode
{
public:
  EnemyStopGate(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;

  bool shouldStopForTarget(
    const robot_msgs::msg::OmniPerception & target_info, float max_distance) const;

  void publishStopCommands();
  void cancelNavigationIfNeeded(double now_sec);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<robot_msgs::msg::ModeControl>::SharedPtr mode_control_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_cancel_client_;
  rclcpp::Logger logger_ = rclcpp::get_logger("EnemyStopGate");
  double stop_linger_until_sec_ = 0.0;
  double last_cancel_time_sec_ = 0.0;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__ENEMY_STOP_GATE_HPP_
