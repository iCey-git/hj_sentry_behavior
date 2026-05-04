#include "pb2025_sentry_behavior/plugins/condition/enemy_stop_gate.hpp"

#include <algorithm>
#include <cmath>

namespace pb2025_sentry_behavior
{

EnemyStopGate::EnemyStopGate(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::ConditionNode(name, config)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }

  mode_control_pub_ = node_->create_publisher<robot_msgs::msg::ModeControl>("/mode_ctrl", 1);
  twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  nav_cancel_client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
}

BT::PortsList EnemyStopGate::providedPorts()
{
  return {
    BT::InputPort<robot_msgs::msg::OmniPerception>(
      "key_port", "{@target_info}", "OmniPerception target info on blackboard"),
    BT::InputPort<double>(
      "rx_time_sec", "{@target_info_rx_time_sec}",
      "Last receive time of target_info in ROS time seconds"),
    BT::InputPort<float>("max_distance", 8.0, "Distance to enemy target"),
    BT::InputPort<double>(
      "stale_timeout_sec", 0.3, "Treat stale target messages as no enemy after this timeout"),
    BT::InputPort<double>(
      "linger_sec", 0.8, "Keep stop state for this many seconds after the last valid enemy"),
    BT::InputPort<bool>("rough_road_active", false, "Override chassis gyro on rough road"),
    BT::InputPort<int>("rough_chassis_gyro", 0, "Chassis gyro value on rough road"),
  };
}

BT::NodeStatus EnemyStopGate::tick()
{
  float max_distance = 8.0F;
  double stale_timeout_sec = 0.3;
  double linger_sec = 0.8;
  const double now_sec = node_->now().seconds();

  auto msg = getInput<robot_msgs::msg::OmniPerception>("key_port");
  auto rx_time_sec = getInput<double>("rx_time_sec");
  if (!msg || !rx_time_sec) {
    if (now_sec <= stop_linger_until_sec_) {
      cancelNavigationIfNeeded(now_sec);
      publishStopCommands();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  getInput("max_distance", max_distance);
  getInput("stale_timeout_sec", stale_timeout_sec);
  getInput("linger_sec", linger_sec);

  const double age_sec = now_sec - rx_time_sec.value();
  if (age_sec > stale_timeout_sec) {
    if (now_sec <= stop_linger_until_sec_) {
      cancelNavigationIfNeeded(now_sec);
      publishStopCommands();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  if (!shouldStopForTarget(msg.value(), max_distance)) {
    if (now_sec <= stop_linger_until_sec_) {
      cancelNavigationIfNeeded(now_sec);
      publishStopCommands();
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  stop_linger_until_sec_ = now_sec + linger_sec;
  cancelNavigationIfNeeded(now_sec);
  publishStopCommands();
  return BT::NodeStatus::SUCCESS;
}

bool EnemyStopGate::shouldStopForTarget(
  const robot_msgs::msg::OmniPerception & target_info, float max_distance) const
{
  return shouldStopForTargetInfo(target_info, max_distance);
}

void EnemyStopGate::publishStopCommands()
{
  bool rough_road_active = false;
  int rough_chassis_gyro = 0;
  getInput("rough_road_active", rough_road_active);
  getInput("rough_chassis_gyro", rough_chassis_gyro);

  int chassis_gyro = 1;
  if (rough_road_active) {
    chassis_gyro = rough_chassis_gyro;
  }

  robot_msgs::msg::ModeControl mode_msg;
  mode_msg.chassis_gyro = static_cast<uint16_t>(std::clamp(chassis_gyro, 0, 65535));
  mode_msg.patrol_mode = 2;
  mode_msg.power_mode = 12;
  mode_control_pub_->publish(mode_msg);

  geometry_msgs::msg::Twist twist_msg;
  twist_msg.linear.x = 0.0;
  twist_msg.linear.y = 0.0;
  twist_msg.angular.z = 0.0;
  twist_pub_->publish(twist_msg);
}

void EnemyStopGate::cancelNavigationIfNeeded(double now_sec)
{
  if (!nav_cancel_client_) {
    return;
  }
  if (now_sec - last_cancel_time_sec_ < 0.2) {
    return;
  }
  if (!nav_cancel_client_->action_server_is_ready()) {
    return;
  }

  last_cancel_time_sec_ = now_sec;
  nav_cancel_client_->async_cancel_all_goals();
}

}

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::EnemyStopGate>("EnemyStopGate", params);
}
