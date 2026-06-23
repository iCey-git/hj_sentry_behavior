#include "pb2025_sentry_behavior/plugins/action/sentry_posture_manager.hpp"

#include <stdexcept>

#include "robot_msgs/msg/competition_info.hpp"

namespace pb2025_sentry_behavior
{

SentryPostureManager::SentryPostureManager(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, config)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }
}

BT::PortsList SentryPostureManager::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "topic_name", "/sentry_posture_cmd", "Posture command topic"),
    BT::InputPort<robot_msgs::msg::CompetitionInfo>(
      "status_port", "{@referee_robotStatus}", "CompetitionInfo port on blackboard"),
    BT::InputPort<robot_msgs::msg::OmniPerception>(
      "enemy_port", "{@target_info}", "OmniPerception target info on blackboard"),
    BT::InputPort<double>(
      "enemy_rx_time_sec", "{@target_info_rx_time_sec}",
      "Last receive time of target_info in ROS time seconds"),
    BT::InputPort<double>("enemy_max_distance", 8.0, "Max enemy distance for attack posture"),
    BT::InputPort<double>("enemy_stale_timeout_sec", 0.3, "Enemy message stale timeout"),
    BT::InputPort<double>("cooldown_sec", 5.0, "Minimum seconds between posture switches"),
    BT::InputPort<double>("linger_sec", 2.0, "Posture condition linger time"),
  };
}

BT::NodeStatus SentryPostureManager::tick()
{
  const auto config = readConfig();
  createPublisherIfNeeded(config.topic_name);

  const double now_sec = node_->now().seconds();
  latest_enemy_target_.reset();
  latest_enemy_target_rx_time_sec_.reset();

  const auto status = getInput<robot_msgs::msg::CompetitionInfo>("status_port");
  if (auto enemy_target = getInput<robot_msgs::msg::OmniPerception>("enemy_port")) {
    latest_enemy_target_ = enemy_target.value();
  }
  if (auto rx_time = getInput<double>("enemy_rx_time_sec")) {
    latest_enemy_target_rx_time_sec_ = rx_time.value();
  }

  const bool enemy_visible = isEnemyVisible(config, now_sec);
  const uint8_t desired_posture = chooseDesiredPosture(config, enemy_visible, now_sec);
  applyCooldown(desired_posture, now_sec, config.cooldown_sec);
  if (status && current_posture_ != static_cast<uint8_t>(status.value().sentry_posture)) {
    publishPosture(config, current_posture_);
  }

  return BT::NodeStatus::SUCCESS;
}

SentryPostureManager::Config SentryPostureManager::readConfig()
{
  Config config;
  getInput("topic_name", config.topic_name);
  getInput("enemy_max_distance", config.enemy_max_distance);
  getInput("enemy_stale_timeout_sec", config.enemy_stale_timeout_sec);
  getInput("cooldown_sec", config.cooldown_sec);
  getInput("linger_sec", config.linger_sec);
  return config;
}

bool SentryPostureManager::isEnemyVisible(const Config & config, double now_sec) const
{
  if (!latest_enemy_target_ || !latest_enemy_target_rx_time_sec_) {
    return false;
  }
  if (now_sec - latest_enemy_target_rx_time_sec_.value() > config.enemy_stale_timeout_sec) {
    return false;
  }

  return isEnemyVisibleForPosture(latest_enemy_target_.value(), config.enemy_max_distance);
}

uint8_t SentryPostureManager::chooseDesiredPosture(
  const Config & config, bool enemy_visible, double now_sec)
{
  if (enemy_visible) {
    attack_linger_until_sec_ = now_sec + config.linger_sec;
  }

  if (now_sec <= attack_linger_until_sec_) {
    return kAttackPosture;
  }

  return kDefensePosture;
}

void SentryPostureManager::applyCooldown(
  uint8_t desired_posture, double now_sec, double cooldown_sec)
{
  if (desired_posture == current_posture_) {
    return;
  }
  if (now_sec - last_switch_time_sec_ < cooldown_sec) {
    return;
  }

  RCLCPP_INFO(
    node_->get_logger(), "Sentry posture switch: %u -> %u", current_posture_, desired_posture);
  current_posture_ = desired_posture;
  last_switch_time_sec_ = now_sec;
}

void SentryPostureManager::publishPosture(const Config & config, uint8_t posture)
{
  createPublisherIfNeeded(config.topic_name);
  std_msgs::msg::UInt8 msg;
  msg.data = posture;
  posture_pub_->publish(msg);
}

void SentryPostureManager::createPublisherIfNeeded(const std::string & topic_name)
{
  if (posture_pub_ && current_topic_ == topic_name) {
    return;
  }
  posture_pub_ = node_->create_publisher<std_msgs::msg::UInt8>(topic_name, 1);
  current_topic_ = topic_name;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::SentryPostureManager>(
    "SentryPostureManager", params);
}
