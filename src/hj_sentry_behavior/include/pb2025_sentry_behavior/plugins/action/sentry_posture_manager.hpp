#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_

#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "pb2025_sentry_behavior/omni_perception_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace pb2025_sentry_behavior
{

class SentryPostureManager : public BT::SyncActionNode
{
public:
  SentryPostureManager(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  static constexpr uint8_t kAttackPosture = 1;
  static constexpr uint8_t kMovePosture = 3;

  struct Config
  {
    std::string topic_name = "/sentry_posture_cmd";
    double enemy_max_distance = 8.0;
    double enemy_stale_timeout_sec = 0.3;
    double cooldown_sec = 5.0;
    double linger_sec = 2.0;
  };

  Config readConfig();
  bool isEnemyVisible(const Config & config, double now_sec) const;
  uint8_t chooseDesiredPosture(const Config & config, bool enemy_visible, double now_sec);
  void applyCooldown(uint8_t desired_posture, double now_sec, double cooldown_sec);
  void publishPosture(const Config & config, uint8_t posture);
  void createPublisherIfNeeded(const std::string & topic_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr posture_pub_;
  std::string current_topic_;

  uint8_t current_posture_ = kMovePosture;
  double last_switch_time_sec_ = -1000000.0;
  double attack_linger_until_sec_ = 0.0;

  std::optional<robot_msgs::msg::OmniPerception> latest_enemy_target_;
  std::optional<double> latest_enemy_target_rx_time_sec_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_
