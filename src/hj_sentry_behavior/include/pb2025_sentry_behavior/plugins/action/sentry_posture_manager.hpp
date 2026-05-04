#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_

#include <array>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "pb2025_sentry_behavior/omni_perception_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/competition_info.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

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
  static constexpr uint8_t kDefensePosture = 2;
  static constexpr uint8_t kMovePosture = 3;
  static constexpr int kGameProgressRunning = 4;

  struct Config
  {
    std::string topic_name = "/sentry_posture_cmd";
    double enemy_max_distance = 8.0;
    double enemy_stale_timeout_sec = 0.3;
    double cooldown_sec = 5.0;
    double decay_sec = 180.0;
    double budget_guard_sec = 170.0;
    double damage_window_sec = 3.0;
    int damage_threshold = 80;
    int defense_hp_threshold = 200;
    int warning_hp_min = 200;
    int warning_hp_max = 300;
    double linger_sec = 2.0;
    std::string highland_polygon;
    std::string map_frame = "map";
    std::string base_frame = "base_footprint";
  };

  Config readConfig();
  void resetMatchState(double now_sec);
  void updatePostureTime(double now_sec, bool count_time, const Config & config);
  void updateHpHistory(double now_sec, int hp, double window_sec);
  bool isDamageSpike(int current_hp, int damage_threshold) const;
  bool isEnemyVisible(const Config & config, double now_sec) const;
  bool isInHighland(const Config & config) const;
  bool isInsidePolygon(
    double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon) const;
  std::vector<geometry_msgs::msg::Point> parsePolygon(const std::string & polygon_str) const;
  uint8_t chooseDesiredPosture(
    const Config & config, bool game_running, bool alive, int current_hp, int enemy_outpost_hp,
    bool defense_raw, bool enemy_visible, bool in_highland, double now_sec);
  void applyCooldown(uint8_t desired_posture, double now_sec, double cooldown_sec);
  void publishPosture(const Config & config, uint8_t posture);
  void createPublisherIfNeeded(const std::string & topic_name);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr posture_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string current_topic_;

  uint8_t current_posture_ = kMovePosture;
  double last_switch_time_sec_ = 0.0;
  double last_tick_time_sec_ = 0.0;
  double defense_linger_until_sec_ = 0.0;
  double attack_linger_until_sec_ = 0.0;
  bool match_state_initialized_ = false;
  bool was_game_running_ = false;

  std::array<double, 4> posture_usage_sec_ = {0.0, 0.0, 0.0, 0.0};
  std::array<bool, 4> posture_decay_reported_ = {false, false, false, false};
  std::deque<std::pair<double, int>> hp_history_;

  std::optional<robot_msgs::msg::CompetitionInfo> latest_status_;
  std::optional<robot_msgs::msg::OmniPerception> latest_enemy_target_;
  std::optional<double> latest_enemy_target_rx_time_sec_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SENTRY_POSTURE_MANAGER_HPP_
