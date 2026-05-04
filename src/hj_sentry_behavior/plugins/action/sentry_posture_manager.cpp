#include "pb2025_sentry_behavior/plugins/action/sentry_posture_manager.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

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
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
    BT::InputPort<double>("decay_sec", 180.0, "Posture decay threshold in seconds"),
    BT::InputPort<double>(
      "budget_guard_sec", 170.0, "Prefer moving posture after this posture usage"),
    BT::InputPort<double>("damage_window_sec", 3.0, "HP drop detection window"),
    BT::InputPort<int>("damage_threshold", 80, "HP drop threshold in damage window"),
    BT::InputPort<int>("defense_hp_threshold", 200, "HP threshold for defense posture"),
    BT::InputPort<int>("warning_hp_min", 200, "Lower bound of warning HP range"),
    BT::InputPort<int>("warning_hp_max", 300, "Upper bound of warning HP range"),
    BT::InputPort<double>("linger_sec", 2.0, "Posture condition linger time"),
    BT::InputPort<int>("enemy_outpost_hp", "{@enemy_outpost_hp}", "Enemy outpost HP"),
    BT::InputPort<std::string>(
      "highland_polygon", "{@highland_polygon}", "Central highland polygon"),
    BT::InputPort<std::string>("map_frame", "map", "Map frame"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
  };
}

BT::NodeStatus SentryPostureManager::tick()
{
  const auto config = readConfig();
  createPublisherIfNeeded(config.topic_name);

  const double now_sec = node_->now().seconds();
  latest_status_.reset();
  latest_enemy_target_.reset();
  latest_enemy_target_rx_time_sec_.reset();

  if (auto status = getInput<robot_msgs::msg::CompetitionInfo>("status_port")) {
    latest_status_ = status.value();
  }
  if (auto enemy_target = getInput<robot_msgs::msg::OmniPerception>("enemy_port")) {
    latest_enemy_target_ = enemy_target.value();
  }
  if (auto rx_time = getInput<double>("enemy_rx_time_sec")) {
    latest_enemy_target_rx_time_sec_ = rx_time.value();
  }

  if (!latest_status_) {
    current_posture_ = kMovePosture;
    publishPosture(config, kMovePosture);
    return BT::NodeStatus::SUCCESS;
  }

  const bool game_running = latest_status_->game_state == kGameProgressRunning;
  const bool alive = latest_status_->our_sentry_hp > 0;
  if (game_running && !was_game_running_) {
    resetMatchState(now_sec);
  }
  was_game_running_ = game_running;

  updatePostureTime(now_sec, game_running && alive, config);

  const int current_hp = static_cast<int>(latest_status_->our_sentry_hp);
  updateHpHistory(now_sec, current_hp, config.damage_window_sec);

  if (!game_running || !alive) {
    current_posture_ = kMovePosture;
    defense_linger_until_sec_ = 0.0;
    attack_linger_until_sec_ = 0.0;
    publishPosture(config, kMovePosture);
    return BT::NodeStatus::SUCCESS;
  }

  int enemy_outpost_hp = static_cast<int>(latest_status_->enemy_outpost_hp);
  if (auto enemy_outpost_hp_input = getInput<int>("enemy_outpost_hp")) {
    enemy_outpost_hp = enemy_outpost_hp_input.value();
  }

  const bool damage_spike = isDamageSpike(current_hp, config.damage_threshold);
  const bool enemy_visible = isEnemyVisible(config, now_sec);
  const bool defense_raw = damage_spike;
  const bool in_highland = isInHighland(config);

  const uint8_t desired_posture = chooseDesiredPosture(
    config, game_running, alive, current_hp, enemy_outpost_hp, defense_raw, enemy_visible,
    in_highland, now_sec);
  applyCooldown(desired_posture, now_sec, config.cooldown_sec);
  publishPosture(config, current_posture_);

  return BT::NodeStatus::SUCCESS;
}

SentryPostureManager::Config SentryPostureManager::readConfig()
{
  Config config;
  getInput("topic_name", config.topic_name);
  getInput("enemy_max_distance", config.enemy_max_distance);
  getInput("enemy_stale_timeout_sec", config.enemy_stale_timeout_sec);
  getInput("cooldown_sec", config.cooldown_sec);
  getInput("decay_sec", config.decay_sec);
  getInput("budget_guard_sec", config.budget_guard_sec);
  getInput("damage_window_sec", config.damage_window_sec);
  getInput("damage_threshold", config.damage_threshold);
  getInput("defense_hp_threshold", config.defense_hp_threshold);
  getInput("warning_hp_min", config.warning_hp_min);
  getInput("warning_hp_max", config.warning_hp_max);
  getInput("linger_sec", config.linger_sec);
  getInput("highland_polygon", config.highland_polygon);
  getInput("map_frame", config.map_frame);
  getInput("base_frame", config.base_frame);
  return config;
}

void SentryPostureManager::resetMatchState(double now_sec)
{
  current_posture_ = kMovePosture;
  last_switch_time_sec_ = now_sec - 1000000.0;
  last_tick_time_sec_ = now_sec;
  defense_linger_until_sec_ = 0.0;
  attack_linger_until_sec_ = 0.0;
  match_state_initialized_ = true;
  posture_usage_sec_.fill(0.0);
  posture_decay_reported_.fill(false);
  hp_history_.clear();
}

void SentryPostureManager::updatePostureTime(
  double now_sec, bool count_time, const Config & config)
{
  if (!match_state_initialized_) {
    last_tick_time_sec_ = now_sec;
    match_state_initialized_ = true;
    return;
  }

  const double dt = std::max(0.0, now_sec - last_tick_time_sec_);
  last_tick_time_sec_ = now_sec;
  if (!count_time || current_posture_ >= posture_usage_sec_.size()) {
    return;
  }

  posture_usage_sec_[current_posture_] += dt;
  if (
    !posture_decay_reported_[current_posture_] &&
    posture_usage_sec_[current_posture_] >= config.decay_sec)
  {
    posture_decay_reported_[current_posture_] = true;
    RCLCPP_WARN(
      node_->get_logger(), "Sentry posture %u usage reached decay threshold %.1fs",
      current_posture_, config.decay_sec);
  }
}

void SentryPostureManager::updateHpHistory(double now_sec, int hp, double window_sec)
{
  hp_history_.emplace_back(now_sec, hp);
  while (!hp_history_.empty() && now_sec - hp_history_.front().first > window_sec) {
    hp_history_.pop_front();
  }
}

bool SentryPostureManager::isDamageSpike(int current_hp, int damage_threshold) const
{
  if (hp_history_.empty()) {
    return false;
  }

  int max_hp = current_hp;
  for (const auto & [time_sec, hp] : hp_history_) {
    (void)time_sec;
    max_hp = std::max(max_hp, hp);
  }
  return max_hp - current_hp >= damage_threshold;
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

bool SentryPostureManager::isInHighland(const Config & config) const
{
  if (config.highland_polygon.empty()) {
    return false;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      config.map_frame, config.base_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG(node_->get_logger(), "Posture highland TF lookup failed: %s", ex.what());
    return false;
  }

  try {
    const auto highland_polygon = parsePolygon(config.highland_polygon);
    return isInsidePolygon(
      transform.transform.translation.x, transform.transform.translation.y, highland_polygon);
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node_->get_logger(), "Posture highland polygon parse failed: %s", ex.what());
    return false;
  }
}

bool SentryPostureManager::isInsidePolygon(
  double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon) const
{
  const int n = static_cast<int>(polygon.size());
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const bool intersect =
      ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<geometry_msgs::msg::Point> SentryPostureManager::parsePolygon(
  const std::string & polygon_str) const
{
  std::vector<geometry_msgs::msg::Point> polygon;
  std::istringstream ss(polygon_str);
  std::string token;
  while (std::getline(ss, token, ';')) {
    std::istringstream pair_ss(token);
    std::string x_str;
    std::string y_str;
    if (!std::getline(pair_ss, x_str, ',') || !std::getline(pair_ss, y_str, ',')) {
      throw std::runtime_error("Invalid polygon token: " + token);
    }
    geometry_msgs::msg::Point point;
    point.x = std::stod(x_str);
    point.y = std::stod(y_str);
    point.z = 0.0;
    polygon.push_back(point);
  }
  if (polygon.size() < 3) {
    throw std::runtime_error("Polygon must have at least 3 vertices");
  }
  return polygon;
}

uint8_t SentryPostureManager::chooseDesiredPosture(
  const Config & config, bool game_running, bool alive, int current_hp, int enemy_outpost_hp,
  bool defense_raw, bool enemy_visible, bool in_highland, double now_sec)
{
  if (!game_running || !alive) {
    return kMovePosture;
  }

  if (current_hp <= config.defense_hp_threshold) {
    return kMovePosture;
  }

  const bool warning_hp =
    current_hp > config.warning_hp_min && current_hp <= config.warning_hp_max;
  const bool healthy_hp = current_hp > config.warning_hp_max;
  const bool outpost_alive = enemy_outpost_hp > 0;

  if (warning_hp && defense_raw) {
    defense_linger_until_sec_ = now_sec + config.linger_sec;
  }
  if (healthy_hp && enemy_visible) {
    attack_linger_until_sec_ = now_sec + config.linger_sec;
  }

  if (warning_hp && now_sec <= defense_linger_until_sec_) {
    if (posture_usage_sec_[kDefensePosture] >= config.budget_guard_sec) {
      return kMovePosture;
    }
    return kDefensePosture;
  }

  if (warning_hp && !outpost_alive && enemy_visible) {
    if (posture_usage_sec_[kDefensePosture] < config.budget_guard_sec) {
      defense_linger_until_sec_ = now_sec + config.linger_sec;
      return kDefensePosture;
    }
    return kMovePosture;
  }

  if (warning_hp && outpost_alive && in_highland) {
    if (posture_usage_sec_[kAttackPosture] < config.budget_guard_sec) {
      return kAttackPosture;
    }
    return kMovePosture;
  }

  if (warning_hp) {
    return kMovePosture;
  }

  if (healthy_hp && now_sec <= attack_linger_until_sec_) {
    if (posture_usage_sec_[kAttackPosture] >= config.budget_guard_sec) {
      return kMovePosture;
    }
    return kAttackPosture;
  }

  return kMovePosture;
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
  // Publish every tick so a lower controller that rejects commands during cooldown
  // can still accept the same request once its own cooldown expires.
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
