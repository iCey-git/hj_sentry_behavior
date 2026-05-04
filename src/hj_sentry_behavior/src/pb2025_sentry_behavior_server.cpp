// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pb2025_sentry_behavior/pb2025_sentry_behavior_server.hpp"

#include <filesystem>
#include <fstream>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "robot_msgs/msg/competition_info.hpp"
#include "robot_msgs/msg/omni_perception.hpp"
#include "robot_msgs/msg/rfid_status.hpp"
namespace pb2025_sentry_behavior
{

template <typename T>
void SentryBehaviorServer::subscribe(
  const std::string & topic, const std::string & bb_key, const rclcpp::QoS & qos)
{
  auto sub = node()->create_subscription<T>(
    topic, qos,
    [this, bb_key](const typename T::SharedPtr msg) { globalBlackboard()->set(bb_key, *msg); });
  subscriptions_.push_back(sub);
}

SentryBehaviorServer::SentryBehaviorServer(const rclcpp::NodeOptions & options)
: TreeExecutionServer(options)
{
  node()->declare_parameter("use_cout_logger", false);
  node()->get_parameter("use_cout_logger", use_cout_logger_);

  auto set_parameter_to_blackboard =
    [this](const std::string & name, const rclcpp::ParameterValue & default_value) {
      node()->declare_parameter(name, default_value);
      rclcpp::Parameter parameter;
      node()->get_parameter(name, parameter);
      switch (parameter.get_type()) {
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          globalBlackboard()->set(name, static_cast<int>(parameter.as_int()));
          break;
        case rclcpp::ParameterType::PARAMETER_STRING:
          globalBlackboard()->set(name, parameter.as_string());
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          globalBlackboard()->set(name, static_cast<float>(parameter.as_double()));
          break;
        default:
          break;
      }
    };

  auto set_double_parameter_to_blackboard =
    [this](const std::string & name, double default_value) {
      node()->declare_parameter(name, default_value);
      double value = default_value;
      node()->get_parameter(name, value);
      globalBlackboard()->set(name, value);
    };

  auto set_pose_parameter_to_blackboard =
    [this](const std::string & name, const std::string & default_value) {
      node()->declare_parameter(name, default_value);
      std::string value;
      node()->get_parameter(name, value);
      try {
        globalBlackboard()->set(
          name, BT::convertFromString<geometry_msgs::msg::PoseStamped>(value));
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(
          node()->get_logger(), "Failed to parse pose parameter '%s'='%s': %s",
          name.c_str(), value.c_str(), ex.what());
        globalBlackboard()->set(
          name, BT::convertFromString<geometry_msgs::msg::PoseStamped>(default_value));
      }
    };

  set_parameter_to_blackboard("supply_ammo_min", rclcpp::ParameterValue(50));
  set_parameter_to_blackboard("supply_hp_min", rclcpp::ParameterValue(200));
  set_parameter_to_blackboard("supply_hp_recover", rclcpp::ParameterValue(400));
  set_pose_parameter_to_blackboard("supply_goal", "-0.9;-6.9;0");
  set_pose_parameter_to_blackboard("base_protect_goal", "0.5;-1.0;0");
  set_pose_parameter_to_blackboard("outpost_pressure_goal", "5.5;-4.1;0");
  set_pose_parameter_to_blackboard("patrol_point_a_goal", "5.5;-4.1;0");
  set_pose_parameter_to_blackboard("patrol_point_b_goal", "5.5;-2.5;0");
  set_parameter_to_blackboard(
    "post_outpost_behavior_mode", rclcpp::ParameterValue(std::string("default_patrol")));
  set_pose_parameter_to_blackboard("post_outpost_hold_goal", "5.5;-4.1;0");
  set_pose_parameter_to_blackboard(
    "enemy_outpost_gain_point", "12.43687629699707;3.2079238891601562;0");
  set_pose_parameter_to_blackboard("post_outpost_alt_point_a_goal", "5.5;-4.1;0");
  set_pose_parameter_to_blackboard("post_outpost_alt_point_b_goal", "5.5;-2.5;0");
  set_parameter_to_blackboard("single_goal_refresh_sec", rclcpp::ParameterValue(10.0));
  set_parameter_to_blackboard(
    "supply_our_half_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "supply_u_inner_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "supply_highland_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "supply_our_half_polygon", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "supply_u_inner_polygon", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard("rough_road_polygon", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard("highland_polygon", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard("base_protect_hp_threshold", rclcpp::ParameterValue(1500));
  set_double_parameter_to_blackboard("posture_cooldown_sec", 5.0);
  set_double_parameter_to_blackboard("posture_decay_sec", 180.0);
  set_double_parameter_to_blackboard("posture_budget_guard_sec", 170.0);
  set_double_parameter_to_blackboard("posture_damage_window_sec", 3.0);
  set_parameter_to_blackboard("posture_damage_threshold", rclcpp::ParameterValue(1));
  set_parameter_to_blackboard("posture_defense_hp_threshold", rclcpp::ParameterValue(200));
  set_parameter_to_blackboard("posture_warning_hp_min", rclcpp::ParameterValue(200));
  set_parameter_to_blackboard("posture_warning_hp_max", rclcpp::ParameterValue(300));
  set_double_parameter_to_blackboard("posture_enemy_stale_timeout_sec", 0.3);
  set_double_parameter_to_blackboard("posture_enemy_max_distance", 8.0);
  set_double_parameter_to_blackboard("posture_linger_sec", 2.0);
  set_parameter_to_blackboard(
    "base_protect_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "outpost_pressure_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard("patrol_phase_threshold", rclcpp::ParameterValue(180));
  set_parameter_to_blackboard(
    "patrol_early_route", rclcpp::ParameterValue(std::string{}));
  set_parameter_to_blackboard(
    "patrol_late_route", rclcpp::ParameterValue(std::string{}));
  globalBlackboard()->set("supply_route_locked", false);
  globalBlackboard()->set("selected_supply_route", std::string{});

  auto competition_sub = node()->create_subscription<robot_msgs::msg::CompetitionInfo>(
    "/competition_info", rclcpp::SystemDefaultsQoS(),
    [this](const robot_msgs::msg::CompetitionInfo::SharedPtr msg) {
      globalBlackboard()->set("referee_gameStatus", *msg);
      globalBlackboard()->set("referee_robotStatus", *msg);
      globalBlackboard()->set("game_state", static_cast<int>(msg->game_state));
      globalBlackboard()->set("our_sentry_hp", static_cast<int>(msg->our_sentry_hp));
      globalBlackboard()->set("remain_bullet", static_cast<int>(msg->remain_bullet));
      globalBlackboard()->set("our_base_hp", static_cast<int>(msg->our_base_hp));
      globalBlackboard()->set("our_outpost_hp", static_cast<int>(msg->our_outpost_hp));
      globalBlackboard()->set("enemy_outpost_hp", static_cast<int>(msg->enemy_outpost_hp));
      globalBlackboard()->set("stage_remain_time", static_cast<int>(msg->stage_remain_time));
    });
  subscriptions_.push_back(competition_sub);

  subscribe<robot_msgs::msg::RfidStatus>("/rfid_status", "referee_rfidStatus");

  auto detector_qos = rclcpp::SensorDataQoS();
  auto detector_sub = node()->create_subscription<auto_aim_interfaces::msg::Armors>(
    "detector/armors", detector_qos,
    [this](const auto_aim_interfaces::msg::Armors::SharedPtr msg) {
      globalBlackboard()->set("detector_armors", *msg);
      globalBlackboard()->set("detector_armors_rx_time_sec", node()->now().seconds());
    });
  subscriptions_.push_back(detector_sub);

  auto target_info_sub = node()->create_subscription<robot_msgs::msg::OmniPerception>(
    "/target_info", detector_qos,
    [this](const robot_msgs::msg::OmniPerception::SharedPtr msg) {
      globalBlackboard()->set("target_info", *msg);
      globalBlackboard()->set("target_info_rx_time_sec", node()->now().seconds());
    });
  subscriptions_.push_back(target_info_sub);

  auto tracker_qos = rclcpp::SensorDataQoS();
  subscribe<auto_aim_interfaces::msg::Target>("/tracker/target", "tracker_target", tracker_qos);

  auto costmap_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  subscribe<nav_msgs::msg::OccupancyGrid>(
    "global_costmap/costmap", "nav_globalCostmap", costmap_qos);
}

bool SentryBehaviorServer::onGoalReceived(
  const std::string & tree_name, const std::string & payload)
{
  RCLCPP_INFO(
    node()->get_logger(), "onGoalReceived with tree name '%s' with payload '%s'", tree_name.c_str(),
    payload.c_str());
  return true;
}

void SentryBehaviorServer::onTreeCreated(BT::Tree & tree)
{
  if (use_cout_logger_) {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
  }
  tick_count_ = 0;
}

std::optional<BT::NodeStatus> SentryBehaviorServer::onLoopAfterTick(BT::NodeStatus /*status*/)
{
  ++tick_count_;
  return std::nullopt;
}

std::optional<std::string> SentryBehaviorServer::onTreeExecutionCompleted(
  BT::NodeStatus status, bool was_cancelled)
{
  RCLCPP_INFO(
    node()->get_logger(), "onTreeExecutionCompleted with status=%d (canceled=%d) after %d ticks",
    static_cast<int>(status), was_cancelled, tick_count_);
  logger_cout_.reset();
  std::string result = treeName() +
                       " tree completed with status=" + std::to_string(static_cast<int>(status)) +
                       " after " + std::to_string(tick_count_) + " ticks";
  return result;
}

}  // namespace pb2025_sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<pb2025_sentry_behavior::SentryBehaviorServer>(options);

  RCLCPP_INFO(action_server->node()->get_logger(), "Starting SentryBehaviorServer");

  rclcpp::executors::MultiThreadedExecutor exec(
    rclcpp::ExecutorOptions(), 0, false, std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  // Groot2 editor requires a model of your registered Nodes.
  // You don't need to write that by hand, it can be automatically
  // generated using the following command.
  std::string xml_models = BT::writeTreeNodesModelXML(action_server->factory());

  // Save the XML models to a file
  std::ofstream file(std::filesystem::path(ROOT_DIR) / "behavior_trees" / "models.xml");
  file << xml_models;

  rclcpp::shutdown();
}
