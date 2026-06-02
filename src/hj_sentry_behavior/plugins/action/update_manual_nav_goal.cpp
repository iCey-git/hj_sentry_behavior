#include "pb2025_sentry_behavior/plugins/action/update_manual_nav_goal.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kCoordinateEpsilon = 1e-4;

bool isFinite(double value)
{
  return std::isfinite(value);
}

bool nearlyEqual(double lhs, double rhs)
{
  return std::abs(lhs - rhs) <= kCoordinateEpsilon;
}

std::string normalizeSide(std::string side)
{
  std::transform(
    side.begin(), side.end(), side.begin(),
    [](unsigned char ch) { return static_cast<char>(std::tolower(ch)); });
  return side;
}

}  // namespace

UpdateManualNavGoalAction::UpdateManualNavGoalAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList UpdateManualNavGoalAction::providedPorts()
{
  return {
    BT::InputPort<robot_msgs::msg::CompetitionInfo>(
      "competition_port", "{@referee_robotStatus}", "CompetitionInfo port on blackboard"),
    BT::InputPort<std::string>("our_side", "{@our_side}", "Current alliance side: red/blue/auto"),
    BT::InputPort<double>("field_length_m", 28.0, "Field length in meters"),
    BT::InputPort<double>("field_width_m", 15.0, "Field width in meters"),
    BT::InputPort<double>(
      "map_offset_x", 0.0, "Offset from remapped field x to map x in meters"),
    BT::InputPort<double>(
      "map_offset_y", 0.0, "Offset from remapped field y to map y in meters"),
    BT::InputPort<double>("goal_yaw", 0.0, "Yaw of the latched manual nav goal in radians"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "{manual_nav_goal}", "Latched manual navigation goal in map frame"),
    BT::OutputPort<bool>(
      "goal_set", "{manual_nav_goal_set}", "True when a manual navigation goal has been latched"),
  };
}

BT::NodeStatus UpdateManualNavGoalAction::tick()
{
  const auto competition_info = getInput<robot_msgs::msg::CompetitionInfo>("competition_port");
  const auto our_side = getInput<std::string>("our_side");
  const auto field_length_m = getInput<double>("field_length_m");
  const auto field_width_m = getInput<double>("field_width_m");
  const auto map_offset_x = getInput<double>("map_offset_x");
  const auto map_offset_y = getInput<double>("map_offset_y");
  const auto goal_yaw = getInput<double>("goal_yaw");

  static const auto logger = rclcpp::get_logger("UpdateManualNavGoal");

  if (
    !competition_info || !our_side || !field_length_m || !field_width_m || !map_offset_x ||
    !map_offset_y || !goal_yaw)
  {
    RCLCPP_ERROR(logger, "[%s] missing required ports", name().c_str());
    setOutput("goal_set", false);
    return BT::NodeStatus::FAILURE;
  }

  const auto & msg = competition_info.value();
  const bool has_active_point =
    msg.is_target_active == 1 && isFinite(msg.target_position_x) && isFinite(msg.target_position_y);

  if (has_active_point) {
    const bool is_new_point =
      !has_latched_goal_ || !nearlyEqual(msg.target_position_x, x_referee_) ||
      !nearlyEqual(msg.target_position_y, y_referee_);

    if (is_new_point) {
      if (!updateLatchedGoal(
            msg, our_side.value(), field_length_m.value(), field_width_m.value(),
            map_offset_x.value(), map_offset_y.value(), goal_yaw.value()))
      {
        if (!has_latched_goal_) {
          setOutput("goal_set", false);
          return BT::NodeStatus::FAILURE;
        }
        RCLCPP_WARN(
          logger,
          "[%s] failed to update manual nav goal, keeping previous latched goal",
          name().c_str());
      }
    }
  }

  if (!has_latched_goal_) {
    setOutput("goal_set", false);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("goal", latched_goal_);
  setOutput("goal_set", true);
  return BT::NodeStatus::SUCCESS;
}

bool UpdateManualNavGoalAction::updateLatchedGoal(
  const robot_msgs::msg::CompetitionInfo & competition_info,
  const std::string & our_side, double field_length_m, double field_width_m,
  double map_offset_x, double map_offset_y, double goal_yaw)
{
  static const auto logger = rclcpp::get_logger("UpdateManualNavGoal");

  const std::string normalized_side = normalizeSide(our_side);
  if (normalized_side == "auto") {
    RCLCPP_WARN(
      logger, "[%s] our_side=auto is not implemented for manual nav goal", name().c_str());
    return false;
  }
  if (normalized_side != "red" && normalized_side != "blue") {
    RCLCPP_WARN(
      logger, "[%s] invalid our_side '%s' for manual nav goal",
      name().c_str(), our_side.c_str());
    return false;
  }

  const double x_referee = competition_info.target_position_x;
  const double y_referee = competition_info.target_position_y;

  double x_remapped = x_referee;
  double y_remapped = y_referee;
  if (normalized_side == "blue") {
    x_remapped = field_length_m - x_referee;
    y_remapped = field_width_m - y_referee;
  }

  const double x_map = x_remapped + map_offset_x;
  const double y_map = y_remapped + map_offset_y;

  tf2::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, goal_yaw);

  latched_goal_.header.frame_id = "map";
  latched_goal_.header.stamp = rclcpp::Time(0, 0, RCL_ROS_TIME);
  latched_goal_.pose.position.x = x_map;
  latched_goal_.pose.position.y = y_map;
  latched_goal_.pose.position.z = 0.0;
  latched_goal_.pose.orientation = tf2::toMsg(quaternion);

  has_latched_goal_ = true;
  x_referee_ = x_referee;
  y_referee_ = y_referee;

  RCLCPP_INFO(
    logger,
    "[%s] latched manual nav goal referee=[%.3f, %.3f] remapped=[%.3f, %.3f] map=[%.3f, %.3f] side=%s",
    name().c_str(), x_referee, y_referee, x_remapped, y_remapped, x_map, y_map,
    normalized_side.c_str());

  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::UpdateManualNavGoalAction>(
    "UpdateManualNavGoal");
}
