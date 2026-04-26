#include "pb2025_sentry_behavior/plugins/action/get_current_waypoint.hpp"

#include <vector>

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logging.hpp"

namespace pb2025_sentry_behavior
{

GetCurrentWaypointAction::GetCurrentWaypointAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList GetCurrentWaypointAction::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "waypoints", "LoadWaypoints 输出的航点列表"),
    BT::InputPort<std::vector<double>>("wait_times", "每个航点的等待时长(秒)"),
    BT::InputPort<int>("wp_idx", 0, "当前航点索引"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("current_goal", "当前目标位姿"),
    BT::OutputPort<double>("current_wait_sec", "当前航点等待秒数"),
  };
}

BT::NodeStatus GetCurrentWaypointAction::tick()
{
  auto waypoints = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("waypoints");
  auto wait_times = getInput<std::vector<double>>("wait_times");
  auto wp_idx = getInput<int>("wp_idx");

  if (!waypoints) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] waypoints port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!wait_times) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] wait_times port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!wp_idx) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] wp_idx port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto & wps = waypoints.value();
  const auto & wts = wait_times.value();
  const int idx = wp_idx.value();

  if (wps.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] waypoints list is empty", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (wps.size() != wts.size()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] waypoints size (%zu) does not match wait_times size (%zu)",
      name().c_str(), wps.size(), wts.size());
    return BT::NodeStatus::FAILURE;
  }

  if (idx < 0 || idx >= static_cast<int>(wps.size())) {
    RCLCPP_ERROR(
      rclcpp::get_logger("GetCurrentWaypoint"),
      "[%s] wp_idx=%d is out of range [0, %zu)",
      name().c_str(), idx, wps.size());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("current_goal", wps[idx]);
  setOutput("current_wait_sec", wts[idx]);

  RCLCPP_DEBUG(
    rclcpp::get_logger("GetCurrentWaypoint"),
    "[%s] waypoint[%d]: x=%.2f y=%.2f wait=%.1fs",
    name().c_str(), idx,
    wps[idx].pose.position.x, wps[idx].pose.position.y, wts[idx]);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::GetCurrentWaypointAction>("GetCurrentWaypoint");
}
