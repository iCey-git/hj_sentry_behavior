#include "pb2025_sentry_behavior/plugins/action/next_waypoint.hpp"

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logging.hpp"

namespace pb2025_sentry_behavior
{

NextWaypointAction::NextWaypointAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList NextWaypointAction::providedPorts()
{
  return {
    BT::InputPort<int>("total_waypoints", 0, "航点总数"),
    BT::BidirectionalPort<int>("wp_idx", "当前航点索引（读取后递增并写回）"),
  };
}

BT::NodeStatus NextWaypointAction::tick()
{
  auto total = getInput<int>("total_waypoints");
  auto wp_idx = getInput<int>("wp_idx");

  if (!total) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NextWaypoint"),
      "[%s] total_waypoints port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (!wp_idx) {
    RCLCPP_ERROR(
      rclcpp::get_logger("NextWaypoint"),
      "[%s] wp_idx port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const int total_wp = total.value();
  int idx = wp_idx.value();

  idx++;
  if (idx >= total_wp) {
    // 路线走完一圈，重置索引并通知上层
    RCLCPP_INFO(
      rclcpp::get_logger("NextWaypoint"),
      "[%s] route complete (total=%d), resetting to 0", name().c_str(), total_wp);
    setOutput("wp_idx", 0);
    return BT::NodeStatus::FAILURE;  // FAILURE 触发 KeepRunningUntilFailure 退出
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("NextWaypoint"),
    "[%s] advance to waypoint %d/%d", name().c_str(), idx, total_wp);
  setOutput("wp_idx", idx);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::NextWaypointAction>("NextWaypoint");
}
