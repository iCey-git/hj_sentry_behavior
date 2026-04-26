#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{

/// @brief 根据黑板中的 wp_idx 取出当前航点和等待时间。
/// 输入黑板键:
///   - waypoints      : std::vector<geometry_msgs::msg::PoseStamped>
///   - wait_times     : std::vector<double>
///   - wp_idx         : int
/// 输出黑板键:
///   - current_goal   : geometry_msgs::msg::PoseStamped
///   - current_wait_sec: double
class GetCurrentWaypointAction : public BT::SyncActionNode
{
public:
  GetCurrentWaypointAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__GET_CURRENT_WAYPOINT_HPP_
