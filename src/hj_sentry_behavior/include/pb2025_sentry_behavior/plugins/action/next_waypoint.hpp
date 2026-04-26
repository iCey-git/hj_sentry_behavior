#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace pb2025_sentry_behavior
{

/// @brief 将 wp_idx 递增。若到达末尾（wp_idx >= total_waypoints）则
/// 重置为 0 并返回 FAILURE，通知上层路线已完整走完一圈。
/// 正常递增时返回 SUCCESS。
class NextWaypointAction : public BT::SyncActionNode
{
public:
  NextWaypointAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NEXT_WAYPOINT_HPP_
