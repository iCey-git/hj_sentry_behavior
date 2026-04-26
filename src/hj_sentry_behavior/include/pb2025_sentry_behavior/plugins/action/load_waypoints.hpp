#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{

/// @brief 从 CSV 文件加载多点路线到黑板。
/// 支持两种 CSV:
///   - 简化格式: x,y,yaw,wait_sec
///   - COD waypoint_editor: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command...
/// 成功后向黑板写入:
///   - waypoints      : std::vector<geometry_msgs::msg::PoseStamped>
///   - wait_times     : std::vector<double>
///   - total_waypoints: int
///   - wp_idx         : int  (重置为 0)
class LoadWaypointsAction : public BT::SyncActionNode
{
public:
  LoadWaypointsAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  std::string last_loaded_file_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__LOAD_WAYPOINTS_HPP_
