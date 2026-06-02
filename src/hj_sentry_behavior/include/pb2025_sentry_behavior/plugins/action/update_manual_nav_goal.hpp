#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__UPDATE_MANUAL_NAV_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__UPDATE_MANUAL_NAV_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robot_msgs/msg/competition_info.hpp"

namespace pb2025_sentry_behavior
{

class UpdateManualNavGoalAction : public BT::SyncActionNode
{
public:
  UpdateManualNavGoalAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  bool updateLatchedGoal(
    const robot_msgs::msg::CompetitionInfo & competition_info,
    const std::string & our_side, double field_length_m, double field_width_m,
    double map_offset_x, double map_offset_y, double goal_yaw);

  bool has_latched_goal_ = false;
  double x_referee_ = 0.0;
  double y_referee_ = 0.0;
  geometry_msgs::msg::PoseStamped latched_goal_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__UPDATE_MANUAL_NAV_GOAL_HPP_
