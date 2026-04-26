#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_

#include <memory>
#include <string>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

namespace pb2025_sentry_behavior
{
class SendNav2GoalAction : public BT::RosActionNode<nav2_msgs::action::NavigateToPose>
{
public:
  SendNav2GoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setGoal(Goal & goal) override;

  void onHalt() override;

  BT::NodeStatus onResultReceived(const WrappedResult & wr) override;

  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SEND_NAV2_GOAL_HPP_
