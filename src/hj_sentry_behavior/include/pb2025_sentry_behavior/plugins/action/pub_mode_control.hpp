#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_MODE_CONTROL_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_MODE_CONTROL_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "robot_msgs/msg/mode_control.hpp"

namespace pb2025_sentry_behavior
{

class PublishModeControlAction
: public BT::RosTopicPubStatefulActionNode<robot_msgs::msg::ModeControl>
{
public:
  PublishModeControlAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(robot_msgs::msg::ModeControl & msg) override;

  bool setHaltMessage(robot_msgs::msg::ModeControl & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_MODE_CONTROL_HPP_
