#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_TWIST_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_TWIST_HPP_

#include <string>

#include "behaviortree_ros2/bt_topic_pub_action_node.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace pb2025_sentry_behavior
{

class PublishTwistAction : public BT::RosTopicPubStatefulActionNode<geometry_msgs::msg::Twist>
{
public:
  PublishTwistAction(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::Twist & msg) override;

  bool setHaltMessage(geometry_msgs::msg::Twist & msg) override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_TWIST_HPP_
