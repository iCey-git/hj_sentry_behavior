#include "pb2025_sentry_behavior/plugins/condition/is_status_ok.hpp"

namespace pb2025_sentry_behavior
{

IsStatusOKCondition::IsStatusOKCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusOKCondition::checkRobotStatus()
{
  int hp_min, hp_recover, ammo_min;
  auto msg = getInput<robot_msgs::msg::CompetitionInfo>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("hp_min", hp_min);
  getInput("hp_recover", hp_recover);
  getInput("ammo_min", ammo_min);

  const int hp = msg->our_sentry_hp;
  const bool is_ammo_ok = (msg->remain_bullet >= ammo_min);

  // 滞回逻辑：hp <= hp_min 触发回家，hp >= hp_recover 恢复正常
  if (!is_retreating_ && hp <= hp_min) {
    is_retreating_ = true;
  } else if (is_retreating_ && hp >= hp_recover) {
    is_retreating_ = false;
  }

  return (!is_retreating_ && is_ammo_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<robot_msgs::msg::CompetitionInfo>(
      "key_port", "{@referee_robotStatus}", "CompetitionInfo port on blackboard"),
    BT::InputPort<int>("hp_min", 200, "HP threshold to start retreat"),
    BT::InputPort<int>("hp_recover", 600, "HP threshold to stop retreat (full HP)"),
    BT::InputPort<int>("ammo_min", 0, "Lower then minimum ammo will return FAILURE")};
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsStatusOKCondition>("IsStatusOK");
}
