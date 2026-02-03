#include "pb2025_sentry_behavior/plugins/action/pub_mode_control.hpp"

#include <algorithm>

namespace pb2025_sentry_behavior
{

namespace
{
uint8_t clamp_to_u8(int value)
{
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}
}  // namespace

PublishModeControlAction::PublishModeControlAction(
  const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
: RosTopicPubStatefulActionNode(name, config, params)
{
}

BT::PortsList PublishModeControlAction::providedPorts()
{
  return providedBasicPorts(
    {BT::InputPort<int>("chassis_gyro", 0, "0: off, 1: on"),
     BT::InputPort<int>("patrol_mode", 2, "Patrol mode (e.g. 2=normal, 255=relax)"),
     BT::InputPort<int>("power_mode", 12, "Power mode (e.g. 12=charge, 13=boost)")});
}

bool PublishModeControlAction::setMessage(robot_msgs::msg::ModeControl & msg)
{
  int chassis_gyro = 0;
  int patrol_mode = 0;
  int power_mode = 0;
  
  getInput("chassis_gyro", chassis_gyro);
  getInput("patrol_mode", patrol_mode);
  getInput("power_mode", power_mode);

  msg.chassis_gyro = clamp_to_u8(chassis_gyro);
  msg.patrol_mode = clamp_to_u8(patrol_mode);
  msg.power_mode = clamp_to_u8(power_mode);
  return true;
}

bool PublishModeControlAction::setHaltMessage(robot_msgs::msg::ModeControl & msg)
{
  msg.chassis_gyro = 0;
  msg.patrol_mode = 0;
  msg.power_mode = 0;
  return true;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishModeControlAction, "PublishModeControl");
