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
    {BT::InputPort<int>("chassis_gyro", 0, "0: off, 1: on, 2: alignment correction"),
     BT::InputPort<int>("patrol_mode", 2, "Patrol mode (e.g. 2=normal, 255=relax)"),
     BT::InputPort<int>("power_mode", 12, "Power mode (e.g. 12=charge, 13=boost)"),
     BT::InputPort<bool>("rough_road_active", false, "Override chassis gyro on rough road"),
     BT::InputPort<int>("rough_chassis_gyro", 0, "Chassis gyro value on rough road")});
}

bool PublishModeControlAction::setMessage(robot_msgs::msg::ModeControl & msg)
{
  int chassis_gyro = 0;
  int patrol_mode = 0;
  int power_mode = 0;
  bool rough_road_active = false;
  int rough_chassis_gyro = 0;
  
  getInput("chassis_gyro", chassis_gyro);
  getInput("patrol_mode", patrol_mode);
  getInput("power_mode", power_mode);
  getInput("rough_road_active", rough_road_active);
  getInput("rough_chassis_gyro", rough_chassis_gyro);
  if (rough_road_active) {
    chassis_gyro = rough_chassis_gyro;
  }

  msg.chassis_gyro = static_cast<uint16_t>(std::clamp(chassis_gyro, 0, 65535));
  msg.patrol_mode = clamp_to_u8(patrol_mode);
  msg.power_mode = clamp_to_u8(power_mode);
  return true;
}

bool PublishModeControlAction::setHaltMessage(robot_msgs::msg::ModeControl & msg)
{
  // Branch switching in reactive trees can halt this node frequently.
  // Publishing a zeroed mode command here causes unintended mode flicker.
  (void)msg;
  return false;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(pb2025_sentry_behavior::PublishModeControlAction, "PublishModeControl");
