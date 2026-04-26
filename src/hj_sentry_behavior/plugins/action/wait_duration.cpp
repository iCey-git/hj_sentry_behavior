#include "pb2025_sentry_behavior/plugins/action/wait_duration.hpp"

#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/logging.hpp"

namespace pb2025_sentry_behavior
{

WaitDurationAction::WaitDurationAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::StatefulActionNode(name, conf)
{
}

BT::PortsList WaitDurationAction::providedPorts()
{
  return {
    BT::InputPort<double>("wait_sec", 0.0, "等待时长（秒）。<=0 时跳过等待直接返回 SUCCESS"),
  };
}

BT::NodeStatus WaitDurationAction::onStart()
{
  auto wait_sec = getInput<double>("wait_sec");
  if (!wait_sec) {
    RCLCPP_ERROR(
      rclcpp::get_logger("WaitDuration"),
      "[%s] wait_sec port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const double secs = wait_sec.value();
  if (secs <= 0.0) {
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("WaitDuration"),
    "[%s] waiting %.1f seconds", name().c_str(), secs);

  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(secs));

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitDurationAction::onRunning()
{
  if (std::chrono::steady_clock::now() >= deadline_) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("WaitDuration"),
      "[%s] wait complete", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitDurationAction::onHalted()
{
  RCLCPP_DEBUG(
    rclcpp::get_logger("WaitDuration"),
    "[%s] wait halted", name().c_str());
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::WaitDurationAction>("WaitDuration");
}
