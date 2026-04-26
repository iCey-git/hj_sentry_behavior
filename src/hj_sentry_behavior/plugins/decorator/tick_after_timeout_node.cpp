#include "pb2025_sentry_behavior/plugins/decorator/tick_after_timeout_node.hpp"

namespace pb2025_sentry_behavior
{

TickAfterTimeout::TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
  this->last_success_time_ = std::chrono::steady_clock::now();
}

BT::NodeStatus TickAfterTimeout::tick()
{
  float timeout;
  if (!this->getInput<float>("timeout", timeout)) {
    throw(BT::RuntimeError("[", this->name(), "] Failed to get input [timeout]"));
  }

  timeout_ = std::chrono::duration<float>(timeout);

  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_success_time_);

  if (dt < timeout_) {
    return BT::NodeStatus::SKIPPED;
  }

  this->setStatus(BT::NodeStatus::RUNNING);
  auto child_status = this->child()->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS) {
    last_success_time_ = std::chrono::steady_clock::now();
  }

  if (child_status != BT::NodeStatus::RUNNING) {
    this->resetChild();
  }

  return child_status;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::TickAfterTimeout>("TickAfterTimeout");
}
