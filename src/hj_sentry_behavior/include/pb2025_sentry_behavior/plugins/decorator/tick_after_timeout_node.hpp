#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/tree_node.h"

namespace pb2025_sentry_behavior
{
class TickAfterTimeout : public BT::DecoratorNode
{
public:
  TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>("timeout", "time in s to wait before ticking child again")};
  }

private:
  std::chrono::duration<float> timeout_;
  std::chrono::time_point<std::chrono::steady_clock> last_success_time_;

  BT::NodeStatus tick() override;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_
