#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_

#include <chrono>
#include <optional>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/tree_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace pb2025_sentry_behavior
{
class TickAfterTimeout : public BT::DecoratorNode
{
public:
  TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<float>("timeout", "time in s to wait before ticking child again"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>(
        "goal", "Optional goal that triggers the child immediately when changed")};
  }

private:
  std::chrono::duration<float> timeout_;
  std::chrono::time_point<std::chrono::steady_clock> last_success_time_;
  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;

  BT::NodeStatus tick() override;
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__DECORATOR__TICK_AFTER_TIMEOUT_NODE_HPP_
