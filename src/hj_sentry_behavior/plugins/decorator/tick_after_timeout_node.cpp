#include "pb2025_sentry_behavior/plugins/decorator/tick_after_timeout_node.hpp"

#include <cmath>

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kGoalEpsilon = 1e-6;

bool nearlyEqual(double lhs, double rhs) { return std::abs(lhs - rhs) <= kGoalEpsilon; }

bool goalsEqual(
  const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs)
{
  return lhs.header.frame_id == rhs.header.frame_id &&
         nearlyEqual(lhs.pose.position.x, rhs.pose.position.x) &&
         nearlyEqual(lhs.pose.position.y, rhs.pose.position.y) &&
         nearlyEqual(lhs.pose.position.z, rhs.pose.position.z) &&
         nearlyEqual(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
         nearlyEqual(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
         nearlyEqual(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
         nearlyEqual(lhs.pose.orientation.w, rhs.pose.orientation.w);
}

}  // namespace

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

  bool goal_changed = false;
  if (const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal")) {
    goal_changed = last_goal_.has_value() && !goalsEqual(goal.value(), last_goal_.value());
    last_goal_ = goal.value();
  }

  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_success_time_);

  if (!goal_changed && dt < timeout_) {
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
