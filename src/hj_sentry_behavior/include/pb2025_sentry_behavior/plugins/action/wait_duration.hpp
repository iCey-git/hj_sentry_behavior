#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/action_node.h"

namespace pb2025_sentry_behavior
{

/// @brief 异步等待指定秒数，可被父节点中断（onHalted 立即返回）。
/// 当 wait_sec <= 0 时直接返回 SUCCESS（跳过等待）。
class WaitDurationAction : public BT::StatefulActionNode
{
public:
  WaitDurationAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point deadline_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__WAIT_DURATION_HPP_
