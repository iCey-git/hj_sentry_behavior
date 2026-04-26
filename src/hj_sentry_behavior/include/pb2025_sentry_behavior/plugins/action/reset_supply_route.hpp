#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__RESET_SUPPLY_ROUTE_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__RESET_SUPPLY_ROUTE_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"

namespace pb2025_sentry_behavior
{

class ResetSupplyRouteAction : public BT::SyncActionNode
{
public:
  ResetSupplyRouteAction(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__RESET_SUPPLY_ROUTE_HPP_
