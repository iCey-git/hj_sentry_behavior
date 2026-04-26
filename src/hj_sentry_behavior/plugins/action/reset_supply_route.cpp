#include "pb2025_sentry_behavior/plugins/action/reset_supply_route.hpp"

namespace pb2025_sentry_behavior
{

ResetSupplyRouteAction::ResetSupplyRouteAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

BT::PortsList ResetSupplyRouteAction::providedPorts()
{
  return {
    BT::OutputPort<bool>("supply_route_locked", "补给路线锁定标志"),
    BT::OutputPort<std::string>("selected_supply_route", "已选补给路线"),
  };
}

BT::NodeStatus ResetSupplyRouteAction::tick()
{
  setOutput("supply_route_locked", false);
  setOutput("selected_supply_route", std::string{});
  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::ResetSupplyRouteAction>("ResetSupplyRoute");
}
