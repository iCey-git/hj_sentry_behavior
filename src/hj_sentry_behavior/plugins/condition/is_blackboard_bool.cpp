#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

class IsBlackboardBool : public BT::ConditionNode
{
public:
  IsBlackboardBool(const std::string & name, const BT::NodeConfig & conf)
  : BT::ConditionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("value", false, "Blackboard bool value"),
      BT::InputPort<bool>("expected", true, "Expected bool value"),
    };
  }

  BT::NodeStatus tick() override
  {
    bool value = false;
    bool expected = true;
    getInput("value", value);
    getInput("expected", expected);
    return value == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

}  // namespace pb2025_sentry_behavior

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsBlackboardBool>("IsBlackboardBool");
}
