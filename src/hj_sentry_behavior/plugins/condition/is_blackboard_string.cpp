#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/bt_factory.h"

namespace pb2025_sentry_behavior
{

class IsBlackboardString : public BT::ConditionNode
{
public:
  IsBlackboardString(const std::string & name, const BT::NodeConfig & conf)
  : BT::ConditionNode(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("value", "", "Blackboard string value"),
      BT::InputPort<std::string>("expected", "", "Expected string value"),
    };
  }

  BT::NodeStatus tick() override
  {
    std::string value;
    std::string expected;
    getInput("value", value);
    getInput("expected", expected);
    return value == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

} 

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsBlackboardString>("IsBlackboardString");
}
