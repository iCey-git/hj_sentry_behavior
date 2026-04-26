#include "pb2025_sentry_behavior/plugins/condition/is_game_status.hpp"

namespace pb2025_sentry_behavior
{

IsGameStatusCondition::IsGameStatusCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsGameStatusCondition::checkGameStart, this), config)
{
}

BT::NodeStatus IsGameStatusCondition::checkGameStart()
{
  int expected_game_progress;
  auto msg = getInput<robot_msgs::msg::CompetitionInfo>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("expected_game_progress", expected_game_progress);

  RCLCPP_DEBUG(
    logger_, "Checking: game_state(%d) == expected(%d)",
    static_cast<int>(msg->game_state), expected_game_progress);

  const bool is_progress_match = (msg->game_state == expected_game_progress);

  return is_progress_match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsGameStatusCondition::providedPorts()
{
  return {
    BT::InputPort<robot_msgs::msg::CompetitionInfo>(
      "key_port", "{@referee_gameStatus}", "CompetitionInfo port on blackboard"),
    BT::InputPort<int>("expected_game_progress", 4, "Expected game progress stage"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
