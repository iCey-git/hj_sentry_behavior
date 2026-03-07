// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
    BT::InputPort<int>("min_remain_time", 0, "Minimum remaining time (s)"),
    BT::InputPort<int>("max_remain_time", 420, "Maximum remaining time (s)"),
  };
}
}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::IsGameStatusCondition>("IsGameStatus");
}
