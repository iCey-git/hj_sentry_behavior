#include <gtest/gtest.h>

#include <map>
#include <string>

#include "behaviortree_cpp/tree_node.h"
#include "pb2025_sentry_behavior/plugins/action/update_manual_nav_goal.hpp"
#include "robot_msgs/msg/competition_info.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

robot_msgs::msg::CompetitionInfo makeCompetitionInfo(float x, float y, int8_t active)
{
  robot_msgs::msg::CompetitionInfo msg;
  msg.target_position_x = x;
  msg.target_position_y = y;
  msg.is_target_active = active;
  return msg;
}

BT::NodeConfig makeConfig()
{
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();
  config.input_ports = {
    {"competition_port", "{competition_msg}"},
    {"our_side", "{our_side_value}"},
    {"field_length_m", "28.0"},
    {"field_width_m", "15.0"},
    {"map_offset_x", "-3.68"},
    {"map_offset_y", "-8.46"},
    {"goal_yaw", "0.0"},
  };
  config.output_ports = {
    {"goal", "{goal_msg}"},
    {"goal_set", "{goal_set_value}"},
  };
  return config;
}

}  // namespace

TEST(UpdateManualNavGoalActionTest, LatchesFirstPointAndKeepsItWhenInactive)
{
  auto config = makeConfig();
  config.blackboard->set("competition_msg", makeCompetitionInfo(10.0F, 5.0F, 1));
  config.blackboard->set("our_side_value", std::string("red"));

  UpdateManualNavGoalAction node("UpdateManualNavGoal", config);

  EXPECT_EQ(node.executeTick(), BT::NodeStatus::SUCCESS);

  auto goal = config.blackboard->get<geometry_msgs::msg::PoseStamped>("goal_msg");
  auto goal_set = config.blackboard->get<bool>("goal_set_value");
  EXPECT_TRUE(goal_set);
  EXPECT_NEAR(goal.pose.position.x, 6.32, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, -3.46, 1e-6);

  config.blackboard->set("competition_msg", makeCompetitionInfo(99.0F, 99.0F, 0));
  EXPECT_EQ(node.executeTick(), BT::NodeStatus::SUCCESS);
  goal = config.blackboard->get<geometry_msgs::msg::PoseStamped>("goal_msg");
  EXPECT_NEAR(goal.pose.position.x, 6.32, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, -3.46, 1e-6);
}

TEST(UpdateManualNavGoalActionTest, ReplacesLatchedGoalWhenNewPointArrives)
{
  auto config = makeConfig();
  config.blackboard->set("competition_msg", makeCompetitionInfo(10.0F, 5.0F, 1));
  config.blackboard->set("our_side_value", std::string("blue"));

  UpdateManualNavGoalAction node("UpdateManualNavGoal", config);

  EXPECT_EQ(node.executeTick(), BT::NodeStatus::SUCCESS);
  auto goal = config.blackboard->get<geometry_msgs::msg::PoseStamped>("goal_msg");
  EXPECT_NEAR(goal.pose.position.x, 14.32, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, 1.54, 1e-6);

  config.blackboard->set("competition_msg", makeCompetitionInfo(12.0F, 4.0F, 1));
  EXPECT_EQ(node.executeTick(), BT::NodeStatus::SUCCESS);
  goal = config.blackboard->get<geometry_msgs::msg::PoseStamped>("goal_msg");
  EXPECT_NEAR(goal.pose.position.x, 12.32, 1e-6);
  EXPECT_NEAR(goal.pose.position.y, 2.54, 1e-6);
}

}  // namespace pb2025_sentry_behavior
