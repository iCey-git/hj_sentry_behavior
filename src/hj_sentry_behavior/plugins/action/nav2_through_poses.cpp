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

#include "pb2025_sentry_behavior/plugins/action/nav2_through_poses.hpp"

#include <cmath>

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kGoalEpsilon = 1e-6;

bool nearlyEqual(double lhs, double rhs)
{
  return std::abs(lhs - rhs) <= kGoalEpsilon;
}

bool posesEqual(
  const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs)
{
  return
    nearlyEqual(lhs.pose.position.x, rhs.pose.position.x) &&
    nearlyEqual(lhs.pose.position.y, rhs.pose.position.y) &&
    nearlyEqual(lhs.pose.position.z, rhs.pose.position.z) &&
    nearlyEqual(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
    nearlyEqual(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
    nearlyEqual(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
    nearlyEqual(lhs.pose.orientation.w, rhs.pose.orientation.w);
}

}  // namespace

Nav2ThroughPosesAction::Nav2ThroughPosesAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  action_name_(params.default_port_value.empty() ? "navigate_through_poses" : params.default_port_value),
  goal_pose_topic_("/goal_pose"),
  server_timeout_(params.server_timeout),
  wait_for_server_timeout_(params.wait_for_server_timeout)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
  goal_pose_publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
}

BT::PortsList Nav2ThroughPosesAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "navigate_through_poses", "Action server name"),
    BT::InputPort<std::string>("goal_pose_topic", "/goal_pose", "Final goal pose topic"),
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "Waypoint list"),
  };
}

BT::NodeStatus Nav2ThroughPosesAction::onStart()
{
  auto goals = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
  if (!goals || goals->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goals are not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto action_name = getInput<std::string>("action_name");
  if (!action_name) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] action_name is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto goal_pose_topic = getInput<std::string>("goal_pose_topic");
  if (!goal_pose_topic) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal_pose_topic is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }
  if (goal_pose_topic_ != goal_pose_topic.value()) {
    goal_pose_topic_ = goal_pose_topic.value();
    goal_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
  }

  if (!createClient(action_name.value())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!sendGoal(goals.value())) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2ThroughPosesAction::onRunning()
{
  callback_executor_.spin_some();

  auto goals = getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
  if (!goals || goals->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goals are not set while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal handle is empty while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!goalsEqual(goals.value(), active_goals_)) {
    RCLCPP_INFO(node_->get_logger(), "[%s] through-poses goal updated", name().c_str());
    if (!sendGoal(goals.value())) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (result_future_.valid() &&
    result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
    const auto result = result_future_.get();
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "[%s] through-poses navigation succeeded", name().c_str());
        resetGoalState();
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "[%s] through-poses navigation aborted", name().c_str());
        resetGoalState();
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "[%s] through-poses navigation canceled", name().c_str());
        resetGoalState();
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::UNKNOWN:
        break;
    }
  }

  switch (goal_handle_->get_status()) {
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      return BT::NodeStatus::RUNNING;

    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "[%s] through-poses navigation succeeded", name().c_str());
      resetGoalState();
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "[%s] through-poses navigation aborted", name().c_str());
      resetGoalState();
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(node_->get_logger(), "[%s] through-poses navigation canceled", name().c_str());
      resetGoalState();
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_WARN(
        node_->get_logger(), "[%s] unexpected goal handle status: %d", name().c_str(),
        goal_handle_->get_status());
      return BT::NodeStatus::RUNNING;
  }
}

void Nav2ThroughPosesAction::onHalted()
{
  callback_executor_.spin_some();

  if (goal_handle_) {
    const auto status = goal_handle_->get_status();
    if (
      status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
      status == action_msgs::msg::GoalStatus::STATUS_EXECUTING ||
      status == action_msgs::msg::GoalStatus::STATUS_CANCELING)
    {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (
        callback_executor_.spin_until_future_complete(cancel_future, server_timeout_) !=
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node_->get_logger(), "[%s] failed to cancel through-poses goal", name().c_str());
      } else {
        RCLCPP_INFO(
          node_->get_logger(), "[%s] through-poses goal canceled", name().c_str());
      }
    }
  }

  resetGoalState();
}

bool Nav2ThroughPosesAction::createClient(const std::string & action_name)
{
  if (action_client_ && action_name_ == action_name) {
    return true;
  }

  action_name_ = action_name;
  action_client_ =
    rclcpp_action::create_client<NavigateThroughPoses>(node_, action_name_, callback_group_);

  if (!action_client_->wait_for_action_server(wait_for_server_timeout_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] action server '%s' is not reachable", name().c_str(),
      action_name_.c_str());
    return false;
  }

  return true;
}

bool Nav2ThroughPosesAction::sendGoal(const std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  NavigateThroughPoses::Goal navigation_goal;
  navigation_goal.poses = goals;
  for (auto & pose : navigation_goal.poses) {
    pose.header.frame_id = "map";
    pose.header.stamp = node_->now();
  }
  navigation_goal.behavior_tree.clear();
  publishFinalGoalPose(navigation_goal.poses);

  auto future_goal_handle = action_client_->async_send_goal(navigation_goal);
  if (
    callback_executor_.spin_until_future_complete(future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] send through-poses goal failed", name().c_str());
    return false;
  }

  auto goal_handle = future_goal_handle.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] through-poses goal was rejected by nav2", name().c_str());
    return false;
  }

  goal_handle_ = goal_handle;
  active_goals_ = navigation_goal.poses;
  result_future_ = action_client_->async_get_result(goal_handle_);

  const auto & first_goal = active_goals_.front().pose.position;
  const auto & last_goal = active_goals_.back().pose.position;
  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] navigating through %zu poses from [%.3f, %.3f] to [%.3f, %.3f]",
    name().c_str(), active_goals_.size(), first_goal.x, first_goal.y, last_goal.x, last_goal.y);

  return true;
}

void Nav2ThroughPosesAction::publishFinalGoalPose(
  const std::vector<geometry_msgs::msg::PoseStamped> & goals)
{
  if (!goal_pose_publisher_ || goals.empty()) {
    return;
  }

  goal_pose_publisher_->publish(goals.back());
}

bool Nav2ThroughPosesAction::goalsEqual(
  const std::vector<geometry_msgs::msg::PoseStamped> & lhs,
  const std::vector<geometry_msgs::msg::PoseStamped> & rhs)
{
  if (lhs.size() != rhs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    if (!posesEqual(lhs[i], rhs[i])) {
      return false;
    }
  }
  return true;
}

void Nav2ThroughPosesAction::resetGoalState()
{
  goal_handle_.reset();
  active_goals_.clear();
  result_future_ = {};
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::Nav2ThroughPosesAction>(
    "Nav2ThroughPoses", params);
}
