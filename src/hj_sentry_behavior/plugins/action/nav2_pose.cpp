#include "pb2025_sentry_behavior/plugins/action/nav2_pose.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "action_msgs/msg/goal_status.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kGoalEpsilon = 1e-6;

bool nearlyEqual(double lhs, double rhs) { return std::abs(lhs - rhs) <= kGoalEpsilon; }

}  // namespace

Nav2PoseAction::Nav2PoseAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::StatefulActionNode(name, conf),
  action_name_(params.default_port_value.empty() ? "navigate_to_pose" : params.default_port_value),
  goal_pose_topic_("/goal_pose"),
  cmd_vel_topic_("cmd_vel"),
  server_timeout_(params.server_timeout),
  wait_for_server_timeout_(params.wait_for_server_timeout)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }

  callback_group_ =
    node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  goal_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
  cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
}

BT::PortsList Nav2PoseAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "navigate_to_pose", "Action server name"),
    BT::InputPort<std::string>(
      "goal_pose_topic", "/goal_pose", "Goal pose topic for yaw controller"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "Twist topic used for halt command"),
  };
}

BT::NodeStatus Nav2PoseAction::onStart()
{
  goal_sent_ = false;
  last_send_attempt_sec_ = 0.0;

  const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const auto action_name = getInput<std::string>("action_name");
  if (!action_name) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] action_name is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!configurePublishers()) {
    return BT::NodeStatus::FAILURE;
  }

  target_goal_ = goal.value();
  last_send_attempt_sec_ = node_->now().seconds();
  if (!createClient(action_name.value())) {
    return BT::NodeStatus::RUNNING;
  }

  if (!sendGoal(target_goal_)) {
    return BT::NodeStatus::RUNNING;
  }

  goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2PoseAction::onRunning()
{
  callback_executor_.spin_some();

  if (!goal_sent_) {
    const double now = node_->now().seconds();
    if (now - last_send_attempt_sec_ < 2.0) {
      return BT::NodeStatus::RUNNING;
    }

    last_send_attempt_sec_ = now;
    if (!createClient(action_name_)) {
      return BT::NodeStatus::RUNNING;
    }

    const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
    if (!goal) {
      return BT::NodeStatus::FAILURE;
    }
    target_goal_ = goal.value();
    if (sendGoal(target_goal_)) {
      goal_sent_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }

  const auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal is not set while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!goalsEqual(goal.value(), target_goal_)) {
    RCLCPP_INFO(node_->get_logger(), "[%s] navigation goal updated", name().c_str());

    goal_sent_ = false;
    cancelActiveGoal();
    resetGoalState();
    target_goal_ = goal.value();
    last_send_attempt_sec_ = node_->now().seconds();

    if (!configurePublishers() || !createClient(action_name_)) {
      return BT::NodeStatus::RUNNING;
    }

    if (sendGoal(target_goal_)) {
      goal_sent_ = true;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal handle is empty while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (
    result_future_.valid() &&
    result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
    const auto result = result_future_.get();
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(node_->get_logger(), "[%s] navigation succeeded", name().c_str());
        resetGoalState();
        return BT::NodeStatus::SUCCESS;

      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "[%s] navigation aborted", name().c_str());
        resetGoalState();
        return BT::NodeStatus::FAILURE;

      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(node_->get_logger(), "[%s] navigation canceled", name().c_str());
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
      RCLCPP_INFO(node_->get_logger(), "[%s] navigation succeeded", name().c_str());
      resetGoalState();
      return BT::NodeStatus::SUCCESS;

    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "[%s] navigation aborted", name().c_str());
      resetGoalState();
      return BT::NodeStatus::FAILURE;

    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_WARN(node_->get_logger(), "[%s] navigation canceled", name().c_str());
      resetGoalState();
      return BT::NodeStatus::FAILURE;

    default:
      RCLCPP_WARN(
        node_->get_logger(), "[%s] unexpected goal handle status: %d", name().c_str(),
        goal_handle_->get_status());
      return BT::NodeStatus::RUNNING;
  }
}

void Nav2PoseAction::onHalted()
{
  callback_executor_.spin_some();
  publishZeroTwist();
  cancelActiveGoal();
  resetGoalState();
  goal_sent_ = false;
  last_send_attempt_sec_ = 0.0;
}

bool Nav2PoseAction::configurePublishers()
{
  const auto goal_pose_topic = getInput<std::string>("goal_pose_topic");
  if (!goal_pose_topic || goal_pose_topic->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal_pose_topic is not set", name().c_str());
    return false;
  }
  if (goal_pose_topic_ != goal_pose_topic.value()) {
    goal_pose_topic_ = goal_pose_topic.value();
    goal_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
  }

  const auto cmd_vel_topic = getInput<std::string>("cmd_vel_topic");
  if (!cmd_vel_topic || cmd_vel_topic->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] cmd_vel_topic is not set", name().c_str());
    return false;
  }
  if (cmd_vel_topic_ != cmd_vel_topic.value()) {
    cmd_vel_topic_ = cmd_vel_topic.value();
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  return true;
}

bool Nav2PoseAction::createClient(const std::string & action_name)
{
  if (!action_client_ || action_name_ != action_name) {
    action_name_ = action_name;
    action_client_ =
      rclcpp_action::create_client<NavigateToPose>(node_, action_name_, callback_group_);
  }

  if (
    !action_client_->action_server_is_ready() &&
    !action_client_->wait_for_action_server(wait_for_server_timeout_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] action server '%s' is not reachable", name().c_str(),
      action_name_.c_str());
    return false;
  }

  return true;
}

bool Nav2PoseAction::sendGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  NavigateToPose::Goal navigation_goal;
  navigation_goal.pose = goal;
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = node_->now();
  publishGoalPose(navigation_goal.pose);

  try {
    auto future_goal_handle = action_client_->async_send_goal(navigation_goal);
    if (
      callback_executor_.spin_until_future_complete(future_goal_handle, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] send goal failed", name().c_str());
      return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] goal was rejected by nav2", name().c_str());
      return false;
    }

    goal_handle_ = goal_handle;
    active_goal_ = navigation_goal.pose;
    result_future_ = action_client_->async_get_result(goal_handle_);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] send goal exception: %s", name().c_str(), ex.what());
    return false;
  }

  RCLCPP_INFO(
    node_->get_logger(), "[%s] navigating to pose [%.3f, %.3f]", name().c_str(),
    active_goal_.pose.position.x, active_goal_.pose.position.y);
  return true;
}

void Nav2PoseAction::publishGoalPose(const geometry_msgs::msg::PoseStamped & goal)
{
  if (goal_pose_publisher_) {
    goal_pose_publisher_->publish(goal);
  }
}

bool Nav2PoseAction::cancelActiveGoal()
{
  if (!goal_handle_) {
    return true;
  }

  const auto status = goal_handle_->get_status();
  if (
    status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
    status != action_msgs::msg::GoalStatus::STATUS_EXECUTING &&
    status != action_msgs::msg::GoalStatus::STATUS_CANCELING) {
    return true;
  }

  try {
    auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
    if (
      callback_executor_.spin_until_future_complete(cancel_future, server_timeout_) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node_->get_logger(), "[%s] failed to cancel active goal", name().c_str());
      return false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(node_->get_logger(), "[%s] cancel goal exception: %s", name().c_str(), ex.what());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] active navigation goal canceled", name().c_str());
  resetGoalState();
  return true;
}

void Nav2PoseAction::resetGoalState()
{
  goal_handle_.reset();
  active_goal_ = geometry_msgs::msg::PoseStamped();
  result_future_ = {};
}

void Nav2PoseAction::publishZeroTwist()
{
  if (cmd_vel_publisher_) {
    cmd_vel_publisher_->publish(geometry_msgs::msg::Twist{});
  }
}

bool Nav2PoseAction::goalsEqual(
  const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs)
{
  return nearlyEqual(lhs.pose.position.x, rhs.pose.position.x) &&
         nearlyEqual(lhs.pose.position.y, rhs.pose.position.y) &&
         nearlyEqual(lhs.pose.position.z, rhs.pose.position.z) &&
         nearlyEqual(lhs.pose.orientation.x, rhs.pose.orientation.x) &&
         nearlyEqual(lhs.pose.orientation.y, rhs.pose.orientation.y) &&
         nearlyEqual(lhs.pose.orientation.z, rhs.pose.orientation.z) &&
         nearlyEqual(lhs.pose.orientation.w, rhs.pose.orientation.w);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::Nav2PoseAction>("Nav2Pose", params);
}
