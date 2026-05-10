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

#include "pb2025_sentry_behavior/plugins/action/nav2_pose_rough_road.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

#include "action_msgs/msg/goal_status.hpp"
#include "pb2025_sentry_behavior/custom_types.hpp"
#include "rclcpp/parameter_client.hpp"

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kGoalEpsilon = 1e-6;
constexpr double kSideUnknownThresholdDefault = 0.3;
constexpr std::chrono::milliseconds kManualPublishPeriod(2);

bool nearlyEqual(double lhs, double rhs)
{
  return std::abs(lhs - rhs) <= kGoalEpsilon;
}

const char * toString(Nav2PoseRoughRoadAction::Side side)
{
  switch (side) {
    case Nav2PoseRoughRoadAction::Side::HOME:
      return "HOME";
    case Nav2PoseRoughRoadAction::Side::HIGHLAND:
      return "HIGHLAND";
    case Nav2PoseRoughRoadAction::Side::UNKNOWN:
      return "UNKNOWN";
  }
  return "UNKNOWN";
}

}  // namespace

Nav2PoseRoughRoadAction::Nav2PoseRoughRoadAction(
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

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  manual_timer_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, true);
  callback_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  goal_pose_publisher_ =
    node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
  cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList Nav2PoseRoughRoadAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("action_name", "navigate_to_pose", "Action server name"),
    BT::InputPort<std::string>("goal_pose_topic", "/goal_pose", "Goal pose topic for yaw controller"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "0;0;0", "Expected goal pose that send to nav2. Fill with format `x;y;yaw`"),
    BT::InputPort<std::string>("rough_road_polygon", "", "Rough road polygon x,y;x,y;..."),
    BT::InputPort<std::string>("rough_direct_drive_polygon", "", "Direct-drive rough road polygon x,y;x,y;..."),
    BT::InputPort<std::string>("map_frame", "map", "Map frame"),
    BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
    BT::InputPort<std::string>("cmd_vel_topic", "cmd_vel", "Twist topic for rough road override"),
    BT::InputPort<double>("rough_cross_timeout_sec", 6.0, "Timeout for manual rough road crossing"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "rough_home_side_anchor", "0;0;0", "Reference anchor on the home side of rough road"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "rough_highland_side_anchor", "0;0;0", "Reference anchor on the highland side of rough road"),
    BT::InputPort<double>("home_to_highland_vx", 0.8, "Linear X velocity from home to highland"),
    BT::InputPort<double>("home_to_highland_vy", 0.0, "Linear Y velocity from home to highland"),
    BT::InputPort<double>("home_to_highland_vyaw", 0.0, "Angular velocity from home to highland"),
    BT::InputPort<double>("highland_to_home_vx", 0.8, "Linear X velocity from highland to home"),
    BT::InputPort<double>("highland_to_home_vy", 0.0, "Linear Y velocity from highland to home"),
    BT::InputPort<double>("highland_to_home_vyaw", 0.0, "Angular velocity from highland to home"),
    BT::InputPort<bool>("rough_direct_drive_enabled", true, "Enable rough road direct drive intercept"),
  };
}

BT::NodeStatus Nav2PoseRoughRoadAction::onStart()
{
  goal_sent_ = false;
  last_send_attempt_sec_ = 0.0;

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto action_name = getInput<std::string>("action_name");
  if (!action_name) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] action_name is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (!configurePublishers()) {
    return BT::NodeStatus::FAILURE;
  }

  if (!createClient(action_name.value())) {
    return BT::NodeStatus::RUNNING;
  }

  if (!configureForGoal(goal.value())) {
    return BT::NodeStatus::FAILURE;
  }

  if (!sendGoal(target_goal_)) {
    return BT::NodeStatus::RUNNING;
  }

  goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Nav2PoseRoughRoadAction::onRunning()
{
  callback_executor_.spin_some();

  if (!goal_sent_) {
    // Retry initialization: action server wasn't ready in onStart
    if (!createClient(action_name_)) {
      return BT::NodeStatus::RUNNING;
    }
    // Only retry sendGoal every 2s to avoid flooding Nav2 with duplicate goals
    const double now = node_->now().seconds();
    if (now - last_send_attempt_sec_ >= 2.0) {
      last_send_attempt_sec_ = now;
      auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
      if (!goal) {
        return BT::NodeStatus::FAILURE;
      }
      if (!configureForGoal(goal.value())) {
        return BT::NodeStatus::FAILURE;
      }
      if (sendGoal(target_goal_)) {
        goal_sent_ = true;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  auto goal = getInput<geometry_msgs::msg::PoseStamped>("goal");
  if (!goal) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal is not set while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  switch (mode_) {
    case ExecutionMode::PLAIN_NAV:
    case ExecutionMode::APPROACH_NAV:
    case ExecutionMode::REJOIN_NAV:
      return handleNavPhase(goal.value());

    case ExecutionMode::MANUAL_CROSS:
      return handleManualCross(goal.value());
  }

  return BT::NodeStatus::FAILURE;
}

void Nav2PoseRoughRoadAction::onHalted()
{
  callback_executor_.spin_some();
  stopManualPublisher();
  publishZeroTwist();
  restoreControllerSpeed();
  cancelActiveGoal();
  resetGoalState();
  rough_intercept_enabled_ = false;
  entered_polygon_once_ = false;
  goal_sent_ = false;
  last_send_attempt_sec_ = 0.0;
  mode_ = ExecutionMode::PLAIN_NAV;
  start_side_ = Side::UNKNOWN;
  goal_side_ = Side::UNKNOWN;
}

bool Nav2PoseRoughRoadAction::configurePublishers()
{
  auto goal_pose_topic = getInput<std::string>("goal_pose_topic");
  if (!goal_pose_topic || goal_pose_topic->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal_pose_topic is not set", name().c_str());
    return false;
  }
  if (goal_pose_topic_ != goal_pose_topic.value()) {
    goal_pose_topic_ = goal_pose_topic.value();
    goal_pose_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic_, 10);
  }

  auto cmd_vel_topic = getInput<std::string>("cmd_vel_topic");
  if (!cmd_vel_topic || cmd_vel_topic->empty()) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] cmd_vel_topic is not set", name().c_str());
    return false;
  }
  if (cmd_vel_topic_ != cmd_vel_topic.value()) {
    cmd_vel_topic_ = cmd_vel_topic.value();
    cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
  }

  getInput("map_frame", map_frame_);
  getInput("base_frame", base_frame_);
  getInput("rough_cross_timeout_sec", rough_cross_timeout_sec_);
  if (rough_cross_timeout_sec_ <= 0.0) {
    rough_cross_timeout_sec_ = 6.0;
  }

  auto home_anchor = getInput<geometry_msgs::msg::PoseStamped>("rough_home_side_anchor");
  auto highland_anchor = getInput<geometry_msgs::msg::PoseStamped>("rough_highland_side_anchor");
  if (!home_anchor || !highland_anchor) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] rough road anchors are not set", name().c_str());
    return false;
  }
  rough_home_anchor_ = home_anchor.value();
  rough_highland_anchor_ = highland_anchor.value();
  side_unknown_threshold_m_ = kSideUnknownThresholdDefault;

  return true;
}

bool Nav2PoseRoughRoadAction::configureForGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  target_goal_ = goal;
  entered_polygon_once_ = false;
  rough_intercept_enabled_ = false;
  manual_start_time_ = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  start_side_ = Side::UNKNOWN;
  goal_side_ = Side::UNKNOWN;
  rough_polygon_.clear();

  if (!loadPolygon(rough_polygon_) || rough_polygon_.empty()) {
    mode_ = ExecutionMode::PLAIN_NAV;
    return true;
  }

  getInput("rough_direct_drive_enabled", rough_direct_drive_enabled_);
  if (!rough_direct_drive_enabled_) {
    mode_ = ExecutionMode::PLAIN_NAV;
    rough_intercept_enabled_ = false;
    return true;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!lookupRobotPosition(robot_x, robot_y)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] failed to classify robot side, fallback to plain navigation", name().c_str());
    mode_ = ExecutionMode::PLAIN_NAV;
    return true;
  }

  if (isInsidePolygon(robot_x, robot_y, rough_polygon_)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] robot starts inside rough road, fallback to plain navigation", name().c_str());
    mode_ = ExecutionMode::PLAIN_NAV;
    return true;
  }

  start_side_ = classifyPoint(robot_x, robot_y);
  goal_side_ = classifyGoalSide(goal);
  if (start_side_ == Side::UNKNOWN || goal_side_ == Side::UNKNOWN) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] side classification failed (start=%s goal=%s), fallback to plain navigation",
      name().c_str(), toString(start_side_), toString(goal_side_));
    mode_ = ExecutionMode::PLAIN_NAV;
    return true;
  }

  if (start_side_ == goal_side_) {
    mode_ = ExecutionMode::PLAIN_NAV;
    return true;
  }

  rough_intercept_enabled_ = true;
  mode_ = ExecutionMode::APPROACH_NAV;
  VelocityCommand command;
  if (start_side_ == Side::HOME && goal_side_ == Side::HIGHLAND) {
    getInput("home_to_highland_vx", command.vx);
    getInput("home_to_highland_vy", command.vy);
    getInput("home_to_highland_vyaw", command.vyaw);
  } else {
    getInput("highland_to_home_vx", command.vx);
    getInput("highland_to_home_vy", command.vy);
    getInput("highland_to_home_vyaw", command.vyaw);
  }
  {
    std::lock_guard<std::mutex> lock(manual_command_mutex_);
    manual_command_ = command;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] rough road intercept armed: start=%s goal=%s cmd=[%.3f, %.3f, %.3f]",
    name().c_str(), toString(start_side_), toString(goal_side_),
    command.vx, command.vy, command.vyaw);
  return true;
}

BT::NodeStatus Nav2PoseRoughRoadAction::handleNavPhase(
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!goalsEqual(goal, target_goal_)) {
    if (!configurePublishers() || !configureForGoal(goal) || !sendGoal(target_goal_)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (!goal_handle_) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] goal handle is empty while running", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  if (mode_ == ExecutionMode::APPROACH_NAV && rough_intercept_enabled_) {
    double robot_x = 0.0;
    double robot_y = 0.0;
    if (lookupRobotPosition(robot_x, robot_y) && isInsidePolygon(robot_x, robot_y, rough_polygon_)) {
      entered_polygon_once_ = true;
      manual_start_time_ = node_->now();
      suppressControllerSpeed();
      cancelActiveGoal();
      mode_ = ExecutionMode::MANUAL_CROSS;
      startManualPublisher();
      RCLCPP_INFO(node_->get_logger(), "[%s] entered rough road, switch to manual cross", name().c_str());
      return BT::NodeStatus::RUNNING;
    }
  }

  if (
    result_future_.valid() &&
    result_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
  {
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
        if (mode_ == ExecutionMode::MANUAL_CROSS) {
          return BT::NodeStatus::RUNNING;
        }
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
      if (mode_ == ExecutionMode::MANUAL_CROSS) {
        return BT::NodeStatus::RUNNING;
      }
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

BT::NodeStatus Nav2PoseRoughRoadAction::handleManualCross(
  const geometry_msgs::msg::PoseStamped & goal)
{
  if (!goalsEqual(goal, target_goal_)) {
    stopManualPublisher();
    publishZeroTwist();
    restoreControllerSpeed();
    resetGoalState();
    if (!configurePublishers() || !configureForGoal(goal) || !sendGoal(target_goal_)) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }

  if (
    rough_cross_timeout_sec_ > 0.0 &&
    (node_->now() - manual_start_time_).seconds() > rough_cross_timeout_sec_)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] manual rough road crossing timed out", name().c_str());
    stopManualPublisher();
    publishZeroTwist();
    restoreControllerSpeed();
    resetGoalState();
    return BT::NodeStatus::FAILURE;
  }

  double robot_x = 0.0;
  double robot_y = 0.0;
  if (!lookupRobotPosition(robot_x, robot_y)) {
    return BT::NodeStatus::RUNNING;
  }

  if (entered_polygon_once_ && !isInsidePolygon(robot_x, robot_y, rough_polygon_)) {
    stopManualPublisher();
    publishZeroTwist();
    restoreControllerSpeed();
    mode_ = ExecutionMode::REJOIN_NAV;
    resetGoalState();
    if (!sendGoal(target_goal_)) {
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_INFO(node_->get_logger(), "[%s] left rough road, rejoin navigation", name().c_str());
  }
  return BT::NodeStatus::RUNNING;
}

bool Nav2PoseRoughRoadAction::createClient(const std::string & action_name)
{
  if (action_client_ && action_name_ == action_name) {
    return true;
  }

  action_name_ = action_name;
  action_client_ =
    rclcpp_action::create_client<NavigateToPose>(node_, action_name_, callback_group_);

  if (!action_client_->wait_for_action_server(wait_for_server_timeout_)) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] action server '%s' is not reachable", name().c_str(),
      action_name_.c_str());
    return false;
  }

  return true;
}

bool Nav2PoseRoughRoadAction::sendGoal(const geometry_msgs::msg::PoseStamped & goal)
{
  NavigateToPose::Goal navigation_goal;
  navigation_goal.pose = goal;
  navigation_goal.pose.header.frame_id = "map";
  navigation_goal.pose.header.stamp = node_->now();
  publishGoalPose(navigation_goal.pose);

  auto future_goal_handle = action_client_->async_send_goal(navigation_goal);
  if (
    callback_executor_.spin_until_future_complete(future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
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

  RCLCPP_INFO(
    node_->get_logger(), "[%s] navigating to pose [%.3f, %.3f] mode=%d", name().c_str(),
    active_goal_.pose.position.x, active_goal_.pose.position.y, static_cast<int>(mode_));
  return true;
}

void Nav2PoseRoughRoadAction::publishGoalPose(const geometry_msgs::msg::PoseStamped & goal)
{
  if (goal_pose_publisher_) {
    goal_pose_publisher_->publish(goal);
  }
}

bool Nav2PoseRoughRoadAction::cancelActiveGoal()
{
  if (!goal_handle_) {
    return true;
  }

  const auto status = goal_handle_->get_status();
  if (
    status != action_msgs::msg::GoalStatus::STATUS_ACCEPTED &&
    status != action_msgs::msg::GoalStatus::STATUS_EXECUTING &&
    status != action_msgs::msg::GoalStatus::STATUS_CANCELING)
  {
    return true;
  }

  auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
  if (
    callback_executor_.spin_until_future_complete(cancel_future, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(node_->get_logger(), "[%s] failed to cancel active goal", name().c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] active navigation goal canceled", name().c_str());
  resetGoalState();
  return true;
}

void Nav2PoseRoughRoadAction::resetGoalState()
{
  goal_handle_.reset();
  active_goal_ = geometry_msgs::msg::PoseStamped();
  result_future_ = {};
}

bool Nav2PoseRoughRoadAction::loadPolygon(std::vector<geometry_msgs::msg::Point> & polygon)
{
  std::string polygon_str;
  getInput("rough_direct_drive_polygon", polygon_str);
  if (polygon_str.empty()) {
    polygon.clear();
    return true;
  }

  try {
    polygon = parsePolygon(polygon_str);
    return true;
  } catch (const std::exception & ex) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] failed to parse rough road polygon: %s",
      name().c_str(), ex.what());
    polygon.clear();
    return false;
  }
}

std::vector<geometry_msgs::msg::Point> Nav2PoseRoughRoadAction::parsePolygon(
  const std::string & polygon_str)
{
  std::vector<geometry_msgs::msg::Point> polygon;
  std::size_t start = 0;
  while (start < polygon_str.size()) {
    const auto end = polygon_str.find(';', start);
    const auto token = polygon_str.substr(start, end == std::string::npos ? std::string::npos : end - start);
    const auto comma = token.find(',');
    if (comma == std::string::npos) {
      throw std::runtime_error("invalid polygon token: " + token);
    }

    geometry_msgs::msg::Point point;
    point.x = std::stod(token.substr(0, comma));
    point.y = std::stod(token.substr(comma + 1));
    polygon.push_back(point);

    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }

  if (polygon.size() < 3) {
    throw std::runtime_error("polygon needs at least 3 points");
  }
  return polygon;
}

bool Nav2PoseRoughRoadAction::lookupRobotPosition(double & x, double & y)
{
  try {
    const auto transform = tf_buffer_->lookupTransform(
      map_frame_, base_frame_, tf2::TimePointZero, tf2::durationFromSec(0.05));
    x = transform.transform.translation.x;
    y = transform.transform.translation.y;
    return true;
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000,
      "[%s] failed to lookup robot pose: %s", name().c_str(), ex.what());
    return false;
  }
}

bool Nav2PoseRoughRoadAction::isInsidePolygon(
  double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon)
{
  bool inside = false;
  const int n = static_cast<int>(polygon.size());
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].x;
    const double yi = polygon[i].y;
    const double xj = polygon[j].x;
    const double yj = polygon[j].y;
    const bool intersect =
      ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

Nav2PoseRoughRoadAction::Side Nav2PoseRoughRoadAction::classifyPoint(double x, double y) const
{
  const double home_distance = std::hypot(
    x - rough_home_anchor_.pose.position.x, y - rough_home_anchor_.pose.position.y);
  const double highland_distance = std::hypot(
    x - rough_highland_anchor_.pose.position.x, y - rough_highland_anchor_.pose.position.y);

  if (std::abs(home_distance - highland_distance) < side_unknown_threshold_m_) {
    return Side::UNKNOWN;
  }
  return home_distance < highland_distance ? Side::HOME : Side::HIGHLAND;
}

std::optional<Nav2PoseRoughRoadAction::Side> Nav2PoseRoughRoadAction::classifyRobotSide()
{
  double x = 0.0;
  double y = 0.0;
  if (!lookupRobotPosition(x, y)) {
    return std::nullopt;
  }
  return classifyPoint(x, y);
}

Nav2PoseRoughRoadAction::Side Nav2PoseRoughRoadAction::classifyGoalSide(
  const geometry_msgs::msg::PoseStamped & goal) const
{
  return classifyPoint(goal.pose.position.x, goal.pose.position.y);
}

void Nav2PoseRoughRoadAction::startManualPublisher()
{
  if (manual_publish_timer_) {
    manual_publish_timer_->cancel();
  }

  manual_publish_timer_ = node_->create_wall_timer(
    kManualPublishPeriod,
    [this]() {
      VelocityCommand command;
      {
        std::lock_guard<std::mutex> lock(manual_command_mutex_);
        command = manual_command_;
      }
      publishTwist(command);
    },
    manual_timer_group_);
}

void Nav2PoseRoughRoadAction::stopManualPublisher()
{
  if (manual_publish_timer_) {
    manual_publish_timer_->cancel();
    manual_publish_timer_.reset();
  }
}

void Nav2PoseRoughRoadAction::publishZeroTwist()
{
  publishTwist(VelocityCommand{});
}

void Nav2PoseRoughRoadAction::publishTwist(const VelocityCommand & command)
{
  if (!cmd_vel_publisher_) {
    return;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x = command.vx;
  msg.linear.y = command.vy;
  msg.angular.z = command.vyaw;
  cmd_vel_publisher_->publish(msg);
}

bool Nav2PoseRoughRoadAction::goalsEqual(
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

void Nav2PoseRoughRoadAction::ensureSpeedParameterClients()
{
  std::lock_guard<std::mutex> lock(speed_param_mutex_);
  if (controller_params_client_ && smoother_params_client_) {
    return;
  }

  controller_params_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, controller_node_);
  smoother_params_client_ =
    std::make_shared<rclcpp::AsyncParametersClient>(node_, smoother_node_);
  controller_defaults_loaded_ = false;
  defaults_requested_ = false;
}

void Nav2PoseRoughRoadAction::suppressControllerSpeed()
{
  ensureSpeedParameterClients();

  // Load defaults if we haven't already
  if (!controller_defaults_loaded_) {
    std::lock_guard<std::mutex> lock(speed_param_mutex_);
    if (!defaults_requested_ && controller_params_client_ && smoother_params_client_)
    {
      if (controller_params_client_->service_is_ready() &&
          smoother_params_client_->service_is_ready())
      {
        defaults_requested_ = true;

        controller_params_client_->get_parameters(
          {"FollowPath.v_linear_min", "FollowPath.v_linear_max",
           "FollowPath.v_angular_min", "FollowPath.v_angular_max"},
          [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
            try {
              const auto params = future.get();
              if (params.size() == 4) {
                std::lock_guard<std::mutex> lock(speed_param_mutex_);
                default_v_linear_min_ = params[0].as_double();
                default_v_linear_max_ = params[1].as_double();
                default_v_angular_min_ = params[2].as_double();
                default_v_angular_max_ = params[3].as_double();
                controller_defaults_loaded_ = true;
              }
            } catch (const std::exception & ex) {
              std::lock_guard<std::mutex> lock(speed_param_mutex_);
              defaults_requested_ = false;
            }
          });

        smoother_params_client_->get_parameters(
          {"max_velocity", "min_velocity"},
          [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
            try {
              const auto params = future.get();
              if (params.size() == 2) {
                std::lock_guard<std::mutex> lock(speed_param_mutex_);
                default_smoother_max_ = params[0].as_double_array();
                default_smoother_min_ = params[1].as_double_array();
              }
            } catch (const std::exception & ex) {
              // non-fatal, smoother restore will be skipped if defaults missing
            }
          });
      }
    }
  }

  // Set controller speed limits to near-zero
  if (controller_params_client_ && controller_params_client_->service_is_ready()) {
    controller_params_client_->set_parameters(
      {
        rclcpp::Parameter("FollowPath.v_linear_min", 0.0),
        rclcpp::Parameter("FollowPath.v_linear_max", 0.0),
        rclcpp::Parameter("FollowPath.v_angular_min", 0.0),
        rclcpp::Parameter("FollowPath.v_angular_max", 0.0),
      },
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          for (const auto & result : future.get()) {
            if (!result.successful) {
              RCLCPP_WARN(node_->get_logger(),
                "[%s] failed to suppress controller speed: %s", name().c_str(),
                result.reason.c_str());
            }
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(node_->get_logger(),
            "[%s] suppress controller speed exception: %s", name().c_str(), ex.what());
        }
      });
  }

  // Set smoother to near-zero as well
  if (smoother_params_client_ && smoother_params_client_->service_is_ready()) {
    const std::vector<double> zero_velocity = {0.1, 0.1, 0.1};
    const std::vector<double> neg_zero_velocity = {-0.1, -0.1, -0.1};
    smoother_params_client_->set_parameters(
      {
        rclcpp::Parameter("max_velocity", zero_velocity),
        rclcpp::Parameter("min_velocity", neg_zero_velocity),
      },
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          for (const auto & result : future.get()) {
            if (!result.successful) {
              RCLCPP_WARN(node_->get_logger(),
                "[%s] failed to suppress smoother speed: %s", name().c_str(),
                result.reason.c_str());
            }
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(node_->get_logger(),
            "[%s] suppress smoother speed exception: %s", name().c_str(), ex.what());
        }
      });
  }
}

void Nav2PoseRoughRoadAction::restoreControllerSpeed()
{
  std::lock_guard<std::mutex> lock(speed_param_mutex_);
  if (!controller_defaults_loaded_) {
    return;
  }

  if (controller_params_client_ && controller_params_client_->service_is_ready()) {
    controller_params_client_->set_parameters(
      {
        rclcpp::Parameter("FollowPath.v_linear_min", default_v_linear_min_),
        rclcpp::Parameter("FollowPath.v_linear_max", default_v_linear_max_),
        rclcpp::Parameter("FollowPath.v_angular_min", default_v_angular_min_),
        rclcpp::Parameter("FollowPath.v_angular_max", default_v_angular_max_),
      },
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          for (const auto & result : future.get()) {
            if (!result.successful) {
              RCLCPP_WARN(node_->get_logger(),
                "[%s] failed to restore controller speed: %s", name().c_str(),
                result.reason.c_str());
            }
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(node_->get_logger(),
            "[%s] restore controller speed exception: %s", name().c_str(), ex.what());
        }
      });
  }

  if (smoother_params_client_ && smoother_params_client_->service_is_ready() &&
      !default_smoother_max_.empty() && !default_smoother_min_.empty())
  {
    smoother_params_client_->set_parameters(
      {
        rclcpp::Parameter("max_velocity", default_smoother_max_),
        rclcpp::Parameter("min_velocity", default_smoother_min_),
      },
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        try {
          for (const auto & result : future.get()) {
            if (!result.successful) {
              RCLCPP_WARN(node_->get_logger(),
                "[%s] failed to restore smoother speed: %s", name().c_str(),
                result.reason.c_str());
            }
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(node_->get_logger(),
            "[%s] restore smoother speed exception: %s", name().c_str(), ex.what());
        }
      });
  }
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::Nav2PoseRoughRoadAction>(
    "Nav2PoseRoughRoad", params);
}
