#include "pb2025_sentry_behavior/plugins/action/find_reachable_goal_in_area.hpp"

#include <algorithm>
#include <cmath>

#include "nav2_util/node_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using nav2_util::declare_parameter_if_not_declared;

namespace pb2025_sentry_behavior
{

FindReachableGoalInAreaAction::FindReachableGoalInAreaAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  declare_parameter_if_not_declared(
    node_, name + ".robot_base_frame", rclcpp::ParameterValue("base_footprint"));
  declare_parameter_if_not_declared(
    node_, name + ".global_frame", rclcpp::ParameterValue("map"));
  declare_parameter_if_not_declared(
    node_, name + ".transform_tolerance", rclcpp::ParameterValue(0.5));
  declare_parameter_if_not_declared(
    node_, name + ".visualize", rclcpp::ParameterValue(true));

  node_->get_parameter(name + ".robot_base_frame", robot_base_frame_);
  node_->get_parameter(name + ".global_frame", global_frame_);
  node_->get_parameter(name + ".transform_tolerance", transform_tolerance_);
  node_->get_parameter(name + ".visualize", visualize_);

  if (visualize_) {
    viz_publisher_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "find_reachable_goal_viz", 10);
  }
}

BT::PortsList FindReachableGoalInAreaAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "center", "0;0;0", "Center of search area. Fill with format `x;y;yaw`"),
    BT::InputPort<double>("radius", 2.0, "Search radius in meters"),
    BT::InputPort<int>("cost_threshold", 50, "Max costmap cost for a feasible point"),
    BT::InputPort<nav_msgs::msg::OccupancyGrid>(
      "costmap_port", "{@nav_globalCostmap}", "GlobalCostmap port on blackboard"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "{reachable_goal}",
      "Output: nearest reachable goal pose within the area"),
  };
}

BT::NodeStatus FindReachableGoalInAreaAction::tick()
{
  // 1. Read inputs
  auto center = getInput<geometry_msgs::msg::PoseStamped>("center");
  if (!center) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] center is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto radius = getInput<double>("radius");
  if (!radius) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] radius is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto cost_threshold = getInput<int>("cost_threshold");
  if (!cost_threshold) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] cost_threshold is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto costmap = getInput<nav_msgs::msg::OccupancyGrid>("costmap_port");
  if (!costmap) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] costmap_port is not set", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 2. Generate candidate points within the circular area
  const double step = costmap->info.resolution;
  if (step <= 0.0) {
    RCLCPP_ERROR(
      node_->get_logger(), "[%s] costmap resolution is invalid (%.4f)", name().c_str(), step);
    return BT::NodeStatus::FAILURE;
  }
  auto candidates = generateGridCandidates(center->pose.position, radius.value(), step);

  if (candidates.empty()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] no candidate points generated", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 3. Filter by costmap cost
  auto feasible = filterFeasiblePoints(candidates, costmap.value(), cost_threshold.value());

  if (feasible.empty()) {
    RCLCPP_WARN(
      node_->get_logger(), "[%s] no reachable point found within %.1fm of (%.2f, %.2f)",
      name().c_str(), radius.value(), center->pose.position.x, center->pose.position.y);
    return BT::NodeStatus::FAILURE;
  }

  // 4. Get robot pose and select nearest feasible point
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
        robot_pose, *tf_buffer_, global_frame_, robot_base_frame_, transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] failed to get robot pose", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  auto best = selectNearestPoint(feasible, robot_pose.pose.position);

  // 5. Build and output goal
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = global_frame_;
  goal.header.stamp = node_->now();
  goal.pose.position = best;
  goal.pose.position.z = 0.0;
  // Face toward the original center
  const double dx = center->pose.position.x - best.x;
  const double dy = center->pose.position.y - best.y;
  tf2::Quaternion q;
  q.setRPY(0, 0, std::atan2(dy, dx));
  goal.pose.orientation = tf2::toMsg(q);

  setOutput("goal", goal);

  // 6. Publish visualization
  if (visualize_ && viz_publisher_) {
    publishVisualization(center->pose.position, radius.value(), best, global_frame_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[%s] found reachable goal (%.2f, %.2f) within %.1fm of center (%.2f, %.2f), "
    "%zu/%zu candidates feasible",
    name().c_str(), best.x, best.y, radius.value(), center->pose.position.x,
    center->pose.position.y, feasible.size(), candidates.size());

  return BT::NodeStatus::SUCCESS;
}

std::vector<geometry_msgs::msg::Point> FindReachableGoalInAreaAction::generateGridCandidates(
  const geometry_msgs::msg::Point & center, double radius, double step) const
{
  std::vector<geometry_msgs::msg::Point> candidates;
  const double radius_sq = radius * radius;

  for (double dx = -radius; dx <= radius; dx += step) {
    for (double dy = -radius; dy <= radius; dy += step) {
      if (dx * dx + dy * dy <= radius_sq) {
        geometry_msgs::msg::Point p;
        p.x = center.x + dx;
        p.y = center.y + dy;
        p.z = 0.0;
        candidates.push_back(p);
      }
    }
  }
  return candidates;
}

std::vector<geometry_msgs::msg::Point> FindReachableGoalInAreaAction::filterFeasiblePoints(
  const std::vector<geometry_msgs::msg::Point> & candidates,
  const nav_msgs::msg::OccupancyGrid & costmap, int cost_threshold) const
{
  std::vector<geometry_msgs::msg::Point> feasible;
  const auto & info = costmap.info;

  for (const auto & p : candidates) {
    const int cell_x =
      static_cast<int>(std::floor((p.x - info.origin.position.x) / info.resolution));
    const int cell_y =
      static_cast<int>(std::floor((p.y - info.origin.position.y) / info.resolution));

    if (
      cell_x < 0 || cell_x >= static_cast<int>(info.width) || cell_y < 0 ||
      cell_y >= static_cast<int>(info.height))
    {
      continue;
    }

    const int index = cell_y * info.width + cell_x;
    const int8_t cost = costmap.data[index];
    if (cost >= 0 && cost <= cost_threshold) {
      feasible.push_back(p);
    }
  }
  return feasible;
}

geometry_msgs::msg::Point FindReachableGoalInAreaAction::selectNearestPoint(
  const std::vector<geometry_msgs::msg::Point> & feasible_points,
  const geometry_msgs::msg::Point & robot_position) const
{
  auto compare = [&](const auto & a, const auto & b) {
    const double dx1 = a.x - robot_position.x;
    const double dy1 = a.y - robot_position.y;
    const double dx2 = b.x - robot_position.x;
    const double dy2 = b.y - robot_position.y;
    return (dx1 * dx1 + dy1 * dy1) < (dx2 * dx2 + dy2 * dy2);
  };
  return *std::min_element(feasible_points.begin(), feasible_points.end(), compare);
}

void FindReachableGoalInAreaAction::publishVisualization(
  const geometry_msgs::msg::Point & center, double radius,
  const geometry_msgs::msg::Point & selected_goal,
  const std::string & frame_id) const
{
  visualization_msgs::msg::MarkerArray msg;
  const auto now = node_->now();
  const auto lifetime = rclcpp::Duration::from_seconds(5.0);

  // --- Marker 0: Search area circle ---
  visualization_msgs::msg::Marker circle;
  circle.header.frame_id = frame_id;
  circle.header.stamp = now;
  circle.ns = "find_reachable_goal";
  circle.id = 0;
  circle.type = visualization_msgs::msg::Marker::LINE_STRIP;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.pose.position.z = 0.05;
  circle.scale.x = 0.05;
  circle.color.b = 1.0;
  circle.color.a = 0.6;
  circle.lifetime = lifetime;

  constexpr int circle_segments = 36;
  for (int i = 0; i <= circle_segments; ++i) {
    const double angle = i * 2.0 * M_PI / circle_segments;
    geometry_msgs::msg::Point p;
    p.x = center.x + radius * std::cos(angle);
    p.y = center.y + radius * std::sin(angle);
    p.z = 0.0;
    circle.points.push_back(p);
  }
  msg.markers.push_back(circle);

  // --- Marker 1: Center point (blue sphere) ---
  visualization_msgs::msg::Marker center_marker;
  center_marker.header.frame_id = frame_id;
  center_marker.header.stamp = now;
  center_marker.ns = "find_reachable_goal";
  center_marker.id = 1;
  center_marker.type = visualization_msgs::msg::Marker::SPHERE;
  center_marker.action = visualization_msgs::msg::Marker::ADD;
  center_marker.pose.position = center;
  center_marker.pose.position.z = 0.1;
  center_marker.scale.x = center_marker.scale.y = center_marker.scale.z = 0.2;
  center_marker.color.b = 1.0;
  center_marker.color.a = 0.8;
  center_marker.lifetime = lifetime;
  msg.markers.push_back(center_marker);

  // --- Marker 2: Selected goal (green sphere) ---
  visualization_msgs::msg::Marker goal_marker;
  goal_marker.header.frame_id = frame_id;
  goal_marker.header.stamp = now;
  goal_marker.ns = "find_reachable_goal";
  goal_marker.id = 2;
  goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
  goal_marker.action = visualization_msgs::msg::Marker::ADD;
  goal_marker.pose.position = selected_goal;
  goal_marker.pose.position.z = 0.1;
  goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.3;
  goal_marker.color.g = 1.0;
  goal_marker.color.a = 1.0;
  goal_marker.lifetime = lifetime;
  msg.markers.push_back(goal_marker);

  // --- Marker 3: Line from center to selected goal ---
  visualization_msgs::msg::Marker line;
  line.header.frame_id = frame_id;
  line.header.stamp = now;
  line.ns = "find_reachable_goal";
  line.id = 3;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.03;
  line.color.g = 0.8;
  line.color.b = 0.2;
  line.color.a = 0.8;
  line.lifetime = lifetime;

  geometry_msgs::msg::Point p_center;
  p_center = center;
  p_center.z = 0.1;
  line.points.push_back(p_center);

  geometry_msgs::msg::Point p_goal;
  p_goal = selected_goal;
  p_goal.z = 0.1;
  line.points.push_back(p_goal);
  msg.markers.push_back(line);

  viz_publisher_->publish(msg);
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::FindReachableGoalInAreaAction>(
    "FindReachableGoalInArea", params);
}
