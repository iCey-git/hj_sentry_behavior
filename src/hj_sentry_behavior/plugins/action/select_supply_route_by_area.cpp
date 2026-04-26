#include "pb2025_sentry_behavior/plugins/action/select_supply_route_by_area.hpp"

#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"

namespace pb2025_sentry_behavior
{

SelectSupplyRouteByAreaAction::SelectSupplyRouteByAreaAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf)
{
  node_ = params.nh.lock();
  if (!node_) {
    throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
  }
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::PortsList SelectSupplyRouteByAreaAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("base_frame", "base_link", "机器人底盘 TF 坐标系"),
    BT::InputPort<std::string>("map_frame", "map", "地图坐标系"),
    BT::InputPort<std::string>("our_half_route", "", "我方半场对应的 CSV 路线文件"),
    BT::InputPort<std::string>("u_inner_route", "", "U 字通道内部对应的 CSV 路线文件"),
    BT::InputPort<std::string>("highland_route", "", "中央高地对应的 CSV 路线文件"),
    BT::InputPort<std::string>(
      "our_half_polygon", "",
      "我方半场四边形顶点，格式 \"x1,y1;x2,y2;x3,y3;x4,y4\""),
    BT::InputPort<std::string>(
      "u_inner_polygon", "",
      "U 字通道内部四边形顶点，格式 \"x1,y1;x2,y2;x3,y3;x4,y4\""),
    BT::InputPort<std::string>(
      "highland_polygon", "",
      "中央高地四边形顶点，格式 \"x1,y1;x2,y2;x3,y3;x4,y4\""),
    BT::BidirectionalPort<bool>(
      "supply_route_locked", false, "是否已锁定本次补给路线"),
    BT::BidirectionalPort<std::string>(
      "selected_supply_route", "", "本次选定的路线文件路径"),
  };
}

BT::NodeStatus SelectSupplyRouteByAreaAction::tick()
{
  // 若已锁定，直接返回（不重新选路线）
  auto locked = getInput<bool>("supply_route_locked");
  if (locked && locked.value()) {
    auto selected_route = getInput<std::string>("selected_supply_route");
    if (selected_route && !selected_route->empty()) {
      setOutput("selected_supply_route", selected_route.value());
    }
    setOutput("supply_route_locked", true);
    RCLCPP_DEBUG(
      node_->get_logger(),
      "[%s] supply route already locked, skip re-selection", name().c_str());
    return BT::NodeStatus::SUCCESS;
  }

  auto base_frame = getInput<std::string>("base_frame");
  auto map_frame = getInput<std::string>("map_frame");
  auto our_half_route = getInput<std::string>("our_half_route");
  auto u_inner_route = getInput<std::string>("u_inner_route");
  auto highland_route = getInput<std::string>("highland_route");
  auto our_half_polygon_str = getInput<std::string>("our_half_polygon");
  auto u_inner_polygon_str = getInput<std::string>("u_inner_polygon");
  auto highland_polygon_str = getInput<std::string>("highland_polygon");

  if (
    !base_frame || !map_frame || !our_half_route || !u_inner_route || !highland_route ||
    !our_half_polygon_str || !u_inner_polygon_str || !highland_polygon_str)
  {
    RCLCPP_ERROR(node_->get_logger(), "[%s] missing required ports", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 获取机器人当前位置（TF 查询）
  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_->lookupTransform(
      map_frame.value(), base_frame.value(),
      tf2::TimePointZero, tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[%s] TF lookup failed: %s — defaulting to highland fallback route",
      name().c_str(), ex.what());
    // TF 失败时默认按中央高地处理，走更保守的撤退路线。
    setOutput("selected_supply_route", highland_route.value());
    setOutput("supply_route_locked", true);
    return BT::NodeStatus::SUCCESS;
  }

  const double robot_x = transform.transform.translation.x;
  const double robot_y = transform.transform.translation.y;

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[%s] robot position: x=%.2f y=%.2f", name().c_str(), robot_x, robot_y);

  std::string selected_route;
  try {
    const auto highland_polygon = parsePolygon(highland_polygon_str.value());
    const auto u_inner_polygon = parsePolygon(u_inner_polygon_str.value());
    const auto our_half_polygon = parsePolygon(our_half_polygon_str.value());

    // 按危险程度和离家距离从高到低判断，避免边界重叠时选到过短路线。
    if (isInsidePolygon(robot_x, robot_y, highland_polygon)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] robot in highland, selecting route: %s",
        name().c_str(), highland_route->c_str());
      selected_route = highland_route.value();
    } else if (isInsidePolygon(robot_x, robot_y, u_inner_polygon)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] robot in U inner area, selecting route: %s",
        name().c_str(), u_inner_route->c_str());
      selected_route = u_inner_route.value();
    } else if (isInsidePolygon(robot_x, robot_y, our_half_polygon)) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] robot in our half, selecting route: %s",
        name().c_str(), our_half_route->c_str());
      selected_route = our_half_route.value();
    } else {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] robot outside configured areas, selecting highland fallback route: %s",
        name().c_str(), highland_route->c_str());
      selected_route = highland_route.value();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "[%s] polygon parse error: %s", name().c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  setOutput("selected_supply_route", selected_route);
  setOutput("supply_route_locked", true);
  return BT::NodeStatus::SUCCESS;
}

bool SelectSupplyRouteByAreaAction::isInsidePolygon(
  double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon) const
{
  // 射线法（Ray Casting）
  const int n = static_cast<int>(polygon.size());
  bool inside = false;
  for (int i = 0, j = n - 1; i < n; j = i++) {
    const double xi = polygon[i].x, yi = polygon[i].y;
    const double xj = polygon[j].x, yj = polygon[j].y;
    const bool intersect =
      ((yi > py) != (yj > py)) &&
      (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
    if (intersect) {
      inside = !inside;
    }
  }
  return inside;
}

std::vector<geometry_msgs::msg::Point> SelectSupplyRouteByAreaAction::parsePolygon(
  const std::string & polygon_str) const
{
  std::vector<geometry_msgs::msg::Point> polygon;
  std::istringstream ss(polygon_str);
  std::string token;
  while (std::getline(ss, token, ';')) {
    std::istringstream pair_ss(token);
    std::string x_str, y_str;
    if (!std::getline(pair_ss, x_str, ',') || !std::getline(pair_ss, y_str, ',')) {
      throw std::runtime_error("Invalid polygon format: " + token);
    }
    geometry_msgs::msg::Point pt;
    pt.x = std::stod(x_str);
    pt.y = std::stod(y_str);
    pt.z = 0.0;
    polygon.push_back(pt);
  }
  if (polygon.size() < 3) {
    throw std::runtime_error("Polygon must have at least 3 vertices");
  }
  return polygon;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::SelectSupplyRouteByAreaAction>(
    "SelectSupplyRouteByArea", params);
}
