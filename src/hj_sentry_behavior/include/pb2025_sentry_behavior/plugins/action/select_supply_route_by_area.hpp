#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_SUPPLY_ROUTE_BY_AREA_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_SUPPLY_ROUTE_BY_AREA_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

/// @brief 根据机器人当前 TF 位置判断所在区域，选择对应的补给路线 CSV 文件。
/// 若本次补给周期已锁定路线（supply_route_locked == true），则直接返回 SUCCESS
/// 而不重新选择，确保同一次补给全程走同一条路线。
///
/// 端口:
///   - base_frame      : 机器人底盘 TF 坐标系（默认 "base_link"）
///   - map_frame       : 地图坐标系（默认 "map"）
///   - our_half_route      : 我方半场对应的 CSV 路线文件
///   - u_inner_route       : U 字通道内部对应的 CSV 路线文件
///   - highland_route      : 中央高地对应的 CSV 路线文件
///   - our_half_polygon    : 我方半场四边形顶点，格式 "x1,y1;x2,y2;x3,y3;x4,y4"
///   - u_inner_polygon     : U 字通道内部四边形顶点
///   - highland_polygon    : 中央高地四边形顶点
///   - supply_route_locked : [输入/输出] bool，是否已锁定路线
///   - selected_supply_route : [输出] 本次选定的路线文件路径
class SelectSupplyRouteByAreaAction : public BT::SyncActionNode
{
public:
  SelectSupplyRouteByAreaAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  /// @brief 判断点 (px, py) 是否在四边形多边形内（射线法）
  bool isInsidePolygon(
    double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon) const;

  /// @brief 解析 "x1,y1;x2,y2;..." 格式的多边形字符串
  std::vector<geometry_msgs::msg::Point> parsePolygon(const std::string & polygon_str) const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__SELECT_SUPPLY_ROUTE_BY_AREA_HPP_
