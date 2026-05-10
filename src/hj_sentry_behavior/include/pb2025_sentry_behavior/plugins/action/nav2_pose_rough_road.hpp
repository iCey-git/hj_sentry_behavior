#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_ROUGH_ROAD_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_ROUGH_ROAD_HPP_

#include <chrono>
#include <future>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

class Nav2PoseRoughRoadAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  enum class ExecutionMode
  {
    PLAIN_NAV,
    APPROACH_NAV,
    MANUAL_CROSS,
    REJOIN_NAV,
  };

  enum class Side
  {
    UNKNOWN,
    HOME,
    HIGHLAND,
  };

  struct VelocityCommand
  {
    double vx = 0.0;
    double vy = 0.0;
    double vyaw = 0.0;
  };

  Nav2PoseRoughRoadAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  bool configurePublishers();
  bool configureForGoal(const geometry_msgs::msg::PoseStamped & goal);
  BT::NodeStatus handleNavPhase(const geometry_msgs::msg::PoseStamped & goal);
  BT::NodeStatus handleManualCross(const geometry_msgs::msg::PoseStamped & goal);
  bool createClient(const std::string & action_name);
  bool sendGoal(const geometry_msgs::msg::PoseStamped & goal);
  void publishGoalPose(const geometry_msgs::msg::PoseStamped & goal);
  bool cancelActiveGoal();
  void resetGoalState();

  bool loadPolygon(std::vector<geometry_msgs::msg::Point> & polygon);
  static std::vector<geometry_msgs::msg::Point> parsePolygon(const std::string & polygon_str);
  bool lookupRobotPosition(double & x, double & y);
  static bool isInsidePolygon(
    double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon);
  Side classifyPoint(double x, double y) const;
  std::optional<Side> classifyRobotSide();
  Side classifyGoalSide(const geometry_msgs::msg::PoseStamped & goal) const;

  void startManualPublisher();
  void stopManualPublisher();
  void publishZeroTwist();
  void publishTwist(const VelocityCommand & command);

  static bool goalsEqual(
    const geometry_msgs::msg::PoseStamped & lhs, const geometry_msgs::msg::PoseStamped & rhs);

  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::string goal_pose_topic_;
  std::string cmd_vel_topic_;
  std::string map_frame_ = "map";
  std::string base_frame_ = "base_footprint";
  std::chrono::milliseconds server_timeout_;
  std::chrono::milliseconds wait_for_server_timeout_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::CallbackGroup::SharedPtr manual_timer_group_;
  rclcpp::executors::SingleThreadedExecutor callback_executor_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr manual_publish_timer_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  geometry_msgs::msg::PoseStamped target_goal_;
  geometry_msgs::msg::PoseStamped active_goal_;
  std::shared_future<GoalHandleNavigateToPose::WrappedResult> result_future_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  geometry_msgs::msg::PoseStamped rough_home_anchor_;
  geometry_msgs::msg::PoseStamped rough_highland_anchor_;
  double rough_cross_timeout_sec_ = 6.0;
  double side_unknown_threshold_m_ = 0.3;
  bool rough_intercept_enabled_ = false;
  bool rough_direct_drive_enabled_ = true;
  bool entered_polygon_once_ = false;
  bool goal_sent_ = false;
  double last_send_attempt_sec_ = 0.0;
  ExecutionMode mode_ = ExecutionMode::PLAIN_NAV;
  Side start_side_ = Side::UNKNOWN;
  Side goal_side_ = Side::UNKNOWN;
  VelocityCommand manual_command_;
  std::mutex manual_command_mutex_;
  rclcpp::Time manual_start_time_{0, 0, RCL_ROS_TIME};
  std::vector<geometry_msgs::msg::Point> rough_polygon_;

  // Controller speed suppression during manual cross
  void ensureSpeedParameterClients();
  void suppressControllerSpeed();
  void restoreControllerSpeed();
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_params_client_;
  std::string controller_node_{"/controller_server"};
  std::string smoother_node_{"/velocity_smoother"};
  bool controller_defaults_loaded_ = false;
  bool defaults_requested_ = false;
  double default_v_linear_min_ = 0.0;
  double default_v_linear_max_ = 0.0;
  double default_v_angular_min_ = 0.0;
  double default_v_angular_max_ = 0.0;
  std::vector<double> default_smoother_max_;
  std::vector<double> default_smoother_min_;
  std::mutex speed_param_mutex_;
};

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__ACTION__NAV2_POSE_ROUGH_ROAD_HPP_
