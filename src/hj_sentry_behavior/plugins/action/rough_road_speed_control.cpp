#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/plugins.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_msgs/msg/mode_control.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace pb2025_sentry_behavior
{

namespace
{

constexpr double kRoughVLinearMin = -1.2;
constexpr double kRoughVLinearMax = 1.1;
constexpr double kRoughVAngularMin = -1.8;
constexpr double kRoughVAngularMax = 1.0;

const std::vector<double> kRoughSmootherMaxVelocity = {1.3, 1.3, 1.8};
const std::vector<double> kRoughSmootherMinVelocity = {-1.3, -1.3, -1.8};

}  // namespace

class RoughRoadSpeedControl : public BT::SyncActionNode
{
public:
  RoughRoadSpeedControl(
    const std::string & name, const BT::NodeConfig & config, const BT::RosNodeParams & params)
  : BT::SyncActionNode(name, config)
  {
    node_ = params.nh.lock();
    if (!node_) {
      throw std::logic_error("RosNodeParams doesn't contain a valid ROS node");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("rough_road_polygon", "", "Rough road polygon x,y;x,y;..."),
      BT::InputPort<std::string>("map_frame", "map", "Map frame"),
      BT::InputPort<std::string>("base_frame", "base_footprint", "Robot base frame"),
      BT::InputPort<std::string>(
        "controller_node", "/controller_server", "Nav2 controller server node"),
      BT::InputPort<std::string>(
        "smoother_node", "/velocity_smoother", "Nav2 velocity smoother node"),
      BT::InputPort<bool>("control_speed", true, "Set Nav2 speed parameters"),
      BT::InputPort<bool>("publish_mode_control", true, "Publish rough road mode override"),
      BT::InputPort<std::string>("mode_topic", "/mode_ctrl", "Mode control topic"),
      BT::InputPort<int>("rough_chassis_gyro", 0, "Chassis gyro value on rough road"),
      BT::InputPort<int>("normal_chassis_gyro", 1, "Chassis gyro value after rough road"),
      BT::InputPort<int>("patrol_mode", 2, "Patrol mode on rough road"),
      BT::InputPort<int>("power_mode", 12, "Power mode on rough road"),
      BT::OutputPort<bool>("rough_road_active", "True when robot is inside rough road polygon"),
    };
  }

  BT::NodeStatus tick() override
  {
    const auto in_rough_road = isInRoughRoad();
    if (!in_rough_road.has_value()) {
      setOutput("rough_road_active", rough_road_active_);
      return BT::NodeStatus::SUCCESS;
    }

    const bool was_rough_road_active = rough_road_active_;
    rough_road_active_ = in_rough_road.value();
    setOutput("rough_road_active", rough_road_active_);

    if (rough_road_active_) {
      int rough_chassis_gyro = 0;
      getInput("rough_chassis_gyro", rough_chassis_gyro);
      publishModeControlIfEnabled(rough_chassis_gyro);
    } else if (was_rough_road_active) {
      int normal_chassis_gyro = 1;
      getInput("normal_chassis_gyro", normal_chassis_gyro);
      publishModeControlIfEnabled(normal_chassis_gyro);
    }

    bool control_speed = true;
    getInput("control_speed", control_speed);
    if (!control_speed) {
      return BT::NodeStatus::SUCCESS;
    }

    ensureParameterClients(controllerNode(), smootherNode());
    if (!defaultsLoaded()) {
      requestDefaultsIfNeeded();
      return BT::NodeStatus::SUCCESS;
    }

    if (!speed_mode_applied_.has_value() || speed_mode_applied_.value() != rough_road_active_) {
      applySpeedMode(rough_road_active_);
      speed_mode_applied_ = rough_road_active_;
    }

    return BT::NodeStatus::SUCCESS;
  }

private:
  struct ControllerDefaults
  {
    double v_linear_min = 0.0;
    double v_linear_max = 0.0;
    double v_angular_min = 0.0;
    double v_angular_max = 0.0;
  };

  struct SmootherDefaults
  {
    std::vector<double> max_velocity;
    std::vector<double> min_velocity;
  };

  std::optional<bool> isInRoughRoad()
  {
    std::string polygon_str;
    std::string map_frame;
    std::string base_frame;
    getInput("rough_road_polygon", polygon_str);
    getInput("map_frame", map_frame);
    getInput("base_frame", base_frame);

    if (polygon_str.empty()) {
      return false;
    }

    try {
      const auto polygon = parsePolygon(polygon_str);
      const auto transform = tf_buffer_->lookupTransform(
        map_frame, base_frame, tf2::TimePointZero, tf2::durationFromSec(0.05));
      return isInsidePolygon(
        transform.transform.translation.x, transform.transform.translation.y, polygon);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] failed to lookup robot pose: %s", name().c_str(), ex.what());
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] rough_road_polygon parse failed: %s", name().c_str(), ex.what());
    }
    return std::nullopt;
  }

  bool applySpeedMode(bool rough_road)
  {
    if (rough_road) {
      RCLCPP_INFO(
        node_->get_logger(),
        "[%s] rough road active: set controller speed to linear[%.2f, %.2f], angular[%.2f, %.2f]",
        name().c_str(), kRoughVLinearMin, kRoughVLinearMax, kRoughVAngularMin,
        kRoughVAngularMax);
      return setControllerSpeed(
               kRoughVLinearMin, kRoughVLinearMax, kRoughVAngularMin, kRoughVAngularMax) &&
             setSmootherSpeed(kRoughSmootherMaxVelocity, kRoughSmootherMinVelocity);
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "[%s] rough road inactive: restore controller speed to linear[%.2f, %.2f], angular[%.2f, %.2f]",
      name().c_str(), controller_defaults_.v_linear_min, controller_defaults_.v_linear_max,
      controller_defaults_.v_angular_min, controller_defaults_.v_angular_max);
    return setControllerSpeed(
             controller_defaults_.v_linear_min, controller_defaults_.v_linear_max,
             controller_defaults_.v_angular_min, controller_defaults_.v_angular_max) &&
           setSmootherSpeed(smoother_defaults_.max_velocity, smoother_defaults_.min_velocity);
  }

  void publishModeControlIfEnabled(int chassis_gyro)
  {
    bool publish_mode_control = true;
    getInput("publish_mode_control", publish_mode_control);
    if (!publish_mode_control) {
      return;
    }

    std::string topic = "/mode_ctrl";
    int patrol_mode = 2;
    int power_mode = 12;
    getInput("mode_topic", topic);
    getInput("patrol_mode", patrol_mode);
    getInput("power_mode", power_mode);

    if (!mode_pub_ || mode_topic_ != topic) {
      mode_topic_ = topic;
      mode_pub_ = node_->create_publisher<robot_msgs::msg::ModeControl>(mode_topic_, 10);
    }

    robot_msgs::msg::ModeControl msg;
    msg.chassis_gyro = static_cast<uint16_t>(std::clamp(chassis_gyro, 0, 65535));
    msg.patrol_mode = static_cast<uint8_t>(std::clamp(patrol_mode, 0, 255));
    msg.power_mode = static_cast<uint8_t>(std::clamp(power_mode, 0, 255));
    mode_pub_->publish(msg);
  }

  void ensureParameterClients(const std::string & controller_node, const std::string & smoother_node)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (controller_params_client_ && smoother_params_client_ &&
        controller_client_node_ == controller_node && smoother_client_node_ == smoother_node)
    {
      return;
    }

    controller_client_node_ = controller_node;
    smoother_client_node_ = smoother_node;
    controller_params_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, controller_client_node_);
    smoother_params_client_ =
      std::make_shared<rclcpp::AsyncParametersClient>(node_, smoother_client_node_);
    controller_defaults_loaded_ = false;
    smoother_defaults_loaded_ = false;
    defaults_requested_ = false;
    speed_mode_applied_.reset();
  }

  bool defaultsLoaded()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return controller_defaults_loaded_ && smoother_defaults_loaded_;
  }

  void requestDefaultsIfNeeded()
  {
    std::shared_ptr<rclcpp::AsyncParametersClient> controller_client;
    std::shared_ptr<rclcpp::AsyncParametersClient> smoother_client;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (defaults_requested_) {
        return;
      }
      controller_client = controller_params_client_;
      smoother_client = smoother_params_client_;
      defaults_requested_ = true;
    }

    if (!controller_client || !smoother_client) {
      return;
    }
    if (!controller_client->service_is_ready() || !smoother_client->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] nav speed parameter services are not ready", name().c_str());
      std::lock_guard<std::mutex> lock(mutex_);
      defaults_requested_ = false;
      return;
    }

    controller_client->get_parameters(
      {"FollowPath.v_linear_min", "FollowPath.v_linear_max", "FollowPath.v_angular_min",
       "FollowPath.v_angular_max"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        try {
          const auto params = future.get();
          if (params.size() != 4) {
            throw std::runtime_error("unexpected controller parameter count");
          }
          std::lock_guard<std::mutex> lock(mutex_);
          controller_defaults_.v_linear_min = params[0].as_double();
          controller_defaults_.v_linear_max = params[1].as_double();
          controller_defaults_.v_angular_min = params[2].as_double();
          controller_defaults_.v_angular_max = params[3].as_double();
          controller_defaults_loaded_ = true;
        } catch (const std::exception & ex) {
          std::lock_guard<std::mutex> lock(mutex_);
          defaults_requested_ = false;
          RCLCPP_WARN(
            node_->get_logger(), "[%s] failed to read controller speed defaults: %s",
            name().c_str(), ex.what());
        }
      });

    smoother_client->get_parameters(
      {"max_velocity", "min_velocity"},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        try {
          const auto params = future.get();
          if (params.size() != 2) {
            throw std::runtime_error("unexpected smoother parameter count");
          }
          std::lock_guard<std::mutex> lock(mutex_);
          smoother_defaults_.max_velocity = params[0].as_double_array();
          smoother_defaults_.min_velocity = params[1].as_double_array();
          smoother_defaults_loaded_ = true;
        } catch (const std::exception & ex) {
          std::lock_guard<std::mutex> lock(mutex_);
          defaults_requested_ = false;
          RCLCPP_WARN(
            node_->get_logger(), "[%s] failed to read smoother speed defaults: %s",
            name().c_str(), ex.what());
        }
      });
  }

  bool setControllerSpeed(
    double v_linear_min, double v_linear_max, double v_angular_min, double v_angular_max)
  {
    return setParameters(
      controllerNode(),
      {
        rclcpp::Parameter("FollowPath.v_linear_min", v_linear_min),
        rclcpp::Parameter("FollowPath.v_linear_max", v_linear_max),
        rclcpp::Parameter("FollowPath.v_angular_min", v_angular_min),
        rclcpp::Parameter("FollowPath.v_angular_max", v_angular_max),
      });
  }

  bool setSmootherSpeed(
    const std::vector<double> & max_velocity, const std::vector<double> & min_velocity)
  {
    return setParameters(
      smootherNode(),
      {
        rclcpp::Parameter("max_velocity", max_velocity),
        rclcpp::Parameter("min_velocity", min_velocity),
      });
  }

  bool setParameters(
    const std::string & node_name, const std::vector<rclcpp::Parameter> & parameters)
  {
    std::shared_ptr<rclcpp::AsyncParametersClient> client;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (node_name == controller_client_node_) {
        client = controller_params_client_;
      } else if (node_name == smoother_client_node_) {
        client = smoother_params_client_;
      }
    }

    if (!client || !client->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        node_->get_logger(), *node_->get_clock(), 2000,
        "[%s] parameter service is unavailable: %s", name().c_str(), node_name.c_str());
      return false;
    }

    client->set_parameters(
      parameters,
      [this, node_name](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
                          future) {
        try {
          for (const auto & result : future.get()) {
            if (!result.successful) {
              RCLCPP_WARN(
                node_->get_logger(), "[%s] set parameters failed for %s: %s", name().c_str(),
                node_name.c_str(), result.reason.c_str());
              return;
            }
          }
        } catch (const std::exception & ex) {
          RCLCPP_WARN(
            node_->get_logger(), "[%s] set parameters failed for %s: %s", name().c_str(),
            node_name.c_str(), ex.what());
        }
      });
    return true;
  }

  std::string controllerNode()
  {
    std::string value;
    getInput("controller_node", value);
    return value;
  }

  std::string smootherNode()
  {
    std::string value;
    getInput("smoother_node", value);
    return value;
  }

  bool isInsidePolygon(
    double px, double py, const std::vector<geometry_msgs::msg::Point> & polygon) const
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

  std::vector<geometry_msgs::msg::Point> parsePolygon(const std::string & polygon_str) const
  {
    std::vector<geometry_msgs::msg::Point> polygon;
    std::istringstream ss(polygon_str);
    std::string token;
    while (std::getline(ss, token, ';')) {
      const auto comma = token.find(',');
      if (comma == std::string::npos) {
        throw std::runtime_error("Invalid polygon token: " + token);
      }

      geometry_msgs::msg::Point point;
      point.x = std::stod(token.substr(0, comma));
      point.y = std::stod(token.substr(comma + 1));
      polygon.push_back(point);
    }

    if (polygon.size() < 3) {
      throw std::runtime_error("Polygon needs at least 3 vertices");
    }
    return polygon;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<robot_msgs::msg::ModeControl>::SharedPtr mode_pub_;
  std::string mode_topic_;

  std::mutex mutex_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_params_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_params_client_;
  std::string controller_client_node_;
  std::string smoother_client_node_;

  bool rough_road_active_ = false;
  std::optional<bool> speed_mode_applied_;
  bool defaults_requested_ = false;
  bool controller_defaults_loaded_ = false;
  bool smoother_defaults_loaded_ = false;
  ControllerDefaults controller_defaults_;
  SmootherDefaults smoother_defaults_;
};

}  // namespace pb2025_sentry_behavior

BT_REGISTER_ROS_NODES(factory, params)
{
  factory.registerNodeType<pb2025_sentry_behavior::RoughRoadSpeedControl>(
    "RoughRoadSpeedControl", params);
}
