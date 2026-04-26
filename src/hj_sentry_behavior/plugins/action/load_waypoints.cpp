#include "pb2025_sentry_behavior/plugins/action/load_waypoints.hpp"

#include <fstream>
#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/basic_types.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace pb2025_sentry_behavior
{

LoadWaypointsAction::LoadWaypointsAction(
  const std::string & name, const BT::NodeConfig & conf)
: BT::SyncActionNode(name, conf)
{
}

namespace
{

std::string trim(const std::string & input)
{
  const auto begin = input.find_first_not_of(" \t\r\n\"");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = input.find_last_not_of(" \t\r\n\"");
  return input.substr(begin, end - begin + 1);
}

bool tryParseDouble(const std::string & input, double & output)
{
  try {
    std::size_t consumed = 0;
    output = std::stod(trim(input), &consumed);
    return consumed > 0;
  } catch (const std::exception &) {
    return false;
  }
}

std::string resolveWaypointPath(const std::string & path)
{
  namespace fs = std::filesystem;
  const fs::path raw(path);
  if (raw.is_absolute() || fs::exists(raw)) {
    return path;
  }

  const std::string normalized = raw.generic_string();
  const auto slash = normalized.find('/');
  if (slash != std::string::npos) {
    const std::string package_name = normalized.substr(0, slash);
    const std::string relative_path = normalized.substr(slash + 1);
    try {
      const fs::path share_path = ament_index_cpp::get_package_share_directory(package_name);
      const fs::path resolved = share_path / relative_path;
      if (fs::exists(resolved)) {
        return resolved.string();
      }
    } catch (const std::exception &) {
      // Fall through to source-tree fallback.
    }
  }

  if (slash != std::string::npos) {
    const fs::path source_resolved = fs::path(ROOT_DIR) / normalized.substr(slash + 1);
    if (fs::exists(source_resolved)) {
      return source_resolved.string();
    }
  }

  return path;
}

double parseWaitSeconds(const std::vector<std::string> & tokens, std::size_t start_index)
{
  for (std::size_t i = start_index; i < tokens.size(); ++i) {
    std::string token = trim(tokens[i]);
    if (token.empty() || token == "end") {
      continue;
    }
    const auto colon = token.find(':');
    if (colon != std::string::npos) {
      const std::string key = token.substr(0, colon);
      if (key == "wait" || key == "stop") {
        double wait_sec = 0.0;
        if (tryParseDouble(token.substr(colon + 1), wait_sec)) {
          return wait_sec;
        }
      }
    } else if (token == "wait" || token == "stop") {
      return 0.0;
    }
  }
  return 0.0;
}

bool parseSimpleWaypoint(
  const std::vector<std::string> & tokens, geometry_msgs::msg::PoseStamped & pose,
  double & wait_sec)
{
  if (tokens.size() < 4) {
    return false;
  }

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  if (
    !tryParseDouble(tokens[0], x) ||
    !tryParseDouble(tokens[1], y) ||
    !tryParseDouble(tokens[2], yaw) ||
    !tryParseDouble(tokens[3], wait_sec))
  {
    return false;
  }

  pose.header.frame_id = "map";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  pose.pose.orientation = tf2::toMsg(q);
  return true;
}

bool parseCodWaypoint(
  const std::vector<std::string> & tokens, geometry_msgs::msg::PoseStamped & pose,
  double & wait_sec)
{
  if (tokens.size() < 8) {
    return false;
  }

  double values[7] = {};
  for (std::size_t i = 0; i < 7; ++i) {
    if (!tryParseDouble(tokens[i + 1], values[i])) {
      return false;
    }
  }

  pose.header.frame_id = "map";
  pose.pose.position.x = values[0];
  pose.pose.position.y = values[1];
  pose.pose.position.z = values[2];
  pose.pose.orientation.x = values[3];
  pose.pose.orientation.y = values[4];
  pose.pose.orientation.z = values[5];
  pose.pose.orientation.w = values[6];
  wait_sec = parseWaitSeconds(tokens, 8);
  return true;
}

}  // namespace

BT::PortsList LoadWaypointsAction::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "waypoint_file", "", "CSV 路线文件路径，支持包内路径和 COD waypoint_editor 格式"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
      "waypoints", "加载的航点列表"),
    BT::OutputPort<std::vector<double>>("wait_times", "每个航点的等待时长(秒)"),
    BT::OutputPort<int>("total_waypoints", "航点总数"),
    BT::OutputPort<int>("wp_idx", "当前航点索引，重置为 0"),
  };
}

BT::NodeStatus LoadWaypointsAction::tick()
{
  auto waypoint_file = getInput<std::string>("waypoint_file");
  if (!waypoint_file || waypoint_file->empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LoadWaypoints"),
      "[%s] waypoint_file port is not set or empty", name().c_str());
    return BT::NodeStatus::FAILURE;
  }

  const std::string file_path = resolveWaypointPath(waypoint_file.value());

  // 路线子树使用 SequenceWithMemory，LoadWaypoints 只在一次路线执行开始时 tick。
  // 因此即使路线文件未变，也要重置索引，保证重新进入路线时从起点开始。
  if (file_path == last_loaded_file_) {
    setOutput("wp_idx", 0);
    return BT::NodeStatus::SUCCESS;
  }

  std::ifstream file(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LoadWaypoints"),
      "[%s] Cannot open waypoint file: %s", name().c_str(), file_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  std::vector<geometry_msgs::msg::PoseStamped> waypoints;
  std::vector<double> wait_times;

  std::string line;
  int line_num = 0;
  while (std::getline(file, line)) {
    line_num++;
    // 跳过空行和注释行
    if (line.empty() || line[0] == '#') {
      continue;
    }
    // 去除 Windows 换行符
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }

    std::istringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    geometry_msgs::msg::PoseStamped pose;
    double wait_sec = 0.0;
    if (!parseCodWaypoint(tokens, pose, wait_sec) && !parseSimpleWaypoint(tokens, pose, wait_sec)) {
      if (line_num == 1) {
        continue;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("LoadWaypoints"),
        "[%s] Skipping invalid waypoint line %d in %s",
        name().c_str(), line_num, file_path.c_str());
      continue;
    }

    waypoints.push_back(pose);
    wait_times.push_back(wait_sec);
  }

  if (waypoints.empty()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("LoadWaypoints"),
      "[%s] No valid waypoints found in %s", name().c_str(), file_path.c_str());
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("LoadWaypoints"),
    "[%s] Loaded %zu waypoints from %s",
    name().c_str(), waypoints.size(), file_path.c_str());

  last_loaded_file_ = file_path;
  setOutput("waypoints", waypoints);
  setOutput("wait_times", wait_times);
  setOutput("total_waypoints", static_cast<int>(waypoints.size()));
  setOutput("wp_idx", 0);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace pb2025_sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<pb2025_sentry_behavior::LoadWaypointsAction>("LoadWaypoints");
}
