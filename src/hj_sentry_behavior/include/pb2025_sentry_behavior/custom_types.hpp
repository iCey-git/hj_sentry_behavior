#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#ifndef PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_
#define PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_

namespace BT
{
template <>
geometry_msgs::msg::PoseStamped convertFromString(StringView key)
{
  auto parts = BT::splitString(key, ';');
  if (parts.size() == 7) {
    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = convertFromString<double>(parts[2]);
    output.pose.orientation.x = convertFromString<double>(parts[3]);
    output.pose.orientation.y = convertFromString<double>(parts[4]);
    output.pose.orientation.z = convertFromString<double>(parts[5]);
    output.pose.orientation.w = convertFromString<double>(parts[6]);
    return output;
  } else if (parts.size() == 3) {
    tf2::Quaternion quaternion;
    auto goal_yaw = convertFromString<double>(parts[2]);
    quaternion.setRPY(0, 0, goal_yaw);
    geometry_msgs::msg::PoseStamped output;
    output.pose.position.x = convertFromString<double>(parts[0]);
    output.pose.position.y = convertFromString<double>(parts[1]);
    output.pose.position.z = 0.0;
    output.pose.orientation = tf2::toMsg(quaternion);
    return output;
  } else {
    throw RuntimeError("Invalid input");
  }
}
}  // namespace BT

#endif  // PB2025_SENTRY_BEHAVIOR__CUSTOM_TYPES_HPP_
