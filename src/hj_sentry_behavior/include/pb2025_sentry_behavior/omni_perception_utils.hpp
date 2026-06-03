#ifndef PB2025_SENTRY_BEHAVIOR__OMNI_PERCEPTION_UTILS_HPP_
#define PB2025_SENTRY_BEHAVIOR__OMNI_PERCEPTION_UTILS_HPP_

#include <cmath>

#include "robot_msgs/msg/omni_perception.hpp"

namespace pb2025_sentry_behavior
{

inline bool isValidOmniPerceptionArmorName(uint8_t armor_name)
{
  return armor_name <= 7U;
}

inline bool isStopArmorName(uint8_t armor_name)
{
  return armor_name <= 5U;
}

inline bool isValidTrackedDistance(float distance)
{
  return std::isfinite(distance) && distance >= 0.0F;
}

inline bool shouldStopForTargetInfo(
  const robot_msgs::msg::OmniPerception & target_info, float max_distance)
{
  if (!isValidOmniPerceptionArmorName(target_info.armor_name)) {
    return false;
  }

  if (target_info.state == 2) {
    return true;
  }

  if (target_info.state != 1) {
    return false;
  }
  if (!isStopArmorName(target_info.armor_name) || !isValidTrackedDistance(target_info.distance)) {
    return false;
  }

  return target_info.distance <= max_distance;
}

inline bool isEnemyVisibleForPosture(
  const robot_msgs::msg::OmniPerception & target_info, float max_distance)
{
  if (target_info.state != 1) {
    return false;
  }
  if (
    !isValidOmniPerceptionArmorName(target_info.armor_name) ||
    !isValidTrackedDistance(target_info.distance))
  {
    return false;
  }

  return target_info.distance <= max_distance;
}

}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__OMNI_PERCEPTION_UTILS_HPP_
