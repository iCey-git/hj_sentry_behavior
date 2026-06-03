#include <limits>

#include "gtest/gtest.h"
#include "pb2025_sentry_behavior/omni_perception_utils.hpp"

namespace
{

robot_msgs::msg::OmniPerception makeTargetInfo(int8_t state, uint8_t armor_name, float distance)
{
  robot_msgs::msg::OmniPerception msg;
  msg.state = state;
  msg.armor_name = armor_name;
  msg.distance = distance;
  return msg;
}

}  // namespace

TEST(OmniPerceptionUtilsTest, StopGateAcceptsTrackedCombatTargetWithinDistance)
{
  const auto msg = makeTargetInfo(1, 5, 4.0F);

  EXPECT_TRUE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_TRUE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, StopGateRejectsTrackedOutpostButPostureKeepsItVisible)
{
  const auto msg = makeTargetInfo(1, 6, 4.0F);

  EXPECT_FALSE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_TRUE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, OmniDetectionOnlyStopsButDoesNotDrivePosture)
{
  const auto msg = makeTargetInfo(2, 3, -1.0F);

  EXPECT_TRUE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_FALSE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, InvalidStateDoesNotTriggerStopOrPosture)
{
  const auto msg = makeTargetInfo(3, 3, 4.0F);

  EXPECT_FALSE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_FALSE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, InvalidArmorDoesNotTriggerStopOrPosture)
{
  const auto msg = makeTargetInfo(1, 8, 4.0F);

  EXPECT_FALSE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_FALSE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, NegativeDistanceInvalidatesTrackedTarget)
{
  const auto msg = makeTargetInfo(1, 3, -0.1F);

  EXPECT_FALSE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_FALSE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}

TEST(OmniPerceptionUtilsTest, NonFiniteDistanceInvalidatesTrackedTarget)
{
  const auto msg = makeTargetInfo(1, 3, std::numeric_limits<float>::infinity());

  EXPECT_FALSE(pb2025_sentry_behavior::shouldStopForTargetInfo(msg, 8.0F));
  EXPECT_FALSE(pb2025_sentry_behavior::isEnemyVisibleForPosture(msg, 8.0F));
}
