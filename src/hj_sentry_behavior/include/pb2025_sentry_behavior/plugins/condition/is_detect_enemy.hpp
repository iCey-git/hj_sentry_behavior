#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_

#include <string>

#include "auto_aim_interfaces/msg/armors.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that checks for the presence of an enemy target
 * returns SUCCESS if an enemy is detected
 */
class IsDetectEnemyCondition : public BT::SimpleConditionNode
{
public:
  IsDetectEnemyCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkEnemy();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsDetectEnemyCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_DETECT_ENEMY_HPP_
