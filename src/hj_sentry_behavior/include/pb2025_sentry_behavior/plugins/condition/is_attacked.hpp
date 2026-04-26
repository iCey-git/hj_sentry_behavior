#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsAttackedCondition : public BT::SimpleConditionNode
{
public:
  IsAttackedCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for robot_status ports
   */
  BT::NodeStatus checkIsAttacked();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsAttackedCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_ATTACKED_HPP_
