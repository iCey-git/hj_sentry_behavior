#ifndef PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_
#define PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_

#include <string>

#include "behaviortree_cpp/condition_node.h"
#include "robot_msgs/msg/rfid_status.hpp"
#include "rclcpp/rclcpp.hpp"

namespace pb2025_sentry_behavior
{
/**
 * @brief A BT::ConditionNode that get GameStatus from port and
 * returns SUCCESS when current game status and remain time is expected
 */
class IsRfidDetectedCondition : public BT::SimpleConditionNode
{
public:
  IsRfidDetectedCondition(const std::string & name, const BT::NodeConfig & config);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts();

private:
  /**
   * @brief Tick function for game status ports
   */
  BT::NodeStatus checkRfidStatus();

  rclcpp::Logger logger_ = rclcpp::get_logger("IsRfidDetectedCondition");
};
}  // namespace pb2025_sentry_behavior

#endif  // PB2025_SENTRY_BEHAVIOR__PLUGINS__CONDITION__IS_RFID_DETECTED_HPP_
