#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import CompetitionInfo, RfidStatus


class FakeReferee(Node):
    def __init__(self):
        super().__init__('fake_referee')

        self.declare_parameter('game_state', 4)
        self.declare_parameter('remain_time', 300)
        self.declare_parameter('our_sentry_hp', 400)
        self.declare_parameter('publish_rate', 10.0)

        self.comp_pub = self.create_publisher(
            CompetitionInfo, '/competition_info', 10)
        self.rfid_pub = self.create_publisher(
            RfidStatus, '/rfid_status', 10)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Fake Referee Started at {rate} Hz")

    def timer_callback(self):
        game_state = self.get_parameter('game_state').value
        remain_time = self.get_parameter('remain_time').value
        our_sentry_hp = self.get_parameter('our_sentry_hp').value

        comp = CompetitionInfo()
        comp.game_state = game_state
        comp.remain_time = remain_time
        comp.our_sentry_hp = our_sentry_hp
        comp.shooter_17mm_1_barrel_heat = 0
        comp.projectile_allowance_17mm = 100
        comp.our_outpost_hp = 1500
        comp.enemy_outpost_hp = 1500
        comp.enemy_sentry_hp = 400
        comp.our_base_hp = 5000
        comp.enemy_base_hp = 5000
        comp.remain_bullet = 500
        comp.remain_energy = 60
        self.comp_pub.publish(comp)

        rfid = RfidStatus()
        rfid.friendly_fortress_gain_point = RfidStatus.NOT_DETECTED
        rfid.friendly_supply_zone_non_exchange = RfidStatus.NOT_DETECTED
        rfid.friendly_supply_zone_exchange = RfidStatus.NOT_DETECTED
        rfid.center_gain_point = RfidStatus.NOT_DETECTED
        self.rfid_pub.publish(rfid)


def main(args=None):
    rclpy.init(args=args)
    node = FakeReferee()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
