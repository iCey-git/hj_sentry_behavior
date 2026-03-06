#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from robot_msgs.msg import CompetitionInfo, RfidStatus


class FakeRefereeBadStatus(Node):
    def __init__(self):
        super().__init__("fake_referee_bad_status")

        # Keep game started, but publish values that make IsStatusOK fail by default.
        self.declare_parameter("game_state", 4)
        self.declare_parameter("remain_time", 300)
        self.declare_parameter("our_sentry_hp", 100)  # < hp_min(200)
        self.declare_parameter("shooter_heat", 380)  # > heat_max(350)
        self.declare_parameter("remain_bullet", 500)
        self.declare_parameter("publish_rate", 10.0)

        self.comp_pub = self.create_publisher(CompetitionInfo, "/competition_info", 10)
        self.rfid_pub = self.create_publisher(RfidStatus, "/rfid_status", 10)

        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(f"Fake Referee Bad Status Started at {rate} Hz")

    def timer_callback(self):
        comp = CompetitionInfo()
        comp.game_state = int(self.get_parameter("game_state").value)
        comp.remain_time = int(self.get_parameter("remain_time").value)
        comp.our_sentry_hp = int(self.get_parameter("our_sentry_hp").value)
        comp.shooter_17mm_1_barrel_heat = int(self.get_parameter("shooter_heat").value)
        comp.remain_bullet = int(self.get_parameter("remain_bullet").value)

        comp.our_outpost_hp = 1500
        comp.enemy_outpost_hp = 1500
        comp.enemy_sentry_hp = 400
        comp.our_base_hp = 5000
        comp.enemy_base_hp = 5000
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
    node = FakeRefereeBadStatus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
