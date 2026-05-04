#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from robot_msgs.msg import CompetitionInfo, RfidStatus


SCENARIOS = {
    # Game not started: check_game_start should keep the chassis stopped.
    "not_started": {
        "game_state": 0,
        "our_sentry_hp": 400,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 420,
    },
    # Normal early state: opening route should run first, then pressure outpost.
    "opening_pressure_outpost": {
        "game_state": 4,
        "our_sentry_hp": 400,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 360,
    },
    # HP below supply_hp_min: check_supply should take priority after opening.
    "need_supply_low_hp": {
        "game_state": 4,
        "our_sentry_hp": 100,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 300,
    },
    # Ammo below supply_ammo_min: check_supply should also trigger.
    "need_supply_low_ammo": {
        "game_state": 4,
        "our_sentry_hp": 400,
        "remain_bullet": 10,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 300,
    },
    # Healthy again: IsStatusOK should pass after low-HP recovery.
    "supply_recovered": {
        "game_state": 4,
        "our_sentry_hp": 400,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 260,
        "friendly_supply_zone_non_exchange": True,
    },
    # Base HP below base_protect_hp_threshold: protect_base should take priority.
    "protect_base": {
        "game_state": 4,
        "our_sentry_hp": 400,
        "remain_bullet": 500,
        "our_base_hp": 1000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 240,
    },
    # Enemy outpost dead: pressure_outpost stops matching and patrol_by_phase is fallback.
    "patrol_after_outpost_down": {
        "game_state": 4,
        "our_sentry_hp": 400,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 0,
        "stage_remain_time": 180,
        "center_gain_point": True,
    },
    # Sentry dead: check_death should take highest priority after game start.
    "sentry_dead": {
        "game_state": 4,
        "our_sentry_hp": 0,
        "remain_bullet": 500,
        "our_base_hp": 5000,
        "enemy_outpost_hp": 1500,
        "stage_remain_time": 120,
    },
}


class FakeRefereeScenarios(Node):
    def __init__(self):
        super().__init__("fake_referee_scenarios")

        self.declare_parameter("scenario", "opening_pressure_outpost")
        self.declare_parameter("publish_rate", 10.0)

        self.comp_pub = self.create_publisher(CompetitionInfo, "/competition_info", 10)
        self.rfid_pub = self.create_publisher(RfidStatus, "/rfid_status", 10)

        rate = float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info(
            "Fake referee scenarios started. Available scenarios: "
            + ", ".join(sorted(SCENARIOS.keys()))
        )

    def timer_callback(self):
        scenario_name = str(self.get_parameter("scenario").value)
        scenario = SCENARIOS.get(scenario_name)
        if scenario is None:
            self.get_logger().warn(
                f"Unknown scenario '{scenario_name}', using opening_pressure_outpost",
                throttle_duration_sec=2.0,
            )
            scenario = SCENARIOS["opening_pressure_outpost"]

        data = copy.deepcopy(scenario)
        comp = CompetitionInfo()
        comp.game_state = int(data.get("game_state", 4))
        comp.our_outpost_hp = int(data.get("our_outpost_hp", 1500))
        comp.enemy_outpost_hp = int(data.get("enemy_outpost_hp", 1500))
        comp.remain_bullet = int(data.get("remain_bullet", 500))
        comp.enemy_sentry_hp = int(data.get("enemy_sentry_hp", 400))
        comp.our_sentry_hp = int(data.get("our_sentry_hp", 400))
        comp.our_base_hp = int(data.get("our_base_hp", 5000))
        comp.first_blood = int(data.get("first_blood", 0))
        comp.target_position_x = float(data.get("target_position_x", 0.0))
        comp.target_position_y = float(data.get("target_position_y", 0.0))
        comp.is_target_active = int(data.get("is_target_active", 0))
        comp.enemy_hero_hp = int(data.get("enemy_hero_hp", 500))
        comp.enemy_engineer_hp = int(data.get("enemy_engineer_hp", 500))
        comp.enemy_infantry_3_hp = int(data.get("enemy_infantry_3_hp", 500))
        comp.enemy_infantry_4_hp = int(data.get("enemy_infantry_4_hp", 500))
        comp.our_hero_hp = int(data.get("our_hero_hp", 500))
        comp.our_engineer_hp = int(data.get("our_engineer_hp", 500))
        comp.our_infantry_3_hp = int(data.get("our_infantry_3_hp", 500))
        comp.our_infantry_4_hp = int(data.get("our_infantry_4_hp", 500))
        comp.enemy_base_hp = int(data.get("enemy_base_hp", 5000))
        comp.remain_energy = int(data.get("remain_energy", 60))
        comp.stage_remain_time = int(data.get("stage_remain_time", 300))
        self.comp_pub.publish(comp)

        rfid = RfidStatus()
        rfid.friendly_fortress_gain_point = self._rfid_value(data, "friendly_fortress_gain_point")
        rfid.friendly_supply_zone_non_exchange = self._rfid_value(
            data, "friendly_supply_zone_non_exchange"
        )
        rfid.friendly_supply_zone_exchange = self._rfid_value(
            data, "friendly_supply_zone_exchange"
        )
        rfid.center_gain_point = self._rfid_value(data, "center_gain_point")
        self.rfid_pub.publish(rfid)

    @staticmethod
    def _rfid_value(data, key):
        return RfidStatus.DETECTED if bool(data.get(key, False)) else RfidStatus.NOT_DETECTED


def main(args=None):
    rclpy.init(args=args)
    node = FakeRefereeScenarios()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
