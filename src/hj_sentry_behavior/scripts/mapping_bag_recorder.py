#!/usr/bin/env python3
import os
import signal
import subprocess
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from robot_msgs.msg import CompetitionInfo


class MappingBagRecorder(Node):
    def __init__(self) -> None:
        super().__init__("mapping_bag_recorder")

        self.declare_parameter("competition_topic", "/competition_info")
        self.declare_parameter("script_path", "/home/hj/sentry_nav_26/record_match_text.sh")
        self.declare_parameter("start_game_state", 3)
        self.declare_parameter("stop_game_state", 5)
        self.declare_parameter("stop_timeout_sec", 20.0)

        self._competition_topic = str(self.get_parameter("competition_topic").value)
        self._script_path = str(self.get_parameter("script_path").value)
        self._start_game_state = int(self.get_parameter("start_game_state").value)
        self._stop_game_state = int(self.get_parameter("stop_game_state").value)
        self._stop_timeout_sec = float(self.get_parameter("stop_timeout_sec").value)

        self._process: Optional[subprocess.Popen] = None
        self._last_game_state: Optional[int] = None

        self._subscription = self.create_subscription(
            CompetitionInfo,
            self._competition_topic,
            self._competition_callback,
            10,
        )

        self.get_logger().info(
            "match text recorder ready, topic=%s start_state=%d stop_state=%d script=%s"
            % (
                self._competition_topic,
                self._start_game_state,
                self._stop_game_state,
                self._script_path,
            )
        )

    def _competition_callback(self, msg: CompetitionInfo) -> None:
        game_state = int(msg.game_state)

        if self._last_game_state != game_state:
            self.get_logger().info(
                "game_state changed: %s -> %d"
                % (
                    "None" if self._last_game_state is None else str(self._last_game_state),
                    game_state,
                )
            )
            self._last_game_state = game_state

        if game_state == self._start_game_state:
            self._start_recording()
            return

        if game_state == self._stop_game_state:
            self._stop_recording()

    def _start_recording(self) -> None:
        if self._process is not None:
            if self._process.poll() is None:
                return
            self.get_logger().warning(
                "previous recording process exited with code %d, cleaning up handle"
                % self._process.returncode
            )
            self._process = None

        if not os.path.isfile(self._script_path):
            self.get_logger().error("record script not found: %s" % self._script_path)
            return

        if not os.access(self._script_path, os.X_OK):
            self.get_logger().error("record script is not executable: %s" % self._script_path)
            return

        try:
            self._process = subprocess.Popen(
                [self._script_path],
                start_new_session=True,
                env=os.environ.copy(),
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error("failed to start record script: %s" % str(exc))
            self._process = None
            return

        self.get_logger().info("started recording, pid=%d" % self._process.pid)

    def _stop_recording(self) -> None:
        if self._process is None:
            return

        if self._process.poll() is not None:
            self.get_logger().info(
                "recording process already exited with code %d" % self._process.returncode
            )
            self._process = None
            return

        pid = self._process.pid
        self.get_logger().info("stopping recording, pid=%d" % pid)

        try:
            os.killpg(os.getpgid(pid), signal.SIGINT)
            self._process.wait(timeout=self._stop_timeout_sec)
            self.get_logger().info("recording saved successfully")
        except subprocess.TimeoutExpired:
            self.get_logger().warning(
                "recording did not stop in %.1fs, sending SIGTERM" % self._stop_timeout_sec
            )
            try:
                os.killpg(os.getpgid(pid), signal.SIGTERM)
                self._process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                self.get_logger().error("recording still alive, sending SIGKILL")
                os.killpg(os.getpgid(pid), signal.SIGKILL)
                self._process.wait()
        except ProcessLookupError:
            self.get_logger().warning("recording process group already gone")
        finally:
            self._process = None

    def destroy_node(self) -> bool:
        self._stop_recording()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MappingBagRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
