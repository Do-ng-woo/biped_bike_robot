#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.config import load_config
from common.joints import ordered_positions
from common.motion import build_smooth_samples
from common.postures import load_reference_postures


class ArmCommandArbiter(Node):
    def __init__(self, config_path: str | None = None):
        super().__init__("manipulator_arm_command_arbiter")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        arm = self.cfg["arm"]
        lower = self.cfg["lower_body"]
        self.names = list(arm["follower_joint_names"])
        self.lower_names = list(lower["joint_names"])
        reference = load_reference_postures(self.cfg)
        lower_count = len(self.lower_names)
        self.kneeling_positions = list(reference.kneeling)
        self.lower_positions = list(reference.kneeling[:lower_count])
        self.kneeling_arm_positions = list(reference.kneeling[lower_count:])
        self.kneeling_duration = float(lower["move_duration_sec"])
        self.minimum = [float(v) for v in arm["min_position"]]
        self.maximum = [float(v) for v in arm["max_position"]]
        self.rate = float(arm["publish_rate_hz"])
        self.timeout = float(arm["source_timeout_sec"])
        self.max_velocity = float(arm["max_velocity_rad_s"])
        self.duration = float(arm["command_duration_sec"])
        self.reset_position = list(self.kneeling_arm_positions)
        if len(self.lower_names) != len(self.lower_positions):
            raise ValueError("lower-body names and kneeling positions differ in length")
        if len(self.reset_position) != len(self.names):
            raise ValueError("arm reset position does not match arm joint count")
        self.mode = "IDLE"
        self.kneeling_sent = False
        self.record_state = "IDLE"
        self.sources: dict[str, tuple[float, list[float]]] = {}
        self.output: list[float] | None = None
        self.measured_whole: list[float] | None = None
        self.publisher = self.create_publisher(
            JointTrajectory, topics["hardware_trajectory"], 10
        )
        self.create_subscription(
            JointTrajectory, topics["teleop_target"],
            lambda msg: self.on_target("teleop", msg), 10,
        )
        self.create_subscription(
            JointTrajectory, topics["inference_target"],
            lambda msg: self.on_target("inference", msg), 10,
        )
        self.create_subscription(JointState, topics["joint_states"], self.on_joint_state, 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(String, topics["record_state"], self.on_record_state, 10)
        self.timer = self.create_timer(1.0 / self.rate, self.tick)

    def on_joint_state(self, msg: JointState):
        whole = ordered_positions(
            msg.name, msg.position, self.lower_names + self.names
        )
        if whole is not None:
            self.measured_whole = whole
        values = ordered_positions(msg.name, msg.position, self.names)
        if values is not None and (
            self.output is None or self.mode not in {"TELEOP", "RECORD", "INFERENCE"}
        ):
            self.output = values

    def on_mode(self, msg: String):
        previous_mode = self.mode
        try:
            self.mode = str(json.loads(msg.data).get("mode", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            self.mode = "IDLE"
        if self.mode not in {"TELEOP", "RECORD", "INFERENCE"}:
            self.sources.clear()
        if self.mode == "KNEELING" and previous_mode != "KNEELING":
            self.kneeling_sent = False

    def on_target(self, source: str, msg: JointTrajectory):
        if not msg.points:
            return
        values = ordered_positions(msg.joint_names, msg.points[-1].positions, self.names)
        if values is not None:
            self.sources[source] = (time.monotonic(), values)

    def on_record_state(self, msg: String):
        try:
            self.record_state = str(json.loads(msg.data).get("state", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            self.record_state = "IDLE"

    def tick(self):
        if self.mode == "KNEELING":
            if self.measured_whole is not None and not self.kneeling_sent:
                self.publish_kneeling_trajectory()
                self.kneeling_sent = True
            return
        if self.mode not in {"TELEOP", "RECORD", "INFERENCE"} or self.output is None:
            return
        target = self.output
        if self.mode == "RECORD" and self.record_state == "RESETTING":
            target = self.reset_position
        else:
            source = "inference" if self.mode == "INFERENCE" else "teleop" if self.mode in {"TELEOP", "RECORD"} else None
            if source in self.sources:
                stamp, candidate = self.sources[source]
                if time.monotonic() - stamp <= self.timeout:
                    target = candidate
        max_step = self.max_velocity / self.rate
        self.output = [
            min(max(self.output[i] + min(max(target[i] - self.output[i], -max_step), max_step), self.minimum[i]), self.maximum[i])
            for i in range(len(self.names))
        ]
        self.publish_whole_body(self.output, self.duration)

    def publish_whole_body(self, arm_positions: list[float], duration: float):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        # The arbiter is the sole hardware trajectory publisher. Every command has
        # the fixed 12-axis lower body and the selected 5-axis arm target.
        msg.joint_names = self.lower_names + self.names
        point = JointTrajectoryPoint()
        point.positions = self.lower_positions + list(arm_positions)
        sec = int(duration)
        point.time_from_start = Duration(sec=sec, nanosec=int((duration - sec) * 1e9))
        msg.points = [point]
        self.publisher.publish(msg)

    def publish_kneeling_trajectory(self):
        """Send one smooth transition to the exact DEEP_SQUAT_ARMS_UP pose."""
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.lower_names + self.names
        for seconds, positions in build_smooth_samples(
            self.measured_whole,
            self.kneeling_positions,
            self.kneeling_duration,
            50.0,
        ):
            point = JointTrajectoryPoint()
            point.positions = positions
            sec = int(seconds)
            nanosec = int(round((seconds - sec) * 1e9))
            if nanosec >= 1_000_000_000:
                sec += 1
                nanosec -= 1_000_000_000
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().warn(
            f"Published {len(msg.points)}-point smooth DEEP_SQUAT_ARMS_UP trajectory "
            f"over {self.kneeling_duration:.1f}s"
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ArmCommandArbiter(args.config)
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
