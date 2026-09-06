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
from common.joints import JointMapping, ordered_positions


class LeaderMapper(Node):
    def __init__(self, config_path: str | None = None):
        super().__init__("leader_to_biped_mapper")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        arm = self.cfg["arm"]
        self.mapping = JointMapping.from_config(arm)
        self.align_on_enable = bool(arm.get("align_on_enable", True))
        self.duration = float(arm["command_duration_sec"])
        self.mode = "IDLE"
        self.last_mode = "IDLE"
        self.latest_leader: list[float] | None = None
        self.latest_follower: list[float] | None = None
        self.runtime_offset: list[float] | None = None
        self.record_state = "IDLE"
        self.publisher = self.create_publisher(JointTrajectory, topics["teleop_target"], 10)
        self.create_subscription(
            JointTrajectory, topics["leader_trajectory"], self.on_leader, 10
        )
        self.create_subscription(JointState, topics["joint_states"], self.on_follower, 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(String, topics["record_state"], self.on_record_state, 10)

    def on_follower(self, msg: JointState):
        values = ordered_positions(msg.name, msg.position, self.mapping.follower_names)
        if values is not None:
            self.latest_follower = values

    def on_mode(self, msg: String):
        try:
            mode = str(json.loads(msg.data).get("mode", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            mode = "IDLE"
        self.last_mode, self.mode = self.mode, mode
        if mode in {"TELEOP", "RECORD"} and self.last_mode not in {"TELEOP", "RECORD"}:
            self.runtime_offset = None
            self.try_align()

    def try_align(self):
        if self.align_on_enable and self.latest_leader is not None and self.latest_follower is not None:
            self.runtime_offset = self.mapping.alignment_offsets(
                self.latest_leader, self.latest_follower
            )
            self.get_logger().info("Aligned leader zero to current follower pose")

    def on_record_state(self, msg: String):
        try:
            state = str(json.loads(msg.data).get("state", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            state = "IDLE"
        # The follower moved to reset_position without the leader. Re-align before
        # accepting the next demonstration so there is no return jump.
        if self.record_state == "RESETTING" and state == "WARMUP":
            self.runtime_offset = None
            self.try_align()
        self.record_state = state

    def on_leader(self, msg: JointTrajectory):
        if not msg.points:
            return
        leader = self.mapping.extract(msg.joint_names, msg.points[-1].positions)
        if leader is None:
            self.get_logger().warn("Leader trajectory does not contain all configured joints", throttle_duration_sec=2.0)
            return
        self.latest_leader = leader
        if self.mode not in {"TELEOP", "RECORD"}:
            return
        if self.mode == "RECORD" and self.record_state == "RESETTING":
            return
        if self.align_on_enable and self.runtime_offset is None:
            self.try_align()
            if self.runtime_offset is None:
                return
        target = self.mapping.map(leader, self.runtime_offset)
        out = JointTrajectory()
        out.header.stamp = self.get_clock().now().to_msg()
        out.joint_names = list(self.mapping.follower_names)
        point = JointTrajectoryPoint()
        point.positions = target
        sec = int(self.duration)
        point.time_from_start = Duration(sec=sec, nanosec=int((self.duration - sec) * 1e9))
        out.points = [point]
        self.publisher.publish(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = LeaderMapper(args.config)
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
