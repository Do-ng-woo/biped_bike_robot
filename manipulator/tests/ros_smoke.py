#!/usr/bin/env python3
"""Hardware-free integration smoke test for the native ROS graph."""
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.config import load_config
from common.postures import load_reference_postures


class SmokeNode(Node):
    def __init__(self):
        super().__init__("manipulator_smoke_test")
        cfg = load_config()
        topics = cfg["topics"]
        self.lower_names = list(cfg["lower_body"]["joint_names"])
        self.lower = list(load_reference_postures(cfg).kneeling[:12])
        self.arm_names = list(cfg["arm"]["follower_joint_names"])
        self.leader_names = list(cfg["arm"]["leader_joint_names"])
        self.mode = "UNKNOWN"
        self.saw_kneeling = False
        self.saw_active_whole_body = False
        self.bad_wheel = False
        self.request_sent = False
        self.started = time.monotonic()
        self.joint_pub = self.create_publisher(JointState, topics["joint_states"], 10)
        self.mode_pub = self.create_publisher(String, topics["mode_request"], 10)
        self.leader_pub = self.create_publisher(JointTrajectory, topics["leader_trajectory"], 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(JointTrajectory, topics["hardware_trajectory"], self.on_hardware, 10)
        self.create_subscription(Float64MultiArray, topics["wheel_command"], self.on_wheel, 10)
        self.timer = self.create_timer(0.05, self.tick)

    def on_mode(self, msg: String):
        self.mode = str(json.loads(msg.data).get("mode", "UNKNOWN"))
        self.saw_kneeling |= self.mode == "KNEELING"

    def on_hardware(self, msg: JointTrajectory):
        if not msg.points:
            return
        names = self.lower_names + self.arm_names
        if msg.joint_names == names and len(msg.points[-1].positions) == 17:
            lower = list(msg.points[-1].positions[:12])
            if all(abs(a - b) < 1e-6 for a, b in zip(lower, self.lower)):
                self.saw_active_whole_body |= self.mode == "TELEOP"

    def on_wheel(self, msg: Float64MultiArray):
        self.bad_wheel |= list(msg.data) != [0.0, 0.0]

    def tick(self):
        elapsed = time.monotonic() - self.started
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = self.lower_names + self.arm_names
        state.position = self.lower + [0.0] * 5
        self.joint_pub.publish(state)
        if elapsed > 0.3 and not self.request_sent:
            request = String()
            request.data = '{"mode":"TELEOP"}'
            self.mode_pub.publish(request)
            self.request_sent = True
        if elapsed > 1.0:
            leader = JointTrajectory()
            leader.header.stamp = self.get_clock().now().to_msg()
            leader.joint_names = self.leader_names
            point = JointTrajectoryPoint()
            point.positions = [0.1, -0.1, 0.1, -0.1, 0.1]
            leader.points = [point]
            self.leader_pub.publish(leader)
        if elapsed > 3.0:
            ok = self.saw_kneeling and self.mode == "TELEOP" and self.saw_active_whole_body and not self.bad_wheel
            print(json.dumps({
                "ok": ok, "mode": self.mode, "saw_kneeling": self.saw_kneeling,
                "whole_body_17": self.saw_active_whole_body, "wheel_zero": not self.bad_wheel,
            }))
            self.exit_code = 0 if ok else 1
            rclpy.shutdown()


def main() -> int:
    rclpy.init()
    node = SmokeNode()
    node.exit_code = 1
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return node.exit_code


if __name__ == "__main__":
    sys.exit(main())
