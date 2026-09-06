#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.config import load_config
from common.joints import ordered_positions
from common.motion import build_smooth_samples
from common.postures import load_reference_postures


def duration_message(seconds: float) -> Duration:
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return Duration(sec=sec, nanosec=nanosec)


class SmoothReadyPublisher(Node):
    def __init__(self, config_path: str | None, duration_sec: float, rate_hz: float):
        super().__init__("manipulator_smooth_ready")
        self.cfg = load_config(config_path)
        reference = load_reference_postures(self.cfg)
        self.names = list(reference.joint_names)
        self.target = list(reference.ready)
        self.duration_sec = duration_sec
        self.rate_hz = rate_hz
        self.published_at: float | None = None
        self.publisher = self.create_publisher(
            JointTrajectory, self.cfg["topics"]["hardware_trajectory"], 10
        )
        self.subscription = self.create_subscription(
            JointState,
            self.cfg["topics"]["joint_states"],
            self.on_joint_state,
            10,
        )
        self.get_logger().info(
            f"Waiting for one complete measured {len(self.names)}-joint state"
        )

    def on_joint_state(self, msg: JointState) -> None:
        if self.published_at is not None:
            return
        start = ordered_positions(msg.name, msg.position, self.names)
        if start is None:
            return

        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = self.names
        for seconds, positions in build_smooth_samples(
            start, self.target, self.duration_sec, self.rate_hz
        ):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = duration_message(seconds)
            trajectory.points.append(point)

        self.publisher.publish(trajectory)
        self.published_at = time.monotonic()
        self.get_logger().warn(
            f"Published {len(trajectory.points)}-point smooth ready trajectory "
            f"from measured pose over {self.duration_sec:.1f}s"
        )

    @property
    def finished(self) -> bool:
        return self.published_at is not None and (
            time.monotonic() - self.published_at >= self.duration_sec + 0.25
        )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--state-timeout", type=float, default=8.0)
    args, ros_args = parser.parse_known_args()
    if args.duration <= 0.0 or args.rate <= 0.0 or args.state_timeout <= 0.0:
        parser.error("duration, rate, and state-timeout must be positive")

    rclpy.init(args=ros_args)
    node = SmoothReadyPublisher(args.config, args.duration, args.rate)
    deadline = time.monotonic() + args.state_timeout
    exit_code = 0
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.published_at is None and time.monotonic() >= deadline:
                node.get_logger().error(
                    "Timed out waiting for a complete measured joint state; "
                    "ready trajectory was not sent"
                )
                exit_code = 2
                break
    except KeyboardInterrupt:
        exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
