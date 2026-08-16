#!/usr/bin/env python3
import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bike_transform_sequence import JOINT_NAMES, REVERT_SEQUENCE


class RevertBikePublisher(Node):
    def __init__(self):
        super().__init__('revert_bike_publisher')
        self.declare_parameter('stage_duration_sec', 3.0)
        self.stage_duration_sec = max(
            0.1,
            float(self.get_parameter('stage_duration_sec').value),
        )
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )
        self.published_at = None
        self.next_stage_to_log = 0
        self.timer = self.create_timer(1.0, self.publish_transform)

    @staticmethod
    def _duration(seconds):
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        return Duration(sec=sec, nanosec=nanosec)

    def publish_transform(self):
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().warn(
                'Waiting for /joint_trajectory_controller/joint_trajectory subscriber...'
            )
            return

        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)

        for index, positions in enumerate(REVERT_SEQUENCE, start=1):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = self._duration(index * self.stage_duration_sec)
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.published_at = time.monotonic()
        self.get_logger().info(
            'Published supported bike revert: '
            f'{len(msg.points)} stages x {self.stage_duration_sec:.1f}s'
        )
        self.log_due_stages()
        self.timer.cancel()

    @staticmethod
    def _format_angle(value):
        return f'{math.degrees(value):.1f} deg'

    def _stage_changes(self, stage_index):
        target = REVERT_SEQUENCE[stage_index]
        if stage_index == 0:
            return [
                f'{name} -> {self._format_angle(value)}'
                for name, value in zip(JOINT_NAMES, target)
                if not math.isclose(value, 0.0, abs_tol=1e-9)
            ]

        previous = REVERT_SEQUENCE[stage_index - 1]
        return [
            f'{name}: {self._format_angle(before)} -> {self._format_angle(after)}'
            for name, before, after in zip(JOINT_NAMES, previous, target)
            if not math.isclose(before, after, abs_tol=1e-9)
        ]

    def log_due_stages(self):
        if self.published_at is None:
            return

        elapsed = time.monotonic() - self.published_at
        while self.next_stage_to_log < len(REVERT_SEQUENCE):
            stage_start = self.next_stage_to_log * self.stage_duration_sec
            if elapsed < stage_start:
                break

            stage_number = self.next_stage_to_log + 1
            changes = self._stage_changes(self.next_stage_to_log)
            change_text = '\n  '.join(changes) if changes else 'no joint changes (hold pose)'
            self.get_logger().info(
                f'Stage {stage_number}/{len(REVERT_SEQUENCE)} '
                f'({stage_start:.1f}-{stage_start + self.stage_duration_sec:.1f}s)\n'
                f'  {change_text}'
            )
            self.next_stage_to_log += 1

    @property
    def playback_finished(self):
        if self.published_at is None:
            return False
        playback_time = len(REVERT_SEQUENCE) * self.stage_duration_sec
        return time.monotonic() - self.published_at >= playback_time


def main(args=None):
    rclpy.init(args=args)
    node = RevertBikePublisher()
    try:
        while rclpy.ok() and not node.playback_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.log_due_stages()
        if node.published_at is not None:
            node.get_logger().info('Bike revert trajectory completed.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
