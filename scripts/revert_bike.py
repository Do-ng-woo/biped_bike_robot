#!/usr/bin/env python3

import math
import time

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bike_transform_sequence import (
    JOINT_NAMES,
    REVERT_POINT_TIME_FACTORS,
    REVERT_RISE_START_FACTOR,
    REVERT_SEQUENCE,
)


class RevertBikePublisher(Node):
    def __init__(self):
        super().__init__('revert_bike_publisher')

        # 전체 기본 stage 시간.
        # 기존과 동일하게 3초를 기준으로 사용한다.
        self.declare_parameter(
            'stage_duration_sec',
            3.0,
        )

        # 일어서기 시작(REVERT_RISE_START_FACTOR) 이후 구간만
        # 추가로 천천히 재생한다.
        # 1.0 = 원래 속도
        # 1.5 = rise 구간 1.5배 느리게
        # 2.0 = rise 구간 2배 느리게
        self.declare_parameter(
            'rise_time_scale',
            1.5,
        )

        self.stage_duration_sec = max(
            0.1,
            float(
                self.get_parameter(
                    'stage_duration_sec'
                ).value
            ),
        )

        self.rise_time_scale = max(
            1.0,
            float(
                self.get_parameter(
                    'rise_time_scale'
                ).value
            ),
        )

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10,
        )

        self.published_at = None
        self.next_stage_to_log = 0

        # 실제 controller에 넣을 각 waypoint의 절대 시간.
        # rise 구간은 rise_time_scale을 적용해서 별도로 늘린다.
        self.waypoint_times = self._build_waypoint_times()

        self.timer = self.create_timer(
            1.0,
            self.publish_transform,
        )

        self.get_logger().info(
            'Revert publisher configured: '
            f'stage_duration={self.stage_duration_sec:.2f}s, '
            f'rise_time_scale={self.rise_time_scale:.2f}, '
            f'rise_start_factor={REVERT_RISE_START_FACTOR:.2f}'
        )

    @staticmethod
    def _duration(seconds):
        """
        Convert floating-point seconds to ROS Duration.

        Convert through integer nanoseconds first so values such as
        24.9 s cannot accidentally produce nanosec == 1_000_000_000
        because of floating-point rounding.
        """
        total_nanoseconds = int(
            round(seconds * 1_000_000_000)
        )

        sec, nanosec = divmod(
            total_nanoseconds,
            1_000_000_000,
        )

        return Duration(
            sec=sec,
            nanosec=nanosec,
        )

    def _factor_to_seconds(self, time_factor):
        """
        Convert a sequence time factor to actual trajectory time.

        Before REVERT_RISE_START_FACTOR:
            normal stage_duration_sec is used.

        After REVERT_RISE_START_FACTOR:
            only the remaining standing-up portion is stretched by
            rise_time_scale.
        """
        time_factor = float(time_factor)

        normal_factor = min(
            time_factor,
            REVERT_RISE_START_FACTOR,
        )

        rise_factor = max(
            0.0,
            time_factor - REVERT_RISE_START_FACTOR,
        )

        return (
            normal_factor * self.stage_duration_sec
            + rise_factor
            * self.stage_duration_sec
            * self.rise_time_scale
        )

    def _build_waypoint_times(self):
        """Validate time factors and build scaled waypoint times."""
        if not REVERT_POINT_TIME_FACTORS:
            raise RuntimeError(
                'REVERT_POINT_TIME_FACTORS is empty.'
            )

        previous_factor = None

        for factor in REVERT_POINT_TIME_FACTORS:
            factor = float(factor)

            if factor <= 0.0:
                raise RuntimeError(
                    'All REVERT_POINT_TIME_FACTORS must be positive.'
                )

            if (
                previous_factor is not None
                and factor <= previous_factor
            ):
                raise RuntimeError(
                    'REVERT_POINT_TIME_FACTORS must be '
                    'strictly increasing.'
                )

            previous_factor = factor

        return [
            self._factor_to_seconds(factor)
            for factor in REVERT_POINT_TIME_FACTORS
        ]

    def publish_transform(self):
        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().warn(
                'Waiting for '
                '/joint_trajectory_controller/joint_trajectory '
                'subscriber...'
            )
            return

        if len(REVERT_SEQUENCE) != len(
            REVERT_POINT_TIME_FACTORS
        ):
            self.get_logger().error(
                'REVERT_SEQUENCE and '
                'REVERT_POINT_TIME_FACTORS length mismatch: '
                f'{len(REVERT_SEQUENCE)} vs '
                f'{len(REVERT_POINT_TIME_FACTORS)}'
            )
            return

        if len(REVERT_SEQUENCE) != len(
            self.waypoint_times
        ):
            self.get_logger().error(
                'REVERT_SEQUENCE and waypoint time length mismatch: '
                f'{len(REVERT_SEQUENCE)} vs '
                f'{len(self.waypoint_times)}'
            )
            return

        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)

        for positions, waypoint_time in zip(
            REVERT_SEQUENCE,
            self.waypoint_times,
        ):
            if len(positions) != len(JOINT_NAMES):
                self.get_logger().error(
                    'Waypoint joint count mismatch: '
                    f'{len(positions)} positions for '
                    f'{len(JOINT_NAMES)} joints'
                )
                return

            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = self._duration(
                waypoint_time
            )

            msg.points.append(point)

        self.publisher_.publish(msg)

        self.published_at = time.monotonic()
        self.next_stage_to_log = 0

        total_playback_time = self.waypoint_times[-1]

        self.get_logger().info(
            'Published backward-biased bike revert: '
            f'{len(msg.points)} waypoints, '
            f'{total_playback_time:.1f}s playback'
        )

        self.log_due_stages()

        # trajectory는 한 번만 publish한다.
        self.timer.cancel()

    @staticmethod
    def _format_angle(value):
        return f'{math.degrees(value):.1f} deg'

    def _stage_changes(self, stage_index):
        target = REVERT_SEQUENCE[stage_index]

        if stage_index == 0:
            return [
                (
                    f'{name} -> '
                    f'{self._format_angle(value)}'
                )
                for name, value in zip(
                    JOINT_NAMES,
                    target,
                )
                if not math.isclose(
                    value,
                    0.0,
                    abs_tol=1e-9,
                )
            ]

        previous = REVERT_SEQUENCE[
            stage_index - 1
        ]

        return [
            (
                f'{name}: '
                f'{self._format_angle(before)} '
                f'-> '
                f'{self._format_angle(after)}'
            )
            for name, before, after in zip(
                JOINT_NAMES,
                previous,
                target,
            )
            if not math.isclose(
                before,
                after,
                abs_tol=1e-9,
            )
        ]

    def log_due_stages(self):
        if self.published_at is None:
            return

        elapsed = (
            time.monotonic()
            - self.published_at
        )

        while (
            self.next_stage_to_log
            < len(REVERT_SEQUENCE)
        ):
            stage_index = self.next_stage_to_log

            if stage_index == 0:
                stage_start = 0.0
            else:
                stage_start = self.waypoint_times[
                    stage_index - 1
                ]

            if elapsed < stage_start:
                break

            stage_end = self.waypoint_times[
                stage_index
            ]

            stage_number = stage_index + 1

            changes = self._stage_changes(
                stage_index
            )

            if changes:
                change_text = '\n  '.join(changes)
            else:
                change_text = (
                    'no joint changes '
                    '(hold pose)'
                )

            self.get_logger().info(
                f'Stage '
                f'{stage_number}/'
                f'{len(REVERT_SEQUENCE)} '
                f'({stage_start:.1f}'
                f'-{stage_end:.1f}s)\n'
                f'  {change_text}'
            )

            self.next_stage_to_log += 1

    @property
    def playback_finished(self):
        if self.published_at is None:
            return False

        playback_time = self.waypoint_times[-1]

        elapsed = (
            time.monotonic()
            - self.published_at
        )

        return elapsed >= playback_time


def main(args=None):
    rclpy.init(args=args)

    node = RevertBikePublisher()

    try:
        while (
            rclpy.ok()
            and not node.playback_finished
        ):
            rclpy.spin_once(
                node,
                timeout_sec=0.1,
            )

            node.log_due_stages()

        if node.published_at is not None:
            node.get_logger().info(
                'Bike revert trajectory completed.'
            )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
