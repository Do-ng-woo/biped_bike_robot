#!/usr/bin/env python3
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from bike_transform_sequence import JOINT_NAMES, TRANSFORM_SEQUENCE


class TransformBikePublisher(Node):
    def __init__(self):
        super().__init__('transform_bike_publisher')
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
        self.timer = self.create_timer(1.0, self.publish_transform)

    @staticmethod
    def _duration(seconds):
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        return Duration(sec=sec, nanosec=nanosec)

    def publish_transform(self):
        msg = JointTrajectory()
        msg.joint_names = list(JOINT_NAMES)

        for index, positions in enumerate(TRANSFORM_SEQUENCE, start=1):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = self._duration(index * self.stage_duration_sec)
            msg.points.append(point)

        self.publisher_.publish(msg)
        self.get_logger().info(
            'Published supported bike transformation: stable revert path reversed, '
            f'{len(msg.points)} stages x {self.stage_duration_sec:.1f}s'
        )
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TransformBikePublisher()
    playback_time = 1.5 + len(TRANSFORM_SEQUENCE) * node.stage_duration_sec
    rclpy.spin_once(node, timeout_sec=playback_time)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

