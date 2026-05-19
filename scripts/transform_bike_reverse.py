#!/usr/bin/env python3
"""
transform_bike.py의 역재생 스크립트.
바이크 형태(Stage 6 최종 자세)에서 출발하여,
원래 변환 시퀀스를 정확히 역순으로 재생하여 기립 자세로 되돌립니다.

원본 순서: p1→p2→p3→p4→p5→p6
역재생 순서: p6→p5→p4→p3→p2→p1→직립(zero)
"""
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class TransformBikeReversePublisher(Node):
    def __init__(self):
        super().__init__('transform_bike_reverse_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        # 1초 뒤 퍼블리시 (publisher 준비 대기)
        self.timer = self.create_timer(1.0, self.publish_reverse)

    def publish_reverse(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
            'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
            'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
            'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
            'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
            'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt'
        ]

        # =====================================================================
        # 원본 transform_bike.py 스테이지를 역순으로 나열
        # (현재 바이크 자세 = 원본 Stage 6 에서 시작한다고 가정)
        # =====================================================================

        # --- Reverse Stage 1: 원본 Stage 6 → Stage 5 ---
        # 팔 당기기 해제 (arm_shoulder: 0.26 → -0.10)
        r1 = JointTrajectoryPoint()
        r1.positions = [
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        r1.time_from_start = Duration(sec=3, nanosec=0)

        # --- Reverse Stage 2: 원본 Stage 5 → Stage 4 ---
        # 발목 복원 (ankle: -1.57 → 1.3)
        r2 = JointTrajectoryPoint()
        r2.positions = [
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        r2.time_from_start = Duration(sec=6, nanosec=0)

        # --- Reverse Stage 3: 원본 Stage 4 → Stage 3 ---
        # 골반 앞으로 숙이기 (hip_pitch: 0→-1.8/1.8, knee: 0→-2.3)
        r3 = JointTrajectoryPoint()
        r3.positions = [
            0.0, 0.0, -1.8, -2.3, 1.3, 0.0,
            0.0, 0.0, 1.8, -2.3, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        r3.time_from_start = Duration(sec=9, nanosec=0)

        # --- Reverse Stage 4: 원본 Stage 3 → Stage 2 ---
        # 골반 복원 (hip_pitch: -1.8→-1.3 / 1.8→1.3)
        r4 = JointTrajectoryPoint()
        r4.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        r4.time_from_start = Duration(sec=12, nanosec=0)

        # --- Reverse Stage 5: 원본 Stage 2 → Stage 1 ---
        # 팔 원위치 (arm_base_yaw: 3.14→0, arm_shoulder: -0.10→0)
        r5 = JointTrajectoryPoint()
        r5.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        r5.time_from_start = Duration(sec=15, nanosec=0)

        # --- Reverse Stage 6: 원본 Stage 1 → 직립 ---
        # 무릎/골반 모두 펴서 직립 자세
        r6 = JointTrajectoryPoint()
        r6.positions = [
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        r6.time_from_start = Duration(sec=18, nanosec=0)

        msg.points = [r1, r2, r3, r4, r5, r6]
        self.publisher_.publish(msg)
        self.get_logger().info(
            'Published Reverse Playback of Bike Transformation (6 Stages, reversed)!'
        )
        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = TransformBikeReversePublisher()
    rclpy.spin_once(node, timeout_sec=20.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
