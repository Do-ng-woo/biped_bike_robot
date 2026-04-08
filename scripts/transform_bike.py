#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class TransformBikePublisher(Node):
    def __init__(self):
        super().__init__('transform_bike_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        # Use a short timer to ensure the publisher is ready before sending
        self.timer = self.create_timer(1.0, self.publish_transform)
        
    def publish_transform(self):
        msg = JointTrajectory()
        msg.joint_names = [
            'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
            'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
            'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
            'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
            'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
            'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt'
        ]
        
        # --- Stage 1: 무릎 꿇기 (Deeper Squat) ---
        p1 = JointTrajectoryPoint()
        p1.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        p1.time_from_start = Duration(sec=3, nanosec=0)
        
        # --- Stage 2: 팔 위치 세팅 (Arm Setup -5 deg) ---
        p2 = JointTrajectoryPoint()
        p2.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        p2.time_from_start = Duration(sec=6, nanosec=0)

        # --- Stage 3: 골반 넘어지기 (Torso Bow Forward: -1.8) ---
        p3 = JointTrajectoryPoint()
        p3.positions = [
            0.0, 0.0, -1.8, -2.3, 1.3, 0.0, 
            0.0, 0.0, 1.8, -2.3, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        p3.time_from_start = Duration(sec=9, nanosec=0)

        # --- Stage 4: 무릎 & 골반 동시 펴기 (Knee & Hip Straighten Simultaneously) ---
        p4 = JointTrajectoryPoint()
        p4.positions = [
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0, 
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        p4.time_from_start = Duration(sec=12, nanosec=0)

        # --- Stage 5: 발목 넘기기 (Lift Feet: Ankle -1.57) ---
        p5 = JointTrajectoryPoint()
        p5.positions = [
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
            3.14159, -0.10, 0.0, 0.0, 0.0
        ]
        p5.time_from_start = Duration(sec=15, nanosec=0)

        # --- Stage 6: 더욱 낮게 엎드리기 (Arm Pull to 15 deg: 0.26) ---
        p6 = JointTrajectoryPoint()
        p6.positions = [
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
            3.14159, 0.26, 0.0, 0.0, 0.0
        ]
        p6.time_from_start = Duration(sec=18, nanosec=0)
        
        msg.points = [p1, p2, p3, p4, p5, p6]
        self.publisher_.publish(msg)
        self.get_logger().info('Published Bike Transformation Command (6 Master Stages)!')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = TransformBikePublisher()
    rclpy.spin_once(node, timeout_sec=19.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
