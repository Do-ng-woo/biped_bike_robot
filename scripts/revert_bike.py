#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class RevertBikePublisher(Node):
    def __init__(self):
        super().__init__('revert_bike_publisher')
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
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
        
        # --- Stage 1: 팔 꿈치 강력히 밀어내기 (-40도 / -0.70 rad) ---
        # 1, 2번 통합 취소: 먼저 확실하게 팔만 -40도(-0.70 rad)로 밀어 올립니다!
        # 엄청난 토크가 필요하므로 모터가 풀리지 않게 이 지점부터 계속 명령을 때려박습니다.
        p1 = JointTrajectoryPoint()
        p1.positions = [
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0, 
            0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
            3.14159, -0.70, 0.0, 0.0, 0.0
        ]
        p1.time_from_start = Duration(sec=3, nanosec=0)

        # --- Stage 2: 발목 복구 (Ankle Reset) ---
        # 팔은 -40도를 절대 놓지 않고 계속(다시) 강제하며, 발목(1.3)을 원위치시킵니다.
        p2 = JointTrajectoryPoint()
        p2.positions = [
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0, 
            0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
            3.14159, -0.70, 0.0, 0.0, 0.0
        ]
        p2.time_from_start = Duration(sec=6, nanosec=0)
        
        # --- Stage 3: 골반 꺾기 & 무릎 접기 동시 실행 (Fold Chassis) ---
        # 언제 모터가 풀리며 털썩 주저앉을지 모르니, 여기서도 팔은 -40도를 악착같이 유지하며 상하체를 접습니다.
        p3 = JointTrajectoryPoint()
        p3.positions = [
            0.0, 0.0, -1.8, -2.3, 1.3, 0.0, 
            0.0, 0.0, 1.8, -2.3, 1.3, 0.0,
            3.14159, -0.70, 0.0, 0.0, 0.0
        ]
        p3.time_from_start = Duration(sec=9, nanosec=0)

        # --- Stage 4: 몸통 기상 (Lift to Deep Squat) ---
        # 딥 스쿼트 형태로 돌아와 무게중심이 발바닥으로 넘어갑니다. 
        # (드디어 팔이 바닥에서 떨어지더라도 팔은 -40도를 굳건히 유지합니다)
        p4 = JointTrajectoryPoint()
        p4.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0, 
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            3.14159, -0.70, 0.0, 0.0, 0.0
        ]
        p4.time_from_start = Duration(sec=12, nanosec=0)

        # --- Stage 5: 팔 원상복구 (Arms Normal) ---
        # 살인적인 지지 임무를 끝낸 팔을 0.0으로 복귀시킵니다.
        p5 = JointTrajectoryPoint()
        p5.positions = [
            0.0, 0.0, -1.3, -2.3, 1.3, 0.0, 
            0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        p5.time_from_start = Duration(sec=15, nanosec=0)

        # --- Stage 6: Stand Up (Ready Posture) ---
        # 부드럽게 일어납니다!
        p6 = JointTrajectoryPoint()
        p6.positions = [
            0.0, 0.0, -0.35, -0.70, 0.35, 0.0, 
            0.0, 0.0, 0.35, -0.70, 0.35, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0
        ]
        p6.time_from_start = Duration(sec=18, nanosec=0)
        
        msg.points = [p1, p2, p3, p4, p5, p6]
        self.publisher_.publish(msg)
        self.get_logger().info('Published Reverse Bike Transformation Command (Continuous -40 deg Grip)!')
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = RevertBikePublisher()
    rclpy.spin_once(node, timeout_sec=19.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
