#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class ReadyPosturePublisher(Node):
    def __init__(self):
        super().__init__('ready_posture_publisher')
        self.declare_parameter('move_duration_sec', 3.0)
        self.declare_parameter('forward_lean_deg', 5.0)
        self.declare_parameter('arm_shoulder_pitch_deg', -70.0)
        self.move_duration_sec = max(
            0.1,
            float(self.get_parameter('move_duration_sec').value),
        )
        self.forward_lean = math.radians(
            float(self.get_parameter('forward_lean_deg').value)
        )
        self.arm_shoulder_pitch = math.radians(
            float(self.get_parameter('arm_shoulder_pitch_deg').value)
        )
        self.publisher_ = self.create_publisher(
            JointTrajectory, 
            '/joint_trajectory_controller/joint_trajectory', 
            10
        )
        # Use a short timer to ensure the publisher is ready before sending
        self.timer = self.create_timer(1.0, self.publish_posture)
        
    def publish_posture(self):
        msg = JointTrajectory()
        # ver3 관절 이름 — controllers.yaml과 일치
        msg.joint_names = [
            # 좌측 다리 (Left Leg) - 6 DOF
            'l_hip_yaw_jnt',
            'l_hip_roll_jnt',
            'l_hip_pitch_jnt',
            'l_knee_pitch_jnt',
            'l_ankle_pitch_jnt',
            'l_foot_roll_jnt',
            # 우측 다리 (Right Leg) - 6 DOF
            'r_hip_yaw_jnt',
            'r_hip_roll_jnt',
            'r_hip_pitch_jnt',
            'r_knee_pitch_jnt',
            'r_ankle_pitch_jnt',
            'r_foot_roll_jnt',
            # 상체 (Upper Body) - 5 DOF
            'arm_base_yaw_jnt',
            'arm_shoulder_pitch_jnt',
            'arm_elbow_pitch_jnt',
            'arm_wrist_pitch_jnt',
            'arm_wrist_roll_jnt',
        ]
        
        point = JointTrajectoryPoint()
        
        # [표준 동작] 무릎 굽히기 자세 (Ready Posture)
        # 로봇이 엉거주춤하게 안정적으로 서 있기 위해 무릎을 약 40도 (0.7 rad) 굽힘.
        # 발바닥이 땅에 수평으로 닿고, 허리가 꼿꼿이 서기 위해선 
        # 고관절(Hip)과 발목(Ankle)이 각각 절반 각도인 20도 (0.35 rad)를 역방향으로 보상해줘야 합니다.
        
        # Left leg: hip_pitch axis Z(+1), knee axis Z(-1), ankle axis Z(-1)
        l_hip_pitch = -0.35 - self.forward_lean  # Z(+1): 음수 → 앞으로 기울임
        l_knee = -0.70        # Z(-1): 음수 → 무릎 앞으로 굽힘
        l_ankle = 0.35        # Z(-1): 양수 → 발목 보상
        
        # Right leg: hip_pitch axis Z(-1), knee axis Z(-1), ankle axis Z(-1)
        r_hip_pitch = 0.35 + self.forward_lean  # Z(-1): 양수 → 앞으로 기울임
        r_knee = -0.70        # Z(-1): 음수 → 무릎 앞으로 굽힘
        r_ankle = 0.35        # Z(-1): 양수 → 발목 보상
        
        point.positions = [
            # Left leg: yaw, roll, pitch, knee, ankle, foot
            0.0, 0.0, l_hip_pitch, l_knee, l_ankle, 0.0,
            # Right leg: yaw, roll, pitch, knee, ankle, foot
            0.0, 0.0, r_hip_pitch, r_knee, r_ankle, 0.0,
            # Arm: base_yaw, shoulder, elbow, wrist_pitch, wrist_roll
            0.0, self.arm_shoulder_pitch, 0.0, 0.0, 0.0,
        ]
        
        # 브릿지가 현재 자세부터 이 목표까지 선형 보간합니다.
        sec = int(self.move_duration_sec)
        nanosec = int(round((self.move_duration_sec - sec) * 1e9))
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        
        msg.points.append(point)
        self.publisher_.publish(msg)
        self.get_logger().info(
            f'Published Ready Posture command ({self.move_duration_sec:.1f}s transition, '
            f'forward_lean={math.degrees(self.forward_lean):.1f}deg, '
            f'shoulder={math.degrees(self.arm_shoulder_pitch):.1f}deg)'
        )
        self.timer.cancel()  # 한 번 보내고 타이머 정지

def main(args=None):
    rclpy.init(args=args)
    node = ReadyPosturePublisher()
    rclpy.spin_once(node, timeout_sec=2.5)  # 2.5초 대기하면서 콜백 실행
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
