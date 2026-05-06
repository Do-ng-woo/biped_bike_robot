#!/usr/bin/env python3
"""
OP3 Walking Engine — Full Port for biped_bike_robot ver3

ROBOTIS-OP3 `op3_walking_module` (1151줄 C++)을 Python으로 완전 이식.
하체 12 DOF만 사용, 상체는 0 고정 (무게중심 수직 유지).

Reference: op3_walking_module.cpp by Kayman (ROBOTIS CO., LTD.)
Ported by: Antigravity Agent for dongwoo's biped_bike_robot

사용법:
    python3 scripts/op3_walker.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import numpy as np
import math
import dataclasses
from typing import Optional


# ============================================================
# Walking Parameters (OP3 defaults, adapted for biped_bike)
# ============================================================
@dataclasses.dataclass
class WalkingParam:
    """OP3 walking_param_ 구조체의 Python 완전 이식"""
    # --- Init Pose Offsets (meters, radians) ---
    init_x_offset: float = -0.010       # 골반 전후 (forward = negative X in our frame)
    init_y_offset: float = 0.005        # 골반 좌우
    init_z_offset: float = 0.020        # 골반 높이
    init_roll_offset: float = 0.0
    init_pitch_offset: float = 0.0
    init_yaw_offset: float = 0.0
    hip_pitch_offset: float = math.radians(13.0)  # 고관절 전방 기울임

    # --- Timing ---
    period_time: float = 0.6            # 한 주기 (초) — OP3 default 600ms
    dsp_ratio: float = 0.3             # Double Support Phase 비율 (무게 이동을 위한 여유 시간 확보)
    step_fb_ratio: float = 0.28        # Forward/Back swap 비율

    # --- Walking Amplitudes ---
    x_move_amplitude: float = 0.0      # 전후 보폭 (m) — 실행 시 설정
    y_move_amplitude: float = 0.0      # 좌우 이동 (m)
    z_move_amplitude: float = 0.020    # 발 높이 (m) — OP3=0.040, 소형이므로 축소
    angle_move_amplitude: float = 0.0  # 회전 (rad)
    move_aim_on: bool = False

    # --- Balance ---
    balance_enable: bool = False
    balance_hip_roll_gain: float = 0.5
    balance_knee_gain: float = 0.3
    balance_ankle_roll_gain: float = 1.0
    balance_ankle_pitch_gain: float = 0.9

    # --- Swing ---
    y_swap_amplitude: float = 0.015    # 좌우 흔들림 (m) — OP3=0.020
    z_swap_amplitude: float = 0.005    # 상하 흔들림 (m)

    # --- Pelvis ---
    pelvis_offset: float = math.radians(3.0)  # 골반 롤 오프셋
    arm_swing_gain: float = 1.5


# ============================================================
# Robot Dimensions (from ver3 URDF)
# ============================================================
THIGH_LENGTH = 0.059      # hip_pitch → knee_pitch
CALF_LENGTH  = 0.08775    # knee_pitch → ankle_pitch
ANKLE_LENGTH = 0.0        # ankle_pitch → foot_roll Z오프셋이 거의 0이므로 0 
LEG_LENGTH   = THIGH_LENGTH + CALF_LENGTH  # ~0.14675m (이 길이를 넘어가면 무릎이 안 구부러짐!)

# Hip pitch offset (small geometric offset in OP3's kinematic model)
HIP_PITCH_OFFSET_M = 0.0  # biped_bike는 hip pitch가 순수 Z축 회전이므로 0

# OP3 joint order for IK output: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
# ver3 joint axis directions
# Invert roll joints (indices 1, 5, 7, 11) to fix incorrect foot tilting in analytical IK
JOINT_AXIS_DIR = np.array([
    -1,  1, -1, -1, -1,  1,   # Right leg: Roll inverted to +1
    -1,  1,  1, -1, -1,  1,   # Left leg: Roll inverted to +1
], dtype=float)


# ============================================================
# Rotation / Transformation helpers (OP3 robotis_math port)
# ============================================================
def rotation_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def rotation_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])

def rotation_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

def rpy_to_rotation(roll, pitch, yaw):
    """RPY → 3x3 회전행렬 (ZYX 순서, OP3 기준)"""
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)

def get_sign(x):
    return 1.0 if x >= 0 else -1.0


# ============================================================
# Analytical 6-DOF Leg IK (calcInverseKinematicsForLeg 완전 이식)
# ============================================================
def analytical_ik_leg(x, y, z, roll, pitch, yaw,
                      thigh_l=THIGH_LENGTH, calf_l=CALF_LENGTH,
                      ankle_l=0.0, hip_offset_m=HIP_PITCH_OFFSET_M):
    """
    OP3 calcInverseKinematicsForLeg 완전 이식.
    
    Input: foot endpoint (x, y, z, roll, pitch, yaw) relative to hip
    Output: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll] (6 angles)
    
    Returns None if IK fails.
    """
    out = np.zeros(6)
    
    R06 = rpy_to_rotation(roll, pitch, yaw)
    p06 = np.array([x, y, z]) + ankle_l * R06[:, 2]  # desired hip to ankle
    
    # calc q6 (ankle roll)
    p60 = -R06.T @ p06
    out[5] = math.atan2(p60[1], p60[2])
    
    # calc q1 (hip yaw)
    R05 = R06 @ rotation_x(-out[5])
    out[0] = math.atan2(-R05[0, 1], R05[1, 1])
    
    # calc q4 (knee)
    p03 = rotation_z(out[0]) @ np.array([hip_offset_m, 0, 0])
    p36 = p06 - p03
    
    cos_val = (thigh_l**2 + calf_l**2 - p36.dot(p36)) / (2 * thigh_l * calf_l)
    cos_val = np.clip(cos_val, -1.0, 1.0)
    out[3] = -math.acos(cos_val) + math.pi
    
    # calc q5 (ankle pitch)
    sin_val = thigh_l * math.sin(math.pi - out[3]) / np.linalg.norm(p36)
    sin_val = np.clip(sin_val, -1.0, 1.0)
    alpha = math.asin(sin_val)
    p63 = -R06.T @ p36
    out[4] = -math.atan2(p63[0], get_sign(p63[2]) * math.sqrt(p63[1]**2 + p63[2]**2)) - alpha
    
    # calc q2 (hip roll) and q3 (hip pitch)
    R13 = rotation_z(-out[0]) @ R05 @ rotation_y(-(out[4] + out[3]))
    out[1] = math.atan2(R13[2, 1], R13[1, 1])
    out[2] = math.atan2(R13[0, 2], R13[0, 0])
    
    # hip pitch offset correction (OP3 specific geometric offset)
    hip_offset_angle = math.atan2(hip_offset_m, thigh_l + calf_l) if hip_offset_m != 0 else 0.0
    out[2] += hip_offset_angle
    out[3] -= hip_offset_angle
    
    return out


def ik_for_right_leg(x, y, z, roll, pitch, yaw):
    """Right leg IK with axis direction applied"""
    out = analytical_ik_leg(x, y, z, roll, pitch, yaw)
    if out is None:
        return None
    for i in range(6):
        out[i] *= JOINT_AXIS_DIR[i]  # R_leg: indices 0-5
    return out


def ik_for_left_leg(x, y, z, roll, pitch, yaw):
    """Left leg IK with axis direction applied"""
    out = analytical_ik_leg(x, y, z, roll, pitch, yaw)
    if out is None:
        return None
    for i in range(6):
        out[i] *= JOINT_AXIS_DIR[6 + i]  # L_leg: indices 6-11
    return out


# ============================================================
# OP3 Walking Engine (op3_walking_module 핵심 로직 완전 이식)
# ============================================================
class OP3WalkingEngine:
    """
    OP3 WalkingModule의 보행 생성 로직만 순수 Python으로 이식.
    ROS/하드웨어 종속성 없이 관절 각도만 계산.
    """
    
    PHASE0 = 0  # DSP start
    PHASE1 = 1  # Left SSP (left foot highest)
    PHASE2 = 2  # DSP middle
    PHASE3 = 3  # Right SSP (right foot highest)
    
    def __init__(self, param: Optional[WalkingParam] = None):
        self.param = param or WalkingParam()
        
        # Phase shift constants (OP3 L137~146)
        self.x_swap_phase_shift = math.pi
        self.x_swap_amplitude_shift = 0.0
        self.x_move_phase_shift = math.pi / 2
        self.x_move_amplitude_shift = 0.0
        self.y_swap_phase_shift = 0.0
        self.y_swap_amplitude_shift = 0.0
        self.y_move_phase_shift = math.pi / 2
        self.z_swap_phase_shift = math.pi * 3 / 2
        self.z_move_phase_shift = math.pi / 2
        self.a_move_phase_shift = math.pi / 2
        
        # Timing variables
        self.period_time = 0.0
        self.dsp_ratio = 0.0
        self.ssp_ratio = 0.0
        self.x_swap_period_time = 0.0
        self.x_move_period_time = 0.0
        self.y_swap_period_time = 0.0
        self.y_move_period_time = 0.0
        self.z_swap_period_time = 0.0
        self.z_move_period_time = 0.0
        self.a_move_period_time = 0.0
        self.ssp_time = 0.0
        self.l_ssp_start_time = 0.0
        self.l_ssp_end_time = 0.0
        self.r_ssp_start_time = 0.0
        self.r_ssp_end_time = 0.0
        self.phase1_time = 0.0
        self.phase2_time = 0.0
        self.phase3_time = 0.0
        
        # Movement amplitudes
        self.x_swap_amplitude = 0.0
        self.x_move_amplitude = 0.0
        self.y_swap_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.y_move_amplitude_shift = 0.0
        self.z_swap_amplitude = 0.0
        self.z_swap_amplitude_shift = 0.0
        self.z_move_amplitude = 0.0
        self.z_move_amplitude_shift = 0.0
        self.a_move_amplitude = 0.0
        self.a_move_amplitude_shift = 0.0
        
        # Pose offsets
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.r_offset = 0.0
        self.p_offset = 0.0
        self.a_offset = 0.0
        self.hit_pitch_offset = 0.0
        
        # Pelvis
        self.pelvis_offset = 0.0
        self.pelvis_swing = 0.0
        self.arm_swing_gain = 0.0
        
        # State
        self.time = 0.0
        self.phase = self.PHASE0
        self.ctrl_running = False
        self.real_running = False
        self.previous_x_move_amplitude = 0.0
        
        # Body swing (output)
        self.body_swing_y = 0.0
        self.body_swing_z = 0.0
        
        # Init position for each joint (12 leg joints) — OP3 L158-160 
        # [r_yaw, r_roll, r_pitch, r_knee, r_ank_pitch, r_ank_roll,
        #  l_yaw, l_roll, l_pitch, l_knee, l_ank_pitch, l_ank_roll]
        self.init_position = np.zeros(12)
        
        # Initialize
        self._update_time_param()
        self._update_movement_param()
    
    # ----------------------------------------------------------
    # wSin: OP3의 핵심 사인파 생성기 (L244-247)
    # ----------------------------------------------------------
    def _wsin(self, time, period, period_shift, mag, mag_shift):
        """mag * sin(2π/period * time - period_shift) + mag_shift"""
        return mag * math.sin(2 * math.pi / period * time - period_shift) + mag_shift
    
    # ----------------------------------------------------------
    # updateTimeParam (L334-361)
    # ----------------------------------------------------------
    def _update_time_param(self, scale=1.0):
        p = self.param
        self.period_time = scale * p.period_time
        self.dsp_ratio = p.dsp_ratio
        self.ssp_ratio = 1.0 - self.dsp_ratio
        
        self.x_swap_period_time = self.period_time / 2
        self.x_move_period_time = self.period_time * self.ssp_ratio
        self.y_swap_period_time = self.period_time
        self.y_move_period_time = self.period_time * self.ssp_ratio
        self.z_swap_period_time = self.period_time / 2
        
        # SSP 동안 Z축 리프트가 [이륙 -> 최고점 -> 착지] 1회 완벽한 포물선을 그리도록 반주기(/ 2)로 설정되어야 함
        self.z_move_period_time = self.period_time * self.ssp_ratio / 2
        
        self.a_move_period_time = self.period_time * self.ssp_ratio
        
        self.ssp_time = self.period_time * self.ssp_ratio
        self.l_ssp_start_time = (1 - self.ssp_ratio) * self.period_time / 4
        self.l_ssp_end_time = (1 + self.ssp_ratio) * self.period_time / 4
        self.r_ssp_start_time = (3 - self.ssp_ratio) * self.period_time / 4
        self.r_ssp_end_time = (3 + self.ssp_ratio) * self.period_time / 4
        
        self.phase1_time = (self.l_ssp_start_time + self.l_ssp_end_time) / 2
        self.phase2_time = (self.l_ssp_end_time + self.r_ssp_start_time) / 2
        self.phase3_time = (self.r_ssp_start_time + self.r_ssp_end_time) / 2
        
        self.pelvis_offset = p.pelvis_offset
        self.pelvis_swing = self.pelvis_offset * 0.35
        self.arm_swing_gain = p.arm_swing_gain
    
    # ----------------------------------------------------------
    # updateMovementParam (L363-405)
    # ----------------------------------------------------------
    def _update_movement_param(self):
        p = self.param
        
        # Forward/Back
        self.x_move_amplitude = p.x_move_amplitude
        self.x_swap_amplitude = p.x_move_amplitude * p.step_fb_ratio
        
        if self.previous_x_move_amplitude == 0:
            self.x_move_amplitude *= 0.5
            self.x_swap_amplitude *= 0.5
        
        # Right/Left
        self.y_move_amplitude = p.y_move_amplitude / 2
        if self.y_move_amplitude > 0:
            self.y_move_amplitude_shift = self.y_move_amplitude
        else:
            self.y_move_amplitude_shift = -self.y_move_amplitude
        self.y_swap_amplitude = p.y_swap_amplitude + self.y_move_amplitude_shift * 0.04
        
        self.z_move_amplitude = p.z_move_amplitude / 2
        self.z_move_amplitude_shift = p.z_move_amplitude / 2  # FIXED: use original p.z_move_amplitude
        self.z_swap_amplitude = p.z_swap_amplitude
        self.z_swap_amplitude_shift = p.z_swap_amplitude
        
        # Direction
        if not p.move_aim_on:
            self.a_move_amplitude = p.angle_move_amplitude / 2
            if self.a_move_amplitude > 0:
                self.a_move_amplitude_shift = self.a_move_amplitude
            else:
                self.a_move_amplitude_shift = -self.a_move_amplitude
        else:
            self.a_move_amplitude = -p.angle_move_amplitude / 2
            if self.a_move_amplitude > 0:
                self.a_move_amplitude_shift = -self.a_move_amplitude
            else:
                self.a_move_amplitude_shift = self.a_move_amplitude
    
    # ----------------------------------------------------------
    # updatePoseParam (L407-416)
    # ----------------------------------------------------------
    def _update_pose_param(self):
        p = self.param
        self.x_offset = p.init_x_offset
        self.y_offset = p.init_y_offset
        self.z_offset = p.init_z_offset
        self.r_offset = p.init_roll_offset
        self.p_offset = p.init_pitch_offset
        self.a_offset = p.init_yaw_offset
        self.hit_pitch_offset = p.hip_pitch_offset
    
    # ----------------------------------------------------------
    # processPhase (L672-733)
    # ----------------------------------------------------------
    def _process_phase(self, time_unit):
        if self.time == 0:
            self._update_time_param()
            self.phase = self.PHASE0
            # FIX: 유저 파라미터(x_move_amplitude 등)를 건드리지 않음!
            #      init pose 계산 시 ctrl_running=False라도 파라미터 보존
            if not self.ctrl_running:
                self.real_running = False
        
        elif (self.time >= self.phase1_time - time_unit / 2 and 
              self.time < self.phase1_time + time_unit / 2):
            self._update_movement_param()
            self._update_time_param()
            self.time = self.phase1_time
            self.phase = self.PHASE1
        
        elif (self.time >= self.phase2_time - time_unit / 2 and 
              self.time < self.phase2_time + time_unit / 2):
            self._update_time_param()
            self.time = self.phase2_time
            self.phase = self.PHASE2
            if not self.ctrl_running:
                self.real_running = False
        
        elif (self.time >= self.phase3_time - time_unit / 2 and 
              self.time < self.phase3_time + time_unit / 2):
            self._update_movement_param()
            self._update_time_param()
            self.time = self.phase3_time
            self.phase = self.PHASE3
    
    # ----------------------------------------------------------
    # computeLegAngle (L735-982) — 핵심! 4구간 사인파 + IK
    # ----------------------------------------------------------
    def _compute_leg_angles(self):
        """
        시간에 따른 발 끝점(6D)을 4구간 사인파로 계산하고,
        해석적 IK로 12개 다리 관절 각도를 생성.
        
        Returns: leg_angle[12] or None on IK failure
        """
        self._update_pose_param()
        
        # --- 4구간 사인파로 발 끝점 계산 ---
        # swap: 양 발 공통 진동
        swap_x = self._wsin(self.time, self.x_swap_period_time,
                            self.x_swap_phase_shift, self.x_swap_amplitude,
                            self.x_swap_amplitude_shift)
        swap_y = self._wsin(self.time, self.y_swap_period_time,
                            self.y_swap_phase_shift, self.y_swap_amplitude,
                            self.y_swap_amplitude_shift)
        swap_z = self._wsin(self.time, self.z_swap_period_time,
                            self.z_swap_phase_shift, self.z_swap_amplitude,
                            self.z_swap_amplitude_shift)
        
        pelvis_offset_l = 0.0
        pelvis_offset_r = 0.0
        
        t = self.time
        
        if t <= self.l_ssp_start_time:
            # DSP: before left SSP
            left_x  = self._wsin(self.l_ssp_start_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            left_y  = self._wsin(self.l_ssp_start_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            left_z  = self._wsin(self.l_ssp_start_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            left_yaw = self._wsin(self.l_ssp_start_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(self.l_ssp_start_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(self.l_ssp_start_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = self._wsin(self.r_ssp_start_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.r_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            right_yaw = self._wsin(self.l_ssp_start_time, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                   -self.a_move_amplitude, -self.a_move_amplitude_shift)
        
        elif t <= self.l_ssp_end_time:
            # Left SSP (left foot swinging)
            left_x  = self._wsin(t, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            left_y  = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            
            left_z  = self._wsin(t, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            
            left_yaw = self._wsin(t, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(t, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = self._wsin(self.r_ssp_start_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.r_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            right_yaw = self._wsin(t, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                   -self.a_move_amplitude, -self.a_move_amplitude_shift)
            pelvis_offset_l = self._wsin(t, self.z_move_period_time,
                                         self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                         self.pelvis_swing/2, self.pelvis_swing/2)
            pelvis_offset_r = self._wsin(t, self.z_move_period_time,
                                         self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                         -self.pelvis_offset/2, -self.pelvis_offset/2)
        
        elif t <= self.r_ssp_start_time:
            # DSP: between left and right SSP
            left_x  = self._wsin(self.l_ssp_end_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            left_y  = self._wsin(self.l_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            left_z  = self._wsin(self.l_ssp_end_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            left_yaw = self._wsin(self.l_ssp_end_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(self.l_ssp_end_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(self.l_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = self._wsin(self.r_ssp_start_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.r_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            right_yaw = self._wsin(self.l_ssp_end_time, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                   -self.a_move_amplitude, -self.a_move_amplitude_shift)
        
        elif t <= self.r_ssp_end_time:
            # Right SSP (right foot swinging)
            phase_r = self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.r_ssp_start_time + math.pi
            
            # 🚨 FIX: In Phase 3, Left foot is supporting! It must use NEGATIVE amplitudes to swing backward!
            left_x  = self._wsin(t, self.x_move_period_time, phase_r,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            left_y  = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            left_z  = self._wsin(self.l_ssp_end_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            left_yaw = self._wsin(t, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                  -self.a_move_amplitude, -self.a_move_amplitude_shift)
                                  
            right_x = self._wsin(t, self.x_move_period_time, phase_r,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            right_y = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            
            right_z = self._wsin(t, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.r_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            
            right_yaw = self._wsin(t, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                   self.a_move_amplitude, self.a_move_amplitude_shift)
                                   
            # OP3 explicitly sets these to 0 in Phase 3
            pelvis_offset_l = 0.0
            pelvis_offset_r = 0.0
        
        else:
            # DSP: after right SSP
            phase_r = self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.r_ssp_start_time + math.pi
            left_x  = self._wsin(self.r_ssp_end_time, self.x_move_period_time, phase_r,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            left_y  = self._wsin(self.r_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            left_z  = self._wsin(self.l_ssp_end_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.l_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            left_yaw = self._wsin(self.r_ssp_end_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                  -self.a_move_amplitude, -self.a_move_amplitude_shift)
                                  
            right_x = self._wsin(self.r_ssp_end_time, self.x_move_period_time, phase_r,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            right_y = self._wsin(self.r_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            right_z = self._wsin(self.r_ssp_end_time, self.z_move_period_time,
                                 self.z_move_phase_shift + 2*math.pi/self.z_move_period_time*self.r_ssp_start_time,
                                 self.z_move_amplitude, self.z_move_amplitude_shift)
            right_yaw = self._wsin(self.r_ssp_end_time, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                   self.a_move_amplitude, self.a_move_amplitude_shift)
        
        # --- Compute foot endpoints (OP3 L917-929) ---
        ep = np.zeros(12)  # [rx, ry, rz, rroll, rpitch, ryaw, lx, ly, lz, lroll, lpitch, lyaw]
        
        ep[0]  = swap_x + right_x + self.x_offset
        ep[1]  = swap_y + right_y - self.y_offset / 2
        ep[2]  = swap_z + right_z + self.z_offset - LEG_LENGTH
        ep[3]  = 0.0 - self.r_offset / 2      # right roll
        ep[4]  = 0.0 + self.p_offset           # right pitch
        ep[5]  = 0.0 + right_yaw - self.a_offset / 2  # right yaw
        
        ep[6]  = swap_x + left_x + self.x_offset
        ep[7]  = swap_y + left_y + self.y_offset / 2
        ep[8]  = swap_z + left_z + self.z_offset - LEG_LENGTH
        ep[9]  = 0.0 + self.r_offset / 2       # left roll
        ep[10] = 0.0 + self.p_offset            # left pitch
        ep[11] = 0.0 + left_yaw + self.a_offset / 2  # left yaw
        
        # 골반은 항상 바닥과 평행 유지!
        # 무게중심 이동은 y_swap(평행사변형 병진)으로만 처리.
        
        # --- Body swing ---
        if t <= self.l_ssp_end_time:
            self.body_swing_y = -ep[7]
            self.body_swing_z = ep[8]
        else:
            self.body_swing_y = -ep[1]
            self.body_swing_z = ep[2]
        self.body_swing_z -= LEG_LENGTH
        
        # --- Solve IK ---
        right_angles = ik_for_right_leg(ep[0], ep[1], ep[2], ep[3], ep[4], ep[5])
        if right_angles is None:
            return None
        
        left_angles = ik_for_left_leg(ep[6], ep[7], ep[8], ep[9], ep[10], ep[11])
        if left_angles is None:
            return None
        
        leg_angle = np.concatenate([right_angles, left_angles])
        
        # --- Apply hip offsets (OP3 L963-979) ---
        # hip_roll pelvis offset
        leg_angle[1] += JOINT_AXIS_DIR[1] * pelvis_offset_r    # r_hip_roll
        leg_angle[7] += JOINT_AXIS_DIR[7] * pelvis_offset_l    # l_hip_roll
        
        # hip_pitch offset (forward lean)
        leg_angle[2] -= JOINT_AXIS_DIR[2] * self.hit_pitch_offset   # r_hip_pitch
        leg_angle[8] -= JOINT_AXIS_DIR[8] * self.hit_pitch_offset   # l_hip_pitch
        
        return leg_angle
    
    # ----------------------------------------------------------
    # step(): 한 타임스텝 실행 → 17개 관절 각도 반환
    # ----------------------------------------------------------
    def step(self, time_unit):
        """
        한 타임스텝(time_unit 초) 실행.
        Returns: 17개 관절 각도 (12 legs + 5 arms=0) or None
        """
        self._process_phase(time_unit)
        
        leg_angles = self._compute_leg_angles()
        if leg_angles is None:
            return None
        
        # 최종: init_position + computed angles
        final_leg = self.init_position + leg_angles
        
        # 17 DOF output: 12 legs (reorder to ver3) + 5 arms (fixed at 0)
        # OP3 order: [r_yaw, r_roll, r_pitch, r_knee, r_ank_pitch, r_ank_roll, 
        #             l_yaw, l_roll, l_pitch, l_knee, l_ank_pitch, l_ank_roll]
        # ver3 order: [l_yaw, l_roll, l_pitch, l_knee, l_ankle, l_foot, 
        #              r_yaw, r_roll, r_pitch, r_knee, r_ankle, r_foot,
        #              arm_yaw, arm_shoulder, arm_elbow, arm_wrist_p, arm_wrist_r]
        output = np.zeros(17)
        # Left leg (OP3 indices 6-11 → ver3 indices 0-5)
        output[0:6] = final_leg[6:12]
        # Right leg (OP3 indices 0-5 → ver3 indices 6-11)
        output[6:12] = final_leg[0:6]
        # Arms: all zero (상체 무게중심 수직 유지)
        output[12:17] = 0.0
        
        # Advance time
        if self.real_running:
            self.time += time_unit
            if self.time >= self.period_time:
                self.time = 0
                # FIX: 실제 계산된 amplitude를 기억 (부드러운 연속 보행)
                self.previous_x_move_amplitude = self.x_move_amplitude
        
        return output
    
    # ----------------------------------------------------------
    # start / stop (OP3 L418-434)
    # ----------------------------------------------------------
    def start(self):
        self.ctrl_running = True
        self.real_running = True
    
    def stop(self):
        self.ctrl_running = False
    
    def is_running(self):
        return self.real_running


# ============================================================
# ROS 2 Node
# ============================================================
class OP3WalkerNode(Node):
    def __init__(self):
        super().__init__('op3_walker')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        # Walking parameters
        # NOTE: 이 로봇의 전진 방향은 -X (OP3는 +X)
        #       따라서 x_move_amplitude를 음수로 설정!
        #
        # 🆕 이 로봇의 힙 간격 = 12.5cm (URDF: L=-0.01425, R=-0.13925)
        #    OP3(힙간격 ~7cm) 대비 1.8배 넓으므로 y_swap만으로 체중 이동 불가.
        #    com_tilt_angle로 발목/골반 롤을 기울여 CoM을 지지발 위로 이동시킴.
        param = WalkingParam()
        param.init_x_offset = -0.010     # 골반 약간 전방 (전도 방지)
        param.init_z_offset = 0.020      # 무릎 살짝 굽혀 충격 흡수
        param.period_time = 2.0          # 느리고 안정적인 보행
        param.dsp_ratio = 0.3            # DSP 비율 (OP3 기본값)
        param.x_move_amplitude = -0.030  # 보폭 3cm 전진
        param.z_move_amplitude = 0.040   # 발 높이 4cm (확실한 리프트)
        # 평행사변형 무게중심 이동: 이 로봇은 OP3 대비 Y축이 반전됨 → 음수 필요!
        # 힙 간격 12.5cm, 지지발 위로 CoM 이동에 충분한 병진량
        param.y_swap_amplitude = -0.040  # 평행사변형 병진 (음수=지지발 방향, 골반 수평 유지)
        param.pelvis_offset = math.radians(5.0)  # 골반 롤 오프셋 (SSP 안정성)
        
        self.engine = OP3WalkingEngine(param)
        
        self.joint_names = [
            'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
            'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
            'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
            'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
            'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
            'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt',
        ]
        
        self.control_cycle = 0.008  # 8ms (125Hz) — OP3 default
        self.num_cycles = 1         # 1 walking cycle (요청에 따라 1보만)
        
        self.timer = self.create_timer(2.0, self.generate_trajectory)
    
    def generate_trajectory(self):
        self.timer.cancel()
        self.get_logger().info('Generating OP3-style walking trajectory...')
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # 1) Init pose (go to ready in 2s)
        self.engine._update_time_param()
        self.engine._update_movement_param()
        init_angles = self.engine.step(self.control_cycle)
        if init_angles is not None:
            p0 = JointTrajectoryPoint()
            p0.positions = init_angles.tolist()
            p0.time_from_start = Duration(sec=2, nanosec=0)
            msg.points.append(p0)
        
        # 2) Start walking
        self.engine.time = 0.0
        self.engine.start()
        
        total_time = self.engine.param.period_time * self.num_cycles
        t = 0.0
        base_time = 3.0  # start at 3s
        point_count = 0
        
        while t < total_time:
            angles = self.engine.step(self.control_cycle)
            if angles is not None:
                point = JointTrajectoryPoint()
                point.positions = angles.tolist()
                abs_t = base_time + t
                point.time_from_start = Duration(
                    sec=int(abs_t),
                    nanosec=int((abs_t % 1) * 1e9)
                )
                msg.points.append(point)
                point_count += 1
            
            t += self.control_cycle
            
            if point_count % 200 == 0:
                self.get_logger().info(f'  Generated {point_count} points ({t:.1f}/{total_time:.1f}s)')
        
        # 3) Stop walking → return to init
        self.engine.stop()
        # Run a few more cycles to decelerate
        for _ in range(int(self.engine.param.period_time * 2 / self.control_cycle)):
            angles = self.engine.step(self.control_cycle)
            if angles is not None:
                point = JointTrajectoryPoint()
                point.positions = angles.tolist()
                abs_t = base_time + t
                point.time_from_start = Duration(
                    sec=int(abs_t),
                    nanosec=int((abs_t % 1) * 1e9)
                )
                msg.points.append(point)
                point_count += 1
            t += self.control_cycle
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {point_count} trajectory points! ({t:.1f}s total)')


def main(args=None):
    rclpy.init(args=args)
    node = OP3WalkerNode()
    rclpy.spin_once(node, timeout_sec=30.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
