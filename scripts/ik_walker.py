#!/usr/bin/env python3
"""
Sinusoidal IK walker for biped_bike_robot.

Generates lower-body walking trajectories with analytical leg IK and publishes
the resulting 17-DOF JointTrajectory command. The arm joints are held in a
stable posture while the walking command is generated.

사용법:
    python3 scripts/ik_walker.py
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
# Walking Parameters
# ============================================================
@dataclasses.dataclass
class WalkingParam:
    """Robot-specific gait parameters used by both ROS and the pure IK engine."""
    # --- Init Pose Offsets (meters, radians) ---
    init_x_offset: float = -0.020       # 골반 전후 (forward = negative X in our frame)
    init_y_offset: float = 0.005        # 기본 IK 발 간격 오프셋
    init_z_offset: float = 0.025        # 골반 높이 보정. 클수록 다리를 더 접어 몸이 낮아짐
    init_roll_offset: float = 0.0
    init_pitch_offset: float = 0.0
    init_yaw_offset: float = 0.0
    hip_pitch_forward: float = math.radians(10.0)

    # --- Timing ---
    period_time: float = 2.0            # 실물 기준 한 주기 (초)
    dsp_ratio: float = 0.3             # Double Support Phase 비율 (무게 이동을 위한 여유 시간 확보)
    step_fb_ratio: float = 0.10         # Forward/Back swap 비율
    x_swap_forward_bias: float = 0.0    # 착지 시 골반이 발보다 앞서지 않도록 중앙 유지
    x_swap_time_advance_ratio: float = 0.0
    x_move_start_scale: float = 1.0     # 출발 시 보폭 비율
    x_move_ramp_per_cycle: float = 0.0  # 매 보행 주기마다 늘릴 보폭 비율

    # --- Walking Amplitudes ---
    x_move_amplitude: float = -0.020   # 실물 전진 보폭 (전진 = -X)
    y_move_amplitude: float = 0.0      # 좌우 이동 (m)
    z_move_amplitude: float = 0.050    # 실물 스윙발 최고 높이 (m)
    angle_move_amplitude: float = 0.0  # 회전 (rad)
    move_aim_on: bool = False

    # --- Balance ---
    balance_enable: bool = False
    balance_hip_roll_gain: float = 0.5
    balance_knee_gain: float = 0.3
    balance_ankle_roll_gain: float = 1.0
    balance_ankle_pitch_gain: float = 0.9

    # --- Swing ---
    y_swap_amplitude: float = -0.047   # 실물 힙 간격에 맞춘 좌우 CoM 이동 (m)
    z_swap_amplitude: float = 0.005    # 상하 흔들림 (m)
    roll_correction: float = math.radians(3.0)

    # --- Upper body ---
    arm_swing_gain: float = 1.5


# ============================================================
# Robot Dimensions (from the physical robot URDF)
# ============================================================
THIGH_LENGTH = 0.059      # hip_pitch → knee_pitch
CALF_LENGTH  = 0.1125     # knee_pitch → ankle_pitch
ANKLE_LENGTH = 0.0        # ankle_pitch → foot_roll Z오프셋이 거의 0이므로 0 
LEG_LENGTH   = THIGH_LENGTH + CALF_LENGTH  # 0.1715m
HIP_SPACING  = 0.125      # URDF left/right hip yaw origin 간격

# Hip pitch offset for the analytical leg model.
HIP_PITCH_OFFSET_M = 0.0  # biped_bike는 hip pitch가 순수 Z축 회전이므로 0

# IK output order: [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll]
# Robot joint axis directions
# 해석 IK의 roll 출력 방향을 로봇 관절 명령 방향에 맞춘다.
JOINT_AXIS_DIR = np.array([
    -1,  1, -1, -1, -1,  1,   # Right leg
    -1,  1,  1, -1, -1,  1,   # Left leg
], dtype=float)

# IK 내부 순서(right leg, left leg)에 적용하는 실물 백래시 보정 부호.
LEG_ROLL_CORRECTION_SIGNS = np.array([
    0, 1, 0, 0, 0, -1,   # Right: hip +, ankle -
    0, -1, 0, 0, 0, 1,   # Left: hip -, ankle +
], dtype=float)

# IK 내부 순서에서 같은 전방 기울기를 만드는 좌/우 hip pitch 부호.
LEG_HIP_PITCH_FORWARD_SIGNS = np.array([
    0, 0, 1, 0, 0, 0,    # Right hip pitch +
    0, 0, -1, 0, 0, 0,   # Left hip pitch -
], dtype=float)

# Published 17-DOF order: left hip/foot roll, right hip/foot roll.
ROLL_JOINT_INDICES = (1, 5, 7, 11)
PUBLISHED_ROLL_CORRECTION_SIGNS = np.array([-1, 1, 1, -1], dtype=float)


def without_roll_correction(angles, correction):
    """Remove only the additive hardware correction, preserving the IK roll."""
    normalized = np.array(angles, dtype=float, copy=True)
    normalized[list(ROLL_JOINT_INDICES)] -= (
        PUBLISHED_ROLL_CORRECTION_SIGNS * correction
    )
    return normalized


# ============================================================
# Rotation / Transformation helpers
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
    """RPY -> 3x3 rotation matrix in ZYX order."""
    return rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)

def get_sign(x):
    return 1.0 if x >= 0 else -1.0


# ============================================================
# Analytical 6-DOF Leg IK
# ============================================================
def analytical_ik_leg(x, y, z, roll, pitch, yaw,
                      thigh_l=THIGH_LENGTH, calf_l=CALF_LENGTH,
                      ankle_l=0.0, hip_offset_m=HIP_PITCH_OFFSET_M):
    """
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
    
    # hip pitch offset correction for models with a nonzero hip pitch offset
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
# IK Walker Engine
# ============================================================
class IKWalkerEngine:
    """
    Generates walking joint angles without depending on ROS or hardware IO.
    """
    
    PHASE0 = 0  # DSP start
    PHASE1 = 1  # Left SSP (left foot highest)
    PHASE2 = 2  # DSP middle
    PHASE3 = 3  # Right SSP (right foot highest)
    
    def __init__(self, param: Optional[WalkingParam] = None):
        self.param = param or WalkingParam()
        
        # Phase shift constants for the sinusoidal gait pattern
        self.x_swap_phase_shift = math.pi
        self.x_swap_amplitude_shift = 0.0
        self.x_move_phase_shift = math.pi / 2
        self.x_move_amplitude_shift = 0.0
        self.y_swap_phase_shift = 0.0
        self.y_swap_amplitude_shift = 0.0
        self.y_move_phase_shift = math.pi / 2
        self.z_swap_phase_shift = math.pi * 3 / 2
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
        self.x_swap_time_advance = 0.0
        self.x_move_amplitude = 0.0
        self.y_swap_amplitude = 0.0
        self.y_move_amplitude = 0.0
        self.y_move_amplitude_shift = 0.0
        self.z_swap_amplitude = 0.0
        self.z_swap_amplitude_shift = 0.0
        self.z_move_amplitude = 0.0
        self.roll_correction = 0.0
        self.hip_pitch_forward = 0.0
        self.a_move_amplitude = 0.0
        self.a_move_amplitude_shift = 0.0
        
        # Pose offsets
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.z_offset = 0.0
        self.r_offset = 0.0
        self.p_offset = 0.0
        self.a_offset = 0.0
        
        # Upper body
        self.arm_swing_gain = 0.0
        
        # State
        self.time = 0.0
        self.phase = self.PHASE0
        self.ctrl_running = False
        self.real_running = False
        self.previous_x_move_amplitude = 0.0
        self.x_move_scale = self.param.x_move_start_scale
        
        # Body swing (output)
        self.body_swing_y = 0.0
        self.body_swing_z = 0.0
        self.last_foot_endpoints = None
        
        # Init position for each leg joint.
        # [r_yaw, r_roll, r_pitch, r_knee, r_ank_pitch, r_ank_roll,
        #  l_yaw, l_roll, l_pitch, l_knee, l_ank_pitch, l_ank_roll]
        self.init_position = np.zeros(12)
        
        # Initialize
        self._update_time_param()
        self._update_movement_param()
    
    # ----------------------------------------------------------
    # Windowed sinusoid helper used by the gait phase generator.
    # ----------------------------------------------------------
    def _wsin(self, time, period, period_shift, mag, mag_shift):
        """mag * sin(2π/period * time - period_shift) + mag_shift"""
        return mag * math.sin(2 * math.pi / period * time - period_shift) + mag_shift

    def _ssp_lift_profile(self, time, start_time, end_time):
        """Smooth 0 -> 1 -> 0 lift with zero velocity at SSP boundaries."""
        if time <= start_time or time >= end_time:
            return 0.0
        phase = (time - start_time) / (end_time - start_time)
        return math.sin(math.pi * phase) ** 2

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
        
        self.a_move_period_time = self.period_time * self.ssp_ratio
        
        self.ssp_time = self.period_time * self.ssp_ratio
        self.l_ssp_start_time = (1 - self.ssp_ratio) * self.period_time / 4
        self.l_ssp_end_time = (1 + self.ssp_ratio) * self.period_time / 4
        self.r_ssp_start_time = (3 - self.ssp_ratio) * self.period_time / 4
        self.r_ssp_end_time = (3 + self.ssp_ratio) * self.period_time / 4
        
        self.phase1_time = (self.l_ssp_start_time + self.l_ssp_end_time) / 2
        self.phase2_time = (self.l_ssp_end_time + self.r_ssp_start_time) / 2
        self.phase3_time = (self.r_ssp_start_time + self.r_ssp_end_time) / 2
        
        self.x_swap_time_advance = self.period_time * p.x_swap_time_advance_ratio
        self.arm_swing_gain = p.arm_swing_gain
    
    # ----------------------------------------------------------
    # updateMovementParam (L363-405)
    # ----------------------------------------------------------
    def _update_movement_param(self):
        p = self.param
        
        # Forward/Back
        self.x_move_scale = max(0.0, min(1.0, self.x_move_scale))
        self.x_move_amplitude = p.x_move_amplitude * self.x_move_scale
        self.x_swap_amplitude = p.x_move_amplitude * p.step_fb_ratio * self.x_move_scale
        self.x_swap_amplitude_shift = p.x_swap_forward_bias * self.x_move_scale
        
        # Right/Left
        self.y_move_amplitude = p.y_move_amplitude / 2
        if self.y_move_amplitude > 0:
            self.y_move_amplitude_shift = self.y_move_amplitude
        else:
            self.y_move_amplitude_shift = -self.y_move_amplitude
        self.y_swap_amplitude = p.y_swap_amplitude + self.y_move_amplitude_shift * 0.04
        
        self.z_move_amplitude = p.z_move_amplitude
        self.z_swap_amplitude = p.z_swap_amplitude
        self.z_swap_amplitude_shift = p.z_swap_amplitude
        self.roll_correction = p.roll_correction
        self.hip_pitch_forward = p.hip_pitch_forward
        
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
        swap_x = self._wsin(self.time + self.x_swap_time_advance, self.x_swap_period_time,
                            self.x_swap_phase_shift, self.x_swap_amplitude,
                            self.x_swap_amplitude_shift)
        swap_y = self._wsin(self.time, self.y_swap_period_time,
                            self.y_swap_phase_shift, self.y_swap_amplitude,
                            self.y_swap_amplitude_shift)
        swap_z = self._wsin(self.time, self.z_swap_period_time,
                            self.z_swap_phase_shift, self.z_swap_amplitude,
                            self.z_swap_amplitude_shift)
        
        t = self.time
        
        if t <= self.l_ssp_start_time:
            # DSP: before left SSP
            left_x  = self._wsin(self.l_ssp_start_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            left_y  = self._wsin(self.l_ssp_start_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            left_z = 0.0
            left_yaw = self._wsin(self.l_ssp_start_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(self.l_ssp_start_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(self.l_ssp_start_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = 0.0
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
            
            left_z = self.z_move_amplitude * self._ssp_lift_profile(
                t, self.l_ssp_start_time, self.l_ssp_end_time
            )
            
            left_yaw = self._wsin(t, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(t, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = 0.0
            right_yaw = self._wsin(t, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                   -self.a_move_amplitude, -self.a_move_amplitude_shift)
        
        elif t <= self.r_ssp_start_time:
            # DSP: between left and right SSP
            left_x  = self._wsin(self.l_ssp_end_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            left_y  = self._wsin(self.l_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            left_z = 0.0
            left_yaw = self._wsin(self.l_ssp_end_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.l_ssp_start_time + math.pi,
                                  self.a_move_amplitude, self.a_move_amplitude_shift)
            right_x = self._wsin(self.l_ssp_end_time, self.x_move_period_time,
                                 self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            right_y = self._wsin(self.l_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.l_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            right_z = 0.0
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
            left_z = 0.0
            left_yaw = self._wsin(t, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                  -self.a_move_amplitude, -self.a_move_amplitude_shift)
                                  
            right_x = self._wsin(t, self.x_move_period_time, phase_r,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            right_y = self._wsin(t, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            
            right_z = self.z_move_amplitude * self._ssp_lift_profile(
                t, self.r_ssp_start_time, self.r_ssp_end_time
            )
            
            right_yaw = self._wsin(t, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                   self.a_move_amplitude, self.a_move_amplitude_shift)
                                   
        
        else:
            # DSP: after right SSP
            phase_r = self.x_move_phase_shift + 2*math.pi/self.x_move_period_time*self.r_ssp_start_time + math.pi
            left_x  = self._wsin(self.r_ssp_end_time, self.x_move_period_time, phase_r,
                                 -self.x_move_amplitude, -self.x_move_amplitude_shift)
            left_y  = self._wsin(self.r_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 -self.y_move_amplitude, -self.y_move_amplitude_shift)
            left_z = 0.0
            left_yaw = self._wsin(self.r_ssp_end_time, self.a_move_period_time,
                                  self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                  -self.a_move_amplitude, -self.a_move_amplitude_shift)
                                  
            right_x = self._wsin(self.r_ssp_end_time, self.x_move_period_time, phase_r,
                                 self.x_move_amplitude, self.x_move_amplitude_shift)
            right_y = self._wsin(self.r_ssp_end_time, self.y_move_period_time,
                                 self.y_move_phase_shift + 2*math.pi/self.y_move_period_time*self.r_ssp_start_time + math.pi,
                                 self.y_move_amplitude, self.y_move_amplitude_shift)
            right_z = 0.0
            right_yaw = self._wsin(self.r_ssp_end_time, self.a_move_period_time,
                                   self.a_move_phase_shift + 2*math.pi/self.a_move_period_time*self.r_ssp_start_time + math.pi,
                                   self.a_move_amplitude, self.a_move_amplitude_shift)
        
        # --- Compute foot endpoints ---
        ep = np.zeros(12)  # [rx, ry, rz, rroll, rpitch, ryaw, lx, ly, lz, lroll, lpitch, lyaw]
        
        ep[0]  = swap_x + right_x + self.x_offset
        ep[1]  = swap_y + right_y - self.y_offset / 2
        ep[2]  = swap_z + right_z + self.z_offset - LEG_LENGTH
        ep[3]  = 0.0 - self.r_offset / 2      # right roll
        ep[4]  = self.p_offset  # right pitch
        ep[5]  = 0.0 + right_yaw - self.a_offset / 2  # right yaw
        
        ep[6]  = swap_x + left_x + self.x_offset
        ep[7]  = swap_y + left_y + self.y_offset / 2
        ep[8]  = swap_z + left_z + self.z_offset - LEG_LENGTH
        ep[9]  = 0.0 + self.r_offset / 2       # left roll
        ep[10] = self.p_offset  # left pitch
        ep[11] = 0.0 + left_yaw + self.a_offset / 2  # left yaw

        self.last_foot_endpoints = ep.copy()
        
        # 발의 roll/pitch 목표는 0으로 유지하고, 무게중심 이동은 y_swap 병진으로 처리한다.
        
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

        # IK와 축 방향 계산은 그대로 두고, 실물 백래시 보정 3도만 마지막에
        # 독립적으로 더한다. 따라서 이 부호를 바꿔도 y_swap 등 IK 방향은
        # 영향을 받지 않는다.
        leg_angle += LEG_ROLL_CORRECTION_SIGNS * self.roll_correction
        leg_angle += LEG_HIP_PITCH_FORWARD_SIGNS * self.hip_pitch_forward
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
        # Internal order: right leg, then left leg.
        # Robot order: left leg, right leg, then arm joints.
        output = np.zeros(17)
        # Left leg
        output[0:6] = final_leg[6:12]
        # Right leg
        output[6:12] = final_leg[0:6]
        # Arms: all zero (상체 무게중심 수직 유지)
        output[12:17] = 0.0
        
        # Advance time
        if self.real_running:
            self.time += time_unit
            if self.time >= self.period_time:
                self.time = 0
                self.x_move_scale = min(
                    1.0,
                    self.x_move_scale + self.param.x_move_ramp_per_cycle,
                )
                self.previous_x_move_amplitude = self.x_move_amplitude
        
        return output
    
    # ----------------------------------------------------------
    # start / stop
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
class IKWalkerNode(Node):
    def __init__(self):
        super().__init__('ik_walker')
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        
        defaults = WalkingParam()
        self.declare_parameter('period_time', defaults.period_time)
        self.declare_parameter('dsp_ratio', defaults.dsp_ratio)
        self.declare_parameter('step_fb_ratio', defaults.step_fb_ratio)
        self.declare_parameter('x_swap_forward_bias', defaults.x_swap_forward_bias)
        self.declare_parameter('x_swap_time_advance_ratio', defaults.x_swap_time_advance_ratio)
        self.declare_parameter('x_move_start_scale', defaults.x_move_start_scale)
        self.declare_parameter('x_move_ramp_per_cycle', defaults.x_move_ramp_per_cycle)
        self.declare_parameter('x_move_amplitude', defaults.x_move_amplitude)
        self.declare_parameter('z_move_amplitude', defaults.z_move_amplitude)
        self.declare_parameter('y_swap_amplitude', defaults.y_swap_amplitude)
        self.declare_parameter(
            'roll_correction_deg',
            math.degrees(defaults.roll_correction),
        )
        self.declare_parameter('init_x_offset', defaults.init_x_offset)
        self.declare_parameter('init_y_offset', defaults.init_y_offset)
        self.declare_parameter('init_z_offset', defaults.init_z_offset)
        self.declare_parameter(
            'hip_pitch_forward_deg',
            math.degrees(defaults.hip_pitch_forward),
        )
        self.declare_parameter('arm_shoulder_pitch_deg', -70.0)
        self.declare_parameter('control_cycle', 0.008)
        self.declare_parameter('trajectory_time_scale', 4.0)
        self.declare_parameter('startup_duration_sec', 3.0)
        self.declare_parameter('shutdown_duration_sec', 2.0)
        self.declare_parameter('num_cycles', 1)

        # Walking parameters
        # NOTE: 이 로봇의 전진 방향은 -X
        #       따라서 x_move_amplitude를 음수로 설정!
        #
        # 이 로봇의 힙 간격 = 12.5cm (URDF: L=-0.01425, R=-0.13925)
        # 상체 수평을 우선하므로 기본값은 골반 롤을 쓰지 않고 y_swap 병진으로 CoM을 옮긴다.
        param = defaults
        param.init_x_offset = float(self.get_parameter('init_x_offset').value)
        param.init_y_offset = float(self.get_parameter('init_y_offset').value)
        param.init_z_offset = float(self.get_parameter('init_z_offset').value)
        param.hip_pitch_forward = math.radians(
            float(self.get_parameter('hip_pitch_forward_deg').value)
        )
        param.period_time = float(self.get_parameter('period_time').value)
        param.dsp_ratio = float(self.get_parameter('dsp_ratio').value)
        param.step_fb_ratio = float(self.get_parameter('step_fb_ratio').value)
        param.x_swap_forward_bias = float(self.get_parameter('x_swap_forward_bias').value)
        param.x_swap_time_advance_ratio = float(
            self.get_parameter('x_swap_time_advance_ratio').value
        )
        param.x_move_start_scale = float(self.get_parameter('x_move_start_scale').value)
        param.x_move_ramp_per_cycle = float(self.get_parameter('x_move_ramp_per_cycle').value)
        param.x_move_amplitude = float(self.get_parameter('x_move_amplitude').value)
        param.z_move_amplitude = float(self.get_parameter('z_move_amplitude').value)
        # 평행사변형 무게중심 이동: 현재 좌표계에서는 음수 방향이 지지발 쪽 이동에 맞는다.
        # 힙 간격 12.5cm, 지지발 위로 CoM 이동에 충분한 병진량
        param.y_swap_amplitude = float(self.get_parameter('y_swap_amplitude').value)
        param.roll_correction = math.radians(
            float(self.get_parameter('roll_correction_deg').value)
        )
        self.engine = IKWalkerEngine(param)
        
        self.joint_names = [
            'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
            'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
            'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
            'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
            'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
            'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt',
        ]
        
        self.control_cycle = float(self.get_parameter('control_cycle').value)
        self.trajectory_time_scale = max(
            0.1,
            float(self.get_parameter('trajectory_time_scale').value),
        )
        self.startup_duration_sec = max(
            0.1,
            float(self.get_parameter('startup_duration_sec').value),
        )
        self.shutdown_duration_sec = max(
            0.1,
            float(self.get_parameter('shutdown_duration_sec').value),
        )
        self.num_cycles = max(1, int(self.get_parameter('num_cycles').value))
        self.arm_shoulder_pitch = math.radians(
            float(self.get_parameter('arm_shoulder_pitch_deg').value)
        )
        self.get_logger().info(
            'Walking params: '
            f'period={param.period_time:.3f}s, x={param.x_move_amplitude:.3f}m, '
            f'z={param.z_move_amplitude:.3f}m, y_swap={param.y_swap_amplitude:.3f}m, '
            f'step_fb_ratio={param.step_fb_ratio:.2f}, '
            f'x_swap_forward_bias={param.x_swap_forward_bias:.3f}m, '
            f'x_swap_time_advance={param.x_swap_time_advance_ratio:.2f}T, '
            f'x_ramp={param.x_move_start_scale:.2f}+{param.x_move_ramp_per_cycle:.2f}/cycle, '
            f'foot_spacing_offset={param.init_y_offset:.3f}m, '
            f'roll_correction={math.degrees(param.roll_correction):.2f}deg, '
            f'hip_pitch_forward={math.degrees(param.hip_pitch_forward):.2f}deg, '
            f'startup_duration={self.startup_duration_sec:.2f}s, '
            f'shutdown_duration={self.shutdown_duration_sec:.2f}s, '
            f'time_scale={self.trajectory_time_scale:.2f}, '
            f'arm_shoulder={math.degrees(self.arm_shoulder_pitch):.2f}deg, '
            f'cycles={self.num_cycles}, steps={self.num_cycles * 2}'
        )
        
        self.timer = self.create_timer(2.0, self.generate_trajectory)

    @staticmethod
    def _duration_from_seconds(seconds: float) -> Duration:
        sec = int(seconds)
        nanosec = int(round((seconds - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        return Duration(sec=sec, nanosec=nanosec)

    def _with_arm_posture(self, angles) -> list:
        positions = angles.tolist()
        positions[13] = self.arm_shoulder_pitch
        positions[14] = 0.0
        return positions
    
    def generate_trajectory(self):
        self.timer.cancel()
        self.get_logger().info(
            'Generating walking trajectory '
            f'({self.num_cycles} cycles = {self.num_cycles * 2} steps)...'
        )
        
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        # 1) Move continuously from the measured hardware pose to the gait init pose.
        self.engine._update_time_param()
        self.engine._update_movement_param()
        init_angles = self.engine.step(self.control_cycle)
        if init_angles is not None:
            p0 = JointTrajectoryPoint()
            p0.positions = self._with_arm_posture(init_angles)
            p0.time_from_start = self._duration_from_seconds(self.startup_duration_sec)
            msg.points.append(p0)
        
        # 2) Start walking
        self.engine.time = 0.0
        self.engine.start()
        
        total_time = self.engine.param.period_time * self.num_cycles
        t = 0.0
        # trajectory_time_scale changes only the gait playback speed. The initial
        # transition has its own fixed duration so slow-motion walking does not add
        # a long idle delay before the first step.
        base_time = self.startup_duration_sec
        point_count = 0
        last_angles = None
        
        while t < total_time:
            angles = self.engine.step(self.control_cycle)
            if angles is not None:
                last_angles = angles
                point = JointTrajectoryPoint()
                point.positions = self._with_arm_posture(angles)
                abs_t = base_time + (t + self.control_cycle) * self.trajectory_time_scale
                point.time_from_start = self._duration_from_seconds(abs_t)
                msg.points.append(point)
                point_count += 1
            
            t += self.control_cycle
            
            if point_count % 200 == 0:
                self.get_logger().info(f'  Generated {point_count} points ({t:.1f}/{total_time:.1f}s)')
        
        # Add one clean cycle-boundary point, then smoothly remove the roll
        # correction without replaying or reversing the last step.
        # A cycle is left + right swing, so num_cycles=5 means 10 forward steps.
        final_angles = last_angles
        boundary_time = base_time + (
            total_time + self.control_cycle
        ) * self.trajectory_time_scale
        if final_angles is not None:
            point = JointTrajectoryPoint()
            point.positions = self._with_arm_posture(final_angles)
            point.time_from_start = self._duration_from_seconds(boundary_time)
            msg.points.append(point)
            point_count += 1

            normalized = JointTrajectoryPoint()
            normalized.positions = self._with_arm_posture(
                without_roll_correction(
                    final_angles, self.engine.param.roll_correction
                )
            )
            normalized.time_from_start = self._duration_from_seconds(
                boundary_time + self.shutdown_duration_sec
            )
            msg.points.append(normalized)
            point_count += 1

        self.engine.stop()
        
        self.publisher_.publish(msg)
        playback_time = boundary_time + self.shutdown_duration_sec
        self.get_logger().info(
            f'Published {point_count} walking points: '
            f'{self.num_cycles} cycles / {self.num_cycles * 2} steps, '
            f'engine_time={total_time:.1f}s, playback_time={playback_time:.1f}s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = IKWalkerNode()
    rclpy.spin_once(node, timeout_sec=30.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
