#!/usr/bin/env python3
"""Shared, pure waypoint definitions for bike transform and revert motions."""

JOINT_NAMES = (
    'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
    'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
    'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
    'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
    'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
    'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt',
)

# Mechanical interference starts at roughly 25 degrees behind neutral.
SHOULDER_BACK_LIMIT_RAD = -0.436332
SHOULDER_UP_RAD = 0.349066
HIP_PITCH_FOLD_LIMIT_RAD = 1.5708
KNEE_PITCH_FOLD_LIMIT_RAD = -2.0944
WRIST_PITCH_DOWN_RAD = -1.5708
WRIST_PITCH_INDEX = JOINT_NAMES.index('arm_wrist_pitch_jnt')


def with_wrist_pitch_down(positions):
    updated = list(positions)
    updated[WRIST_PITCH_INDEX] = WRIST_PITCH_DOWN_RAD
    return tuple(updated)


BIKE_SUPPORTED = (
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
LEGS_STRAIGHT_SUPPORTED = (
    0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
FOLDED_SUPPORTED = (
    0.0, 0.0, -HIP_PITCH_FOLD_LIMIT_RAD, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, HIP_PITCH_FOLD_LIMIT_RAD, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
DEEP_SQUAT_SUPPORTED = (
    0.0, 0.0, -1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
BACK_SHIFT_SUPPORTED = (
    0.0, 0.0, -0.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 0.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
DEEP_SQUAT_CLAW_PITCH_DOWN = (
    0.0, 0.0, -1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, WRIST_PITCH_DOWN_RAD, 0.0,
)
DEEP_SQUAT_ARMS_NORMAL = (
    0.0, 0.0, -1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
)
DEEP_SQUAT_ARMS_UP = (
    0.0, 0.0, -1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, 0.0, 1.3, KNEE_PITCH_FOLD_LIMIT_RAD, 1.3, 0.0,
    0.0, SHOULDER_UP_RAD, 0.0, 0.0, 0.0,
)
READY = (
    0.0, 0.0, -0.35, -0.70, 0.35, 0.0,
    0.0, 0.0, 0.35, -0.70, 0.35, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
)
BIKE_FINAL = (
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    3.14159, 0.26, 0.0, 0.0, 0.0,
)


# Bike -> ready. The -25 degree shoulder command is intentionally repeated while
# the arm supports the chassis; it is a held position, not a separate constraint.
REVERT_SEQUENCE = (
    BIKE_SUPPORTED,
    LEGS_STRAIGHT_SUPPORTED,
    FOLDED_SUPPORTED,
    DEEP_SQUAT_SUPPORTED,
    BACK_SHIFT_SUPPORTED,
    DEEP_SQUAT_ARMS_NORMAL,
    READY,
)

# Ready -> bike. Lower the shoulder while the arm base yaw rotates to 180
# degrees, then pitch the end gripper down and continue the fold.
TRANSFORM_SEQUENCE = (
    DEEP_SQUAT_ARMS_UP,
    DEEP_SQUAT_SUPPORTED,
    DEEP_SQUAT_CLAW_PITCH_DOWN,
    with_wrist_pitch_down(FOLDED_SUPPORTED),
    with_wrist_pitch_down(LEGS_STRAIGHT_SUPPORTED),
    with_wrist_pitch_down(BIKE_SUPPORTED),
    with_wrist_pitch_down(BIKE_FINAL),
)
