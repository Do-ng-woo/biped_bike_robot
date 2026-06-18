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
    0.0, 0.0, -1.8, -2.3, 1.3, 0.0,
    0.0, 0.0, 1.8, -2.3, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
DEEP_SQUAT_SUPPORTED = (
    0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
    0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
    3.14159, SHOULDER_BACK_LIMIT_RAD, 0.0, 0.0, 0.0,
)
DEEP_SQUAT_ARMS_NORMAL = (
    0.0, 0.0, -1.3, -2.3, 1.3, 0.0,
    0.0, 0.0, 1.3, -2.3, 1.3, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0,
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
    DEEP_SQUAT_ARMS_NORMAL,
    READY,
)

# Ready -> bike: exact reverse of the stable revert path, followed by releasing
# the support arm into the existing final bike pose.
TRANSFORM_SEQUENCE = tuple(reversed(REVERT_SEQUENCE[:-1])) + (BIKE_FINAL,)
