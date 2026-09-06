#!/usr/bin/env python3
"""Shared, pure waypoint definitions for bike transform and revert motions."""

import math


JOINT_NAMES = (
    'l_hip_yaw_jnt', 'l_hip_roll_jnt', 'l_hip_pitch_jnt',
    'l_knee_pitch_jnt', 'l_ankle_pitch_jnt', 'l_foot_roll_jnt',
    'r_hip_yaw_jnt', 'r_hip_roll_jnt', 'r_hip_pitch_jnt',
    'r_knee_pitch_jnt', 'r_ankle_pitch_jnt', 'r_foot_roll_jnt',
    'arm_base_yaw_jnt', 'arm_shoulder_pitch_jnt', 'arm_elbow_pitch_jnt',
    'arm_wrist_pitch_jnt', 'arm_wrist_roll_jnt',
)


# ============================================================
# Arm constants
# ============================================================

SHOULDER_READY_RAD = -1.22173
SHOULDER_YAWED_SUPPORT_RAD = -1.91986
ELBOW_LIFT_RAD = math.radians(20.0)

SHOULDER_UP_RAD = SHOULDER_READY_RAD

WRIST_PITCH_DOWN_RAD = -1.5708
WRIST_ROLL_YAWED_RAD = math.pi


# ============================================================
# Leg constants
# ============================================================

BIKE_HIP_YAW_INWARD_RAD = math.radians(5.0)

HIP_PITCH_FOLD_LIMIT_RAD = 1.74533

DEEP_SQUAT_HIP_PITCH_RAD = 0.7764
DEEP_SQUAT_ANKLE_PITCH_RAD = 1.48353
KNEE_PITCH_FOLD_LIMIT_RAD = -2.0944


# ============================================================
# Ready pose
# ============================================================

READY_HIP_FORWARD_OFFSET_RAD = 0.0
READY_HIP_PITCH_RAD = 0.0

READY_KNEE_PITCH_RAD = -0.30

READY_ANKLE_FORWARD_OFFSET_RAD = 0.174533
READY_ANKLE_PITCH_RAD = (
    0.15 + READY_ANKLE_FORWARD_OFFSET_RAD
)


# ============================================================
# Joint indices
# ============================================================

WRIST_PITCH_INDEX = JOINT_NAMES.index(
    'arm_wrist_pitch_jnt'
)
WRIST_ROLL_INDEX = JOINT_NAMES.index(
    'arm_wrist_roll_jnt'
)

L_HIP_PITCH_INDEX = JOINT_NAMES.index('l_hip_pitch_jnt')
R_HIP_PITCH_INDEX = JOINT_NAMES.index('r_hip_pitch_jnt')


# ============================================================
# Revert rise tuning
#
# 목표:
#   1) deep squat에서 바로 무릎을 펴지 않는다.
#   2) 먼저 hip pitch를 약간 ready 방향으로 보내 상체/골반을 뒤로 둔다.
#   3) 그 다음 knee/ankle을 펴면서 hip을 더 빠르게 복귀시킨다.
#   4) 거의 선 뒤에 마지막으로 READY에 들어간다.
#
# 아래 HIP_RETURN 값은 "deep squat hip 각도에서 READY 방향으로
# 얼마나 복귀했는가"를 뜻한다.
#
# 예: DEEP_SQUAT_HIP_PITCH_RAD ~= 44.5 deg 이므로
#     EARLY 25 deg return -> hip에 약 19.5 deg가 남는다.
# ============================================================

# Publisher에서 이 factor 이후의 구간만 별도로 느리게 재생할 수 있다.
# REVERT_YAWED_SHOULDER_READY 이후부터 hip-back / rise 구간으로 본다.
REVERT_RISE_START_FACTOR = 7.0

# 무릎을 펴기 전에 hip만 먼저 더 크게 열어 뒤쪽으로 체중을 만든다.
# ELBOW_LIFT_RAD=20 deg와 매니퓰레이터 질량을 유지한 상태에서
# 전방 쏠림을 줄이기 위해 기존 10 deg -> 15 deg로 강화한다.
REVERT_PRE_RISE_HIP_RETURN_RAD = math.radians(15.0)

# knee / ankle 진행률과, 같은 시점의 hip 누적 복귀량.
# rise 초반에는 hip을 knee/ankle보다 빠르게 열어 backward bias를 유지한다.
REVERT_RISE_EARLY_RATIO = 0.30
REVERT_RISE_EARLY_HIP_RETURN_RAD = math.radians(28.0)

REVERT_RISE_MID_RATIO = 0.65
REVERT_RISE_MID_HIP_RETURN_RAD = math.radians(40.0)

REVERT_RISE_LATE_RATIO = 0.85
REVERT_RISE_LATE_HIP_RETURN_RAD = math.radians(43.5)


# ============================================================
# Helpers
# ============================================================

def with_wrist_pitch_down(positions):
    """Return a copy of a posture with wrist pitch fixed at -90 deg."""
    updated = list(positions)
    updated[WRIST_PITCH_INDEX] = WRIST_PITCH_DOWN_RAD
    return tuple(updated)


def with_wrist_roll_yawed(positions):
    """Return a copy of a posture with wrist roll/yaw rotated 180 deg."""
    updated = list(positions)
    updated[WRIST_ROLL_INDEX] = WRIST_ROLL_YAWED_RAD
    return tuple(updated)


def with_wrist_pitch_down_and_roll_yawed(positions):
    """Return a copy with wrist roll/yaw at 180 deg before pitch folds."""
    return with_wrist_pitch_down(
        with_wrist_roll_yawed(positions)
    )


def hip_pose_from_return(base_positions, hip_return_rad):
    """
    Return a copy of ``base_positions`` with only hip pitch changed.

    ``hip_return_rad`` is measured from the deep-squat hip angle
    toward READY.  Left/right hip joint axes are mirrored in this model,
    so the command signs are opposite.
    """
    updated = list(base_positions)

    hip_return_rad = min(
        max(float(hip_return_rad), 0.0),
        DEEP_SQUAT_HIP_PITCH_RAD,
    )

    hip_remaining = (
        DEEP_SQUAT_HIP_PITCH_RAD
        - hip_return_rad
    )

    updated[L_HIP_PITCH_INDEX] = -hip_remaining
    updated[R_HIP_PITCH_INDEX] = hip_remaining

    return tuple(updated)


def interpolate_with_hip_return(
    start,
    end,
    ratio,
    hip_return_rad,
):
    """
    Interpolate the full posture by ``ratio`` but command hip pitch
    independently so the hip opens earlier than knee/ankle.

    This creates a backward-biased rise instead of allowing knee
    extension to push the pelvis/torso forward.
    """
    ratio = min(max(float(ratio), 0.0), 1.0)

    updated = [
        before + (after - before) * ratio
        for before, after in zip(start, end)
    ]

    hip_return_rad = min(
        max(float(hip_return_rad), 0.0),
        DEEP_SQUAT_HIP_PITCH_RAD,
    )

    hip_remaining = (
        DEEP_SQUAT_HIP_PITCH_RAD
        - hip_return_rad
    )

    updated[L_HIP_PITCH_INDEX] = -hip_remaining
    updated[R_HIP_PITCH_INDEX] = hip_remaining

    return tuple(updated)


def yawed_revert_arm_clearance(shoulder, wrist_pitch, wrist_roll=0.0):
    return (
        0.0,
        0.0,
        -DEEP_SQUAT_HIP_PITCH_RAD,
        KNEE_PITCH_FOLD_LIMIT_RAD,
        DEEP_SQUAT_ANKLE_PITCH_RAD,
        0.0,

        0.0,
        0.0,
        DEEP_SQUAT_HIP_PITCH_RAD,
        KNEE_PITCH_FOLD_LIMIT_RAD,
        DEEP_SQUAT_ANKLE_PITCH_RAD,
        0.0,

        3.14159,
        shoulder,
        ELBOW_LIFT_RAD,
        wrist_pitch,
        wrist_roll,
    )


# ============================================================
# Supported postures
# ============================================================

BIKE_SUPPORTED = (
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    0.0, 0.0, 0.0, 0.0, -1.57, 0.0,
    3.14159, SHOULDER_READY_RAD, ELBOW_LIFT_RAD, 0.0, 0.0,
)


LEGS_STRAIGHT_SUPPORTED = (
    0.0, 0.0, 0.0, 0.0, DEEP_SQUAT_ANKLE_PITCH_RAD, 0.0,
    0.0, 0.0, 0.0, 0.0, DEEP_SQUAT_ANKLE_PITCH_RAD, 0.0,
    3.14159, SHOULDER_READY_RAD, ELBOW_LIFT_RAD, 0.0, 0.0,
)


FOLDED_SUPPORTED = (
    0.0,
    0.0,
    -HIP_PITCH_FOLD_LIMIT_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    HIP_PITCH_FOLD_LIMIT_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_YAWED_SUPPORT_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


HIP_RETURN_SUPPORTED = (
    0.0,
    0.0,
    -HIP_PITCH_FOLD_LIMIT_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    HIP_PITCH_FOLD_LIMIT_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_YAWED_SUPPORT_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


DEEP_SQUAT_SUPPORTED = (
    0.0,
    0.0,
    -DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_YAWED_SUPPORT_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


# 기존 테스트용 posture.
# 현재 REVERT_SEQUENCE에서는 직접 사용하지 않는다.
BACK_SHIFT_SUPPORTED = (
    0.0, 0.0, -0.3,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0, 0.0, 0.3,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_YAWED_SUPPORT_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


DEEP_SQUAT_WRIST_ROLL_YAWED = with_wrist_roll_yawed(
    DEEP_SQUAT_SUPPORTED
)


DEEP_SQUAT_CLAW_PITCH_DOWN = (
    0.0,
    0.0,
    -DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_YAWED_SUPPORT_RAD,
    ELBOW_LIFT_RAD,
    WRIST_PITCH_DOWN_RAD,
    WRIST_ROLL_YAWED_RAD,
)


# ============================================================
# Revert-specific postures
# ============================================================

REVERT_YAWED_SHOULDER_READY = (
    yawed_revert_arm_clearance(
        SHOULDER_READY_RAD,
        0.0,
        WRIST_ROLL_YAWED_RAD,
    )
)


REVERT_AFTER_YAW_SHOULDER_READY = (
    0.0,
    0.0,
    -DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    SHOULDER_READY_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


# 무릎/발목과 매니퓰레이터는 그대로 둔 상태에서
# hip만 먼저 ready 방향으로 15 deg 복귀시킨 자세.
# 중요한 점: arm base yaw / wrist roll을 아직 되돌리지 않는다.
# 즉 안정된 yawed arm configuration을 유지한 채 먼저 뒤쪽 bias를 만든다.
REVERT_YAWED_HIP_BACK = hip_pose_from_return(
    REVERT_YAWED_SHOULDER_READY,
    REVERT_PRE_RISE_HIP_RETURN_RAD,
)

# 동일 자세를 한 waypoint 더 유지하여 hip 이동 후 남은 운동량이
# knee extension과 바로 겹치지 않도록 한다.
REVERT_HIP_BACK_HOLD = REVERT_YAWED_HIP_BACK

# 이전 이름을 외부 코드가 참조하고 있을 수 있으므로 호환용 alias는 유지한다.
REVERT_AFTER_YAW_HIP_BACK = REVERT_YAWED_HIP_BACK


DEEP_SQUAT_ARMS_NORMAL = (
    REVERT_AFTER_YAW_SHOULDER_READY
)


DEEP_SQUAT_ARMS_UP = (
    0.0,
    0.0,
    -DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    DEEP_SQUAT_HIP_PITCH_RAD,
    KNEE_PITCH_FOLD_LIMIT_RAD,
    DEEP_SQUAT_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    SHOULDER_UP_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


# ============================================================
# Ready
# ============================================================

READY = (
    0.0,
    0.0,
    -READY_HIP_PITCH_RAD - READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    READY_HIP_PITCH_RAD + READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    SHOULDER_READY_RAD,
    0.0,
    0.0,
    0.0,
)


HARDWARE_READY = READY


# ============================================================
# Standing target while the manipulator is held fixed
# ============================================================

# 다리는 READY까지 펴지만, 일어서는 동안 매니퓰레이터 configuration은
# REVERT_YAWED_SHOULDER_READY와 동일하게 유지한다.
# 이렇게 하면 rise 보간 중에 arm base yaw / elbow / wrist가 동시에 움직여
# 상체 CoM을 다시 흔드는 것을 막을 수 있다.
STANDING_ARM_HELD = (
    0.0,
    0.0,
    -READY_HIP_PITCH_RAD - READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    READY_HIP_PITCH_RAD + READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    3.14159,
    SHOULDER_READY_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    WRIST_ROLL_YAWED_RAD,
)


# 완전히 선 뒤 arm base yaw / wrist roll만 먼저 READY 방향으로 복귀한다.
# ELBOW_LIFT_RAD=20 deg는 링크 간섭 방지를 위해 이 단계까지 유지한다.
STANDING_ARM_BASE_RETURNED = (
    0.0,
    0.0,
    -READY_HIP_PITCH_RAD - READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    0.0,
    READY_HIP_PITCH_RAD + READY_HIP_FORWARD_OFFSET_RAD,
    READY_KNEE_PITCH_RAD,
    READY_ANKLE_PITCH_RAD,
    0.0,

    0.0,
    SHOULDER_READY_RAD,
    ELBOW_LIFT_RAD,
    0.0,
    0.0,
)


# ============================================================
# Squat -> ready rise waypoints
# ============================================================

# 1) PRE-RISE
# REVERT_YAWED_HIP_BACK에서 knee/ankle과 arm은 그대로 유지하고
# hip만 15 deg 먼저 복귀한다.
#
# 2) HOLD
# REVERT_HIP_BACK_HOLD에서 같은 자세를 잠시 유지한다.
#
# 3) EARLY
# knee / ankle = 전체 복귀의 30%
# hip          = deep squat 기준 28 deg 복귀
REVERT_RISE_EARLY = (
    interpolate_with_hip_return(
        REVERT_YAWED_HIP_BACK,
        STANDING_ARM_HELD,
        REVERT_RISE_EARLY_RATIO,
        REVERT_RISE_EARLY_HIP_RETURN_RAD,
    )
)


# 4) MID
# knee / ankle = 전체 복귀의 65%
# hip          = deep squat 기준 40 deg 복귀
REVERT_RISE_MID = (
    interpolate_with_hip_return(
        REVERT_YAWED_HIP_BACK,
        STANDING_ARM_HELD,
        REVERT_RISE_MID_RATIO,
        REVERT_RISE_MID_HIP_RETURN_RAD,
    )
)


# 5) LATE
# knee / ankle = 전체 복귀의 85%
# hip          = deep squat 기준 43.5 deg 복귀
# READY 직전까지 작은 backward bias를 남겨 마지막 전방 쏠림을 줄인다.
REVERT_RISE_LATE = (
    interpolate_with_hip_return(
        REVERT_YAWED_HIP_BACK,
        STANDING_ARM_HELD,
        REVERT_RISE_LATE_RATIO,
        REVERT_RISE_LATE_HIP_RETURN_RAD,
    )
)


# ============================================================
# Final bike posture
# ============================================================

BIKE_FINAL = (
    BIKE_HIP_YAW_INWARD_RAD, 0.0, 0.0, 0.0, -1.57, 0.0,
    -BIKE_HIP_YAW_INWARD_RAD, 0.0, 0.0, 0.0, -1.57, 0.0,
    3.14159, SHOULDER_READY_RAD, ELBOW_LIFT_RAD, 0.0, WRIST_ROLL_YAWED_RAD,
)


# ============================================================
# Bike -> Ready
# ============================================================
#
# standing 구간만 보면:
#
# 7.0  : yawed arm clearance 자세, deep squat 유지
# 8.0  : hip만 15 deg 먼저 복귀 (pre-rise)
# 8.4  : 같은 hip-back 자세 HOLD
# 9.0  : knee/ankle 30%, hip 28 deg 누적 복귀
# 9.8  : knee/ankle 65%, hip 40 deg 누적 복귀
# 10.6 : knee/ankle 85%, hip 43.5 deg 누적 복귀
# 11.2 : 다리는 READY, arm은 yawed/20 deg elbow 상태 유지
# 11.8 : 선 상태에서 arm base yaw / wrist roll만 복귀
# 12.4 : 마지막으로 elbow까지 HARDWARE_READY 복귀
#
# publisher의 rise_time_scale > 1.0을 사용하면 7.0 이후 구간만
# 추가로 느리게 재생된다. 따라서 hip-back 동작부터 천천히 수행한다.
# ============================================================

REVERT_SEQUENCE = (
    with_wrist_pitch_down_and_roll_yawed(BIKE_FINAL),
    with_wrist_pitch_down_and_roll_yawed(BIKE_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(LEGS_STRAIGHT_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(HIP_RETURN_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(FOLDED_SUPPORTED),
    DEEP_SQUAT_CLAW_PITCH_DOWN,
    REVERT_YAWED_SHOULDER_READY,

    # 먼저 hip으로 뒤쪽 bias를 만든다.
    REVERT_YAWED_HIP_BACK,

    # 같은 자세를 유지해 hip 이동의 운동량을 죽인다.
    REVERT_HIP_BACK_HOLD,

    # 팔은 완전히 고정한 채 다리만 일어난다.
    REVERT_RISE_EARLY,
    REVERT_RISE_MID,
    REVERT_RISE_LATE,
    STANDING_ARM_HELD,

    # 완전히 선 다음 arm을 두 단계로 천천히 READY에 복귀한다.
    STANDING_ARM_BASE_RETURNED,
    HARDWARE_READY,
)


REVERT_POINT_TIME_FACTORS = (
    1.0,
    2.0,
    3.0,
    4.0,
    5.0,
    6.0,
    7.0,
    8.0,
    8.4,
    9.0,
    9.8,
    10.6,
    11.2,
    11.8,
    12.4,
)


assert len(REVERT_SEQUENCE) == len(REVERT_POINT_TIME_FACTORS)


# ============================================================
# Ready -> Bike
# ============================================================

TRANSFORM_SEQUENCE = (
    DEEP_SQUAT_ARMS_UP,
    DEEP_SQUAT_SUPPORTED,
    # Rotate the wrist roll/yaw first, then fold wrist pitch down.
    DEEP_SQUAT_WRIST_ROLL_YAWED,
    DEEP_SQUAT_CLAW_PITCH_DOWN,
    with_wrist_pitch_down_and_roll_yawed(FOLDED_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(HIP_RETURN_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(LEGS_STRAIGHT_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(BIKE_SUPPORTED),
    with_wrist_pitch_down_and_roll_yawed(BIKE_FINAL),
)
