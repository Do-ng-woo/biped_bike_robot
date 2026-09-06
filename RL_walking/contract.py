"""Shared training and hardware contract for reference-G1-style walking."""

LOWER_BODY_JOINTS = (
  "l_hip_yaw_jnt",
  "l_hip_roll_jnt",
  "l_hip_pitch_jnt",
  "l_knee_pitch_jnt",
  "l_ankle_pitch_jnt",
  "l_foot_roll_jnt",
  "r_hip_yaw_jnt",
  "r_hip_roll_jnt",
  "r_hip_pitch_jnt",
  "r_knee_pitch_jnt",
  "r_ankle_pitch_jnt",
  "r_foot_roll_jnt",
)

TARGET_JOINTS = LOWER_BODY_JOINTS + (
  "arm_base_yaw_jnt",
  "arm_shoulder_pitch_jnt",
  "arm_elbow_pitch_jnt",
  "arm_wrist_pitch_jnt",
  "arm_wrist_roll_jnt",
)

# This is both the training action offset and the hardware takeover target.
READY_TARGET = (
  -0.002625883,
  -0.037534283,
  -0.328608305,
  -0.879421005,
  0.550763551,
  0.037625983,
  0.002625883,
  0.037534283,
  0.328608305,
  -0.879421005,
  0.550763551,
  -0.037625983,
  0.0,
  -1.22173,
  0.0872665,
  0.0,
  0.0,
)

# G1 formula: residual scale = 0.25 * actuator effort limit / stiffness.
# The crouched ready pose leaves enough room that all 12 joints can use these
# actuator-derived residuals in both directions without approaching MJCF limits.
ACTION_SCALE_NEGATIVE = (
  0.063636364,
  0.050000000,
  0.079545455,
  0.159574468,
  0.079545455,
  0.144230769,
  0.063636364,
  0.050000000,
  0.079545455,
  0.159574468,
  0.079545455,
  0.144230769,
)

ACTION_SCALE_POSITIVE = ACTION_SCALE_NEGATIVE

OBSERVATION_HISTORY_LENGTH = 5
OBSERVATION_SLICES = {
  "base_ang_vel": (0, 15),
  "projected_gravity": (15, 30),
  "joint_pos": (30, 90),
  "joint_vel": (90, 150),
  "actions": (150, 210),
  "command": (210, 225),
}

OBSERVATION_DIM = 225
ACTION_DIM = 12
POLICY_DT = 0.02
DEFAULT_FORWARD_SPEED = 0.08
DEFAULT_LATERAL_SPEED = 0.0
DEFAULT_YAW_RATE = 0.0
GAIT_ACTIVATION_SPEED = 0.08
GAIT_LATERAL_ENCODING_SCALE = 0.25
GAIT_YAW_ENCODING_SCALE = 0.20

assert len(LOWER_BODY_JOINTS) == ACTION_DIM
assert len(ACTION_SCALE_NEGATIVE) == ACTION_DIM
assert len(ACTION_SCALE_POSITIVE) == ACTION_DIM
assert len(READY_TARGET) == len(TARGET_JOINTS)
