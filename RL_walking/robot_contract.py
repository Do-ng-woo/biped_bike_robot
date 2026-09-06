"""Hardware joint order and common ready pose shared by both policies."""

ACTOR_JOINTS = (
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
  "arm_base_yaw_jnt",
  "arm_shoulder_pitch_jnt",
  "arm_elbow_pitch_jnt",
  "arm_wrist_pitch_jnt",
  "arm_wrist_roll_jnt",
)

TARGET_JOINTS = ACTOR_JOINTS
ARM_POSE = (0.0, -1.22173, 0.0872665, 0.0, 0.0)
HARDWARE_READY_TARGET = (
  0.0,
  0.0,
  0.0,
  -0.30,
  0.411799,
  0.0,
  0.0,
  0.0,
  0.0,
  -0.30,
  0.411799,
  0.0,
  *ARM_POSE,
)

assert len(HARDWARE_READY_TARGET) == len(TARGET_JOINTS)
