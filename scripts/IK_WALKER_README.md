# IK Walker Technical Notes

`ik_walker.py` generates walking trajectories for `biped_bike_robot`.
It uses a sinusoidal gait pattern, lateral center-of-mass shifting, and
analytical 6-DOF leg IK to produce a 17-DOF `JointTrajectory`.

## Architecture

```text
IKWalkerNode (ROS 2)
  WalkingParam -> IKWalkerEngine -> JointTrajectory
```

| Module | Role |
|---|---|
| `WalkingParam` | Walking stride, timing, lift height, and compensation parameters |
| `IKWalkerEngine` | Sinusoidal gait phase generation and leg IK |
| `analytical_ik_leg()` | 6-DOF leg inverse kinematics |
| `IKWalkerNode` | ROS 2 publisher for `/joint_trajectory_controller/joint_trajectory` |

## Core Idea

The controller shifts the body laterally before lifting a foot, so the center of
mass moves over the support foot. The pelvis is kept level as much as possible,
and the legs absorb most of the side-to-side motion.

The gait cycle is split into four phases:

| Phase | Meaning |
|---|---|
| DSP start | Both feet support the robot while weight transfer begins |
| Left SSP | Left foot swings while the right foot supports |
| DSP middle | Both feet support the robot while weight transfer reverses |
| Right SSP | Right foot swings while the left foot supports |

## Robot-Specific Notes

- Forward motion is negative X in the current robot frame, so forward
  `x_move_amplitude` values are negative.
- The hip spacing is wider than many small humanoid examples, so the default
  lateral shift is intentionally large.
- `JOINT_AXIS_DIR` maps analytical IK output into the joint axes exported in the
  robot URDF.

## Run

```bash
ros2 run biped_bike_robot ik_walker.py
```

Useful hardware-tested parameters:

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p num_cycles:=1 \
  -p support_hip_roll_lift_deg:=20.0 \
  -p support_ankle_roll_lift_deg:=10.0 \
  -p support_ankle_roll_lift_sign:=1.0 \
  -p pelvis_pitch_forward_lift_deg:=30.0 \
  -p pelvis_pitch_forward_lift_sign:=1.0 \
  -p trajectory_time_scale:=4.0
```

## References

This controller was designed while studying open-source humanoid gait
controllers and analytical leg IK implementations. Keep third-party source code
and license notices in the repository's reference or notice material if any
third-party code is distributed with a release.
