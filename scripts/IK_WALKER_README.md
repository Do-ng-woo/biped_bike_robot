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
| `WalkingParam` | Physical robot dimensions, stance, stride, timing, and lift parameters |
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

The hardware-tested gait is the default profile inside `WalkingParam`, so the
web controller and direct execution use the same values. Only run-specific
settings such as the number of cycles need to be supplied:

```bash
ros2 run biped_bike_robot ik_walker.py --ros-args \
  -p num_cycles:=1
```

The gait does not add hardware-only hip, ankle, or pelvis corrections after
IK. Foot spacing, center-of-mass motion, and swing height are expressed as
Cartesian foot targets and solved by the same IK path.

The analytical IK and lateral `y_swap` path keep their original axis mapping.
After IK, an independent hardware backlash correction adds left hip `-3` /
ankle `+3` degrees and right hip `+3` / ankle `-3` degrees. Changing this
correction therefore does not change the underlying IK trajectory or foot
targets. After the last cycle, only this 3-degree correction is interpolated
out over two seconds; the base IK roll remains intact.

The default posture also adds 10 degrees of forward hip pitch after IK (left
`-10`, right `+10` degrees in hardware command coordinates). This corrects the
backward body lean without changing the foot targets or ankle pitch trajectory.

## References

This controller was designed while studying open-source humanoid gait
controllers and analytical leg IK implementations. Keep third-party source code
and license notices in the repository's reference or notice material if any
third-party code is distributed with a release.
