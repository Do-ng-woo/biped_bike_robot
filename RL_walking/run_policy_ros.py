#!/usr/bin/env python3
"""Run a history-based current-G1-style 225D/12D policy over the ROS bridge."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool

PIPELINE_DIR = Path(__file__).resolve().parent
SENSORS_DIR = PIPELINE_DIR

from imu import ImuTransform, quaternion_to_matrix  # noqa: E402
from policy_logging import PolicyRunLogger  # noqa: E402

try:
  from .checkpoint import resolve_policy_path
  from .contract import (
    DEFAULT_FORWARD_SPEED,
    DEFAULT_LATERAL_SPEED,
    DEFAULT_YAW_RATE,
    LOWER_BODY_JOINTS,
    OBSERVATION_SLICES,
    POLICY_DT,
    READY_TARGET,
    TARGET_JOINTS,
  )
  from .control_ui import PolicyControlServer
  from .controller import FreeWalkingController
  from .observation import ObservationHistory
  from .policy import NumpyPolicy
except ImportError:
  from checkpoint import resolve_policy_path
  from contract import (
    DEFAULT_FORWARD_SPEED,
    DEFAULT_LATERAL_SPEED,
    DEFAULT_YAW_RATE,
    LOWER_BODY_JOINTS,
    OBSERVATION_SLICES,
    POLICY_DT,
    READY_TARGET,
    TARGET_JOINTS,
  )
  from control_ui import PolicyControlServer
  from controller import FreeWalkingController
  from observation import ObservationHistory
  from policy import NumpyPolicy


class FreeWalkingPolicyRunner(Node):
  def __init__(
    self,
    policy_path: Path,
    config: dict,
    duration: float,
    log_dir: Path,
    forward_speed: float,
    lateral_speed: float,
    yaw_rate: float,
    action_filter_alpha: float,
    interactive_control: bool = False,
  ) -> None:
    super().__init__("biped_free_walking_policy")
    self.policy = NumpyPolicy(policy_path)
    if tuple(self.policy.actor_joint_names) != LOWER_BODY_JOINTS:
      raise RuntimeError("Free-walking actor joint order does not match runtime")
    if tuple(self.policy.target_joint_names) != TARGET_JOINTS:
      raise RuntimeError("Free-walking target joint order does not match runtime")
    control = config["control"]
    self.dt = 1.0 / float(control["rate_hz"])
    if not math.isclose(self.dt, POLICY_DT, abs_tol=1.0e-6):
      raise RuntimeError("Training and hardware control periods differ")
    if not math.isclose(self.policy.policy_dt, self.dt, abs_tol=1.0e-6):
      raise RuntimeError("Exported policy period does not match hardware")
    imu_cfg = config["imu"]
    self.imu_transform = ImuTransform(
      float(imu_cfg["mount_roll_deg"]),
      float(imu_cfg["mount_pitch_deg"]),
      float(imu_cfg["mount_yaw_deg"]),
      str(imu_cfg["rpy_remap"]),
    )
    self.controller = FreeWalkingController(action_filter_alpha)
    self.observation_history = ObservationHistory()
    self.forward_speed = float(forward_speed)
    self.lateral_speed = float(lateral_speed)
    self.yaw_rate = float(yaw_rate)
    initial_command = (0.0, 0.0, 0.0) if interactive_control else (
      self.forward_speed,
      self.lateral_speed,
      self.yaw_rate,
    )
    self.command = np.asarray(initial_command, np.float32)
    self.blend_time = float(control["walking_ready_time_sec"])
    self.ready_settle_time = float(
      control.get("walking_ready_settle_time_sec", 1.0)
    )
    self.ready_max_tilt_deg = float(
      control.get("walking_ready_max_tilt_deg", 10.0)
    )
    self.ready_level_warning_deg = float(
      control.get("walking_ready_level_warning_deg", 6.0)
    )
    self.hip_tracking_integral_gain = float(
      control.get("hip_pitch_tracking_integral_gain", 1.5)
    )
    self.hip_tracking_correction_limit = math.radians(
      float(control.get("hip_pitch_tracking_correction_max_deg", 8.0))
    )
    self.hip_pitch_indices = tuple(
      TARGET_JOINTS.index(name)
      for name in ("l_hip_pitch_jnt", "r_hip_pitch_jnt")
    )
    self.hip_tracking_correction = np.zeros(len(self.hip_pitch_indices), np.float32)
    self.maximum_tilt_cos = math.cos(
      math.radians(float(control["maximum_tilt_deg"]))
    )
    self.maximum_z = float(control["maximum_normalized_observation"])
    self.maximum_step = float(control["maximum_target_step_rad"])
    self.required_gyro_samples = int(control["gyro_bias_samples"])
    self.limits = config["joint_limits"]
    self.duration = float(duration)
    self.positions: dict[str, float] = {}
    self.velocities: dict[str, float] = {}
    self.efforts: dict[str, float] = {}
    self.accelerations: dict[str, float] = {}
    self.previous_velocities: dict[str, float] = {}
    self.previous_velocity_time: float | None = None
    self.gravity: np.ndarray | None = None
    self.gyro: np.ndarray | None = None
    self.heading: float | None = None
    self.heading_zero: float | None = None
    self.gyro_samples: list[np.ndarray] = []
    self.gravity_samples: list[np.ndarray] = []
    self.gyro_bias: np.ndarray | None = None
    self.last_joint_time = 0.0
    self.last_imu_time = 0.0
    self.started: float | None = None
    self.policy_started = False
    self.policy_started_time: float | None = None
    self.initial_target: np.ndarray | None = None
    self.previous_target: np.ndarray | None = None
    self.cycle_count = 0
    self.finished = False
    self.run_logger = PolicyRunLogger(
      log_dir, ("command/vx", "command/vy", "command/yaw")
    )
    self.target_pub = self.create_publisher(JointState, "/biped_rl/joint_targets", 1)
    self.enable_pub = self.create_publisher(Bool, "/biped_rl/enable", 1)
    self.create_subscription(
      JointState, "/biped_rl/joint_states", self.joint_callback, 1
    )
    self.create_subscription(Imu, "/biped_rl/imu", self.imu_callback, 1)
    self.timer = self.create_timer(self.dt, self.control_cycle)
    if interactive_control:
      command_description = (
        "interactive command starts stopped; "
        f"forward={self.forward_speed:.3f} m/s, "
        f"crab=+/-{self.lateral_speed:.3f} m/s, "
        f"turn=+/-{self.yaw_rate:.3f} rad/s"
      )
    else:
      command_description = (
        f"command=({self.forward_speed:.3f}, {self.lateral_speed:.3f}) m/s, "
        f"yaw={self.yaw_rate:.3f} rad/s"
      )
    self.get_logger().info(
      f"Waiting for joint/IMU data; {command_description}, "
      f"action filter alpha={action_filter_alpha:.2f}"
    )

  def set_drive_command(self, drive_command: str) -> None:
    commands = {
      "stop": (0.0, 0.0, 0.0),
      "forward": (self.forward_speed, 0.0, 0.0),
      "left": (0.0, self.lateral_speed, 0.0),
      "right": (0.0, -self.lateral_speed, 0.0),
      "ccw": (0.0, 0.0, self.yaw_rate),
      "cw": (0.0, 0.0, -self.yaw_rate),
    }
    command = np.asarray(commands[drive_command], dtype=np.float32)
    if not np.array_equal(command, self.command):
      self.command[:] = command
      self.get_logger().info(
        f"Drive command {drive_command}: "
        f"vx={command[0]:.3f}, vy={command[1]:.3f}, yaw={command[2]:.3f}"
      )

  def joint_callback(self, msg: JointState) -> None:
    now = time.monotonic()
    incoming_pos = dict(zip(msg.name, msg.position, strict=False))
    incoming_vel = dict(zip(msg.name, msg.velocity, strict=False))
    incoming_effort = dict(zip(msg.name, msg.effort, strict=False))
    if not all(name in incoming_pos for name in TARGET_JOINTS):
      return
    dt = None if self.previous_velocity_time is None else now - self.previous_velocity_time
    for name in TARGET_JOINTS:
      self.positions[name] = float(incoming_pos[name])
      velocity = float(incoming_vel.get(name, 0.0))
      self.velocities[name] = velocity
      self.efforts[name] = float(incoming_effort.get(name, 0.0))
      previous = self.previous_velocities.get(name)
      raw_accel = (
        0.0
        if previous is None or dt is None or dt <= 1.0e-4
        else (velocity - previous) / dt
      )
      old = self.accelerations.get(name, 0.0)
      self.accelerations[name] = old + 0.2 * (raw_accel - old)
    self.previous_velocities = dict(self.velocities)
    self.previous_velocity_time = now
    self.last_joint_time = now

  def imu_callback(self, msg: Imu) -> None:
    quaternion = np.asarray(
      (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    )
    raw_gyro = np.asarray(
      (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
    )
    orientation = self.imu_transform.orientation(quaternion)
    orientation_matrix = quaternion_to_matrix(orientation)
    self.heading = math.atan2(orientation_matrix[1, 0], orientation_matrix[0, 0])
    self.gravity = self.imu_transform.projected_gravity(quaternion)
    self.gyro = self.imu_transform.angular_velocity(raw_gyro)
    self.last_imu_time = time.monotonic()
    if self.gyro_bias is None:
      self.gyro_samples.append(self.gyro.copy())
      self.gravity_samples.append(self.gravity.copy())
      if len(self.gyro_samples) >= self.required_gyro_samples:
        gyro_samples = np.asarray(self.gyro_samples, dtype=np.float32)
        gravity_samples = np.asarray(self.gravity_samples, dtype=np.float32)
        self.gyro_bias = np.mean(gyro_samples, axis=0).astype(np.float32)
        gyro_noise = np.std(gyro_samples, axis=0)
        gravity_noise = np.std(gravity_samples, axis=0)
        tilt_noise_deg = math.degrees(float(np.linalg.norm(gravity_noise[:2])))
        self.get_logger().info(f"Gyro bias rad/s: {self.gyro_bias.tolist()}")
        self.get_logger().info(
          "Stationary IMU noise over "
          f"{self.required_gyro_samples} samples: "
          f"gyro_std={gyro_noise.tolist()} rad/s, "
          f"gravity_std={gravity_noise.tolist()}, "
          f"tilt_std~{tilt_noise_deg:.3f}deg"
        )

  def _publish_enable(self, enabled: bool) -> None:
    msg = Bool()
    msg.data = enabled
    self.enable_pub.publish(msg)

  def _publish_target(self, target: np.ndarray) -> None:
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = list(TARGET_JOINTS)
    msg.position = [float(value) for value in target]
    self.target_pub.publish(msg)

  def _limit_target(self, target: np.ndarray) -> tuple[np.ndarray, bool]:
    assert self.previous_target is not None
    limited = np.clip(
      target,
      self.previous_target - self.maximum_step,
      self.previous_target + self.maximum_step,
    )
    clipped = not np.array_equal(limited, target)
    for index, name in enumerate(TARGET_JOINTS):
      lower, upper = self.limits[name]
      value = float(np.clip(limited[index], lower, upper))
      clipped = clipped or value != limited[index]
      limited[index] = value
    return limited.astype(np.float32), clipped

  def control_cycle(self) -> None:
    if self.finished:
      return
    now = time.monotonic()
    if not (
      self.positions
      and self.gravity is not None
      and self.gyro is not None
      and self.heading is not None
      and self.gyro_bias is not None
    ):
      return
    if now - self.last_joint_time > 0.10 or now - self.last_imu_time > 0.10:
      self.request_stop("sensor timeout")
      return
    if self.started is None:
      self.started = now
      self.initial_target = np.asarray(
        [self.positions[name] for name in TARGET_JOINTS], np.float32
      )
      self.previous_target = self.initial_target.copy()
      self.heading_zero = self.heading
      self.get_logger().warn(
        f"Taking over ready pose with {self.blend_time:.1f}-second blend"
      )
    assert self.started is not None
    assert self.initial_target is not None
    assert self.previous_target is not None
    assert self.gravity is not None
    assert self.gyro is not None
    assert self.heading is not None
    assert self.heading_zero is not None
    assert self.gyro_bias is not None
    elapsed = now - self.started
    tilt_deg = math.degrees(
      math.acos(float(np.clip(-self.gravity[2], -1.0, 1.0)))
    )
    if self.gravity[2] > -self.maximum_tilt_cos:
      self.request_stop(f"tilt safety stop: {tilt_deg:.1f} deg")
      return

    blend_linear = float(
      np.clip(elapsed / max(self.blend_time, self.dt), 0.0, 1.0)
    )
    blend = blend_linear * blend_linear * (3.0 - 2.0 * blend_linear)
    readying = elapsed < self.blend_time + self.ready_settle_time
    if readying:
      ready_target = np.asarray(READY_TARGET, dtype=np.float32)
      target = (1.0 - blend) * self.initial_target + blend * ready_target
      if blend_linear >= 1.0:
        actual = np.asarray(
          [self.positions[name] for name in TARGET_JOINTS], np.float32
        )
        hip_error = ready_target[list(self.hip_pitch_indices)] - actual[
          list(self.hip_pitch_indices)
        ]
        self.hip_tracking_correction += (
          self.hip_tracking_integral_gain * self.dt * hip_error
        )
        np.clip(
          self.hip_tracking_correction,
          -self.hip_tracking_correction_limit,
          self.hip_tracking_correction_limit,
          out=self.hip_tracking_correction,
        )
      target[list(self.hip_pitch_indices)] += self.hip_tracking_correction
      normalized_max = 0.0
      term_zmax = {name: 0.0 for name in OBSERVATION_SLICES}
    else:
      if not self.policy_started:
        if tilt_deg > self.ready_max_tilt_deg:
          self.request_stop(
            f"ready pose is not level: tilt={tilt_deg:.1f}deg exceeds "
            f"{self.ready_max_tilt_deg:.1f}deg"
          )
          return
        if tilt_deg > self.ready_level_warning_deg:
          self.get_logger().warn(
            f"ready pose tilt remains {tilt_deg:.1f}deg; continuing below "
            f"the {self.ready_max_tilt_deg:.1f}deg safety limit"
          )
        self.controller.reset()
        self.observation_history.reset()
        self.policy_started = True
        self.policy_started_time = now
        self.get_logger().warn(
          "FreeWalking ready pose reached; observation history reset; "
          "policy started; hip pitch correction="
          f"{np.degrees(self.hip_tracking_correction).tolist()}deg"
        )
      observation = self.observation_history.append(
        self.positions,
        self.velocities,
        self.gravity,
        self.gyro - self.gyro_bias,
        self.command,
        self.controller.raw_action,
      )
      normalized_observation = self.policy.normalized_observation(observation)
      normalized_abs = np.abs(normalized_observation)
      normalized_max = float(np.max(normalized_abs))
      term_zmax = {
        name: float(np.max(normalized_abs[start:end]))
        for name, (start, end) in OBSERVATION_SLICES.items()
      }
      if normalized_max > self.maximum_z:
        self.request_stop(f"observation safety stop: z={normalized_max:.1f}")
        return
      target = self.controller.step(self.policy(observation))
      target[list(self.hip_pitch_indices)] += self.hip_tracking_correction
    target, clipped = self._limit_target(target)
    self._publish_target(target)
    self._publish_enable(True)
    self.previous_target = target
    actual = np.asarray([self.positions[name] for name in TARGET_JOINTS], np.float32)
    velocity = np.asarray([self.velocities[name] for name in TARGET_JOINTS], np.float32)
    acceleration = np.asarray(
      [self.accelerations.get(name, 0.0) for name in TARGET_JOINTS], np.float32
    )
    torque = np.asarray(
      [self.efforts.get(name, 0.0) for name in TARGET_JOINTS], np.float32
    )
    self.run_logger.record(
      elapsed,
      0.0,
      tilt_deg,
      normalized_max,
      target,
      actual,
      velocity,
      acceleration,
      torque,
      extra_values=self.command,
    )
    if self.cycle_count % 50 == 0:
      self.get_logger().info(
        f"cycle={self.cycle_count} blend={blend:.2f} tilt={tilt_deg:.1f}deg "
        f"obs_zmax={normalized_max:.2f} "
        f"z(gyro/gravity/joint_pos/joint_vel/action)="
        f"{term_zmax['base_ang_vel']:.1f}/{term_zmax['projected_gravity']:.1f}/"
        f"{term_zmax['joint_pos']:.1f}/{term_zmax['joint_vel']:.1f}/"
        f"{term_zmax['actions']:.1f} gravity={np.round(self.gravity, 3).tolist()} "
        f"clipped={clipped}"
      )
    self.cycle_count += 1
    policy_elapsed = (
      0.0
      if self.policy_started_time is None
      else now - self.policy_started_time
    )
    if self.duration > 0.0 and policy_elapsed >= self.duration:
      self.request_stop("requested duration complete")

  def request_stop(self, reason: str) -> None:
    if self.finished:
      return
    self.get_logger().warn(f"Stopping policy: {reason}")
    self._publish_enable(False)
    self.finished = True

  def close(self) -> None:
    self._publish_enable(False)
    csv_path, plot_path = self.run_logger.close()
    self.get_logger().info(f"Policy CSV saved: {csv_path}")
    if plot_path is not None:
      self.get_logger().info(f"Policy overlay plot saved: {plot_path}")


def main() -> None:
  folder = Path(__file__).resolve().parent
  parser = argparse.ArgumentParser()
  parser.add_argument("--policy", required=True, type=Path)
  parser.add_argument("--config", type=Path, default=SENSORS_DIR / "config.yaml")
  parser.add_argument("--duration", type=float, default=10.0)
  parser.add_argument("--forward-speed", type=float, default=DEFAULT_FORWARD_SPEED)
  parser.add_argument("--lateral-speed", type=float, default=DEFAULT_LATERAL_SPEED)
  parser.add_argument("--yaw-rate", type=float, default=DEFAULT_YAW_RATE)
  parser.add_argument(
    "--control-ui",
    action="store_true",
    help="Start stopped and serve hold-to-drive controls on localhost",
  )
  parser.add_argument("--control-port", type=int, default=8081)
  parser.add_argument(
    "--action-filter-alpha",
    type=float,
    default=0.10,
    help="EMA coefficient for hardware actions; 1.0 disables filtering",
  )
  parser.add_argument("--arm", action="store_true")
  parser.add_argument("--log-dir", type=Path, default=folder / "logs")
  args = parser.parse_args()
  if not args.arm:
    raise SystemExit("Refusing hardware policy: add --arm after supporting the robot")
  if not 0.0 <= args.forward_speed <= 0.12:
    raise SystemExit("--forward-speed must be in [0.0, 0.12] m/s")
  if not -0.06 <= args.lateral_speed <= 0.06:
    raise SystemExit("--lateral-speed must be in [-0.06, 0.06] m/s")
  if not -0.35 <= args.yaw_rate <= 0.35:
    raise SystemExit("--yaw-rate must be in [-0.35, 0.35] rad/s")
  if args.control_ui and not 0.0 < abs(args.yaw_rate) <= 0.35:
    raise SystemExit("--control-ui requires a non-zero --yaw-rate")
  if args.control_ui and not 0.0 < args.lateral_speed <= 0.06:
    raise SystemExit(
      "--control-ui requires --lateral-speed in (0.0, 0.06] for crab controls"
    )
  if not 0.0 < args.action_filter_alpha <= 1.0:
    raise SystemExit("--action-filter-alpha must be in (0.0, 1.0]")
  policy_path = resolve_policy_path(args.policy)
  with args.config.open(encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
  rclpy.init()
  node = FreeWalkingPolicyRunner(
    policy_path,
    config,
    args.duration,
    args.log_dir,
    args.forward_speed,
    args.lateral_speed,
    args.yaw_rate,
    args.action_filter_alpha,
    args.control_ui,
  )
  control_server = None
  if args.control_ui:
    try:
      control_server = PolicyControlServer(args.control_port, node.set_drive_command)
    except OSError as exc:
      node.destroy_node()
      if rclpy.ok():
        rclpy.shutdown()
      if exc.errno == 98:
        raise SystemExit(
          f"Control UI port {args.control_port} is already in use. "
          "A previous hardware policy may still be running; stop it before "
          "starting another one. If the port belongs to an unrelated program, "
          f"choose another port with --control-port {args.control_port + 1}."
        ) from exc
      raise
    control_server.start()
    node.get_logger().warn(
      f"Hardware controls: http://localhost:{control_server.port} "
      "(W/Up=forward, A/Left and D/Right=crab, Q/E=turn, Space=stop)"
    )
  try:
    while rclpy.ok() and not node.finished:
      rclpy.spin_once(node, timeout_sec=0.1)
  except KeyboardInterrupt:
    node.request_stop("Ctrl+C")
  finally:
    if control_server is not None:
      control_server.close()
    node.close()
    for _ in range(3):
      rclpy.spin_once(node, timeout_sec=0.02)
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()
