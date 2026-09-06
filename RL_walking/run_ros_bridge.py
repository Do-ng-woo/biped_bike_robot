#!/usr/bin/env python3
"""Own the OpenCR bus and expose the 50 Hz RL hardware interface over ROS 2."""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

import rclpy
import yaml
from dynamixel_io import DynamixelBus
from rclpy.node import Node
from robot_contract import HARDWARE_READY_TARGET, TARGET_JOINTS
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool


class RlHardwareBridge(Node):
  def __init__(self, config: dict):
    super().__init__("biped_rl_hardware_bridge")
    control = config["control"]
    imu_cfg = config["imu"]
    self.bus = DynamixelBus(
      config["hardware"]["config_path"],
      imu_cfg["virtual_dxl_id"],
    )
    self.bus.open()
    self.bus.verify_position_motors()
    torque_cfg = config.get("torque_estimation", {})
    default_stall_torque = float(
      torque_cfg.get("default_stall_torque_nm", 1.4)
    )
    torque_overrides = torque_cfg.get("joint_stall_torque_nm", {})
    self.stall_torque_nm = {
      name: float(torque_overrides.get(name, default_stall_torque))
      for name in TARGET_JOINTS
    }
    self.enabled = False
    self.last_command_time: float | None = None
    self.command_timeout = float(control["command_timeout_sec"])
    self.ready_time = float(control["ready_time_sec"])
    self.return_time = float(
      control.get("return_to_ready_time_sec", self.ready_time)
    )
    self.control_dt = 1.0 / float(control["rate_hz"])
    self.pending_targets: dict[str, float] | None = None
    self.latest_positions: dict[str, float] | None = None
    self.latest_velocities: dict[str, float] | None = None
    self.latest_effort_ratios: dict[str, float] | None = None
    self.policy_active = False
    self.returning_to_ready = False
    self.return_started = 0.0
    self.return_start: dict[str, float] | None = None
    self.imu_read_attempts = int(imu_cfg.get("read_attempts", 3))
    self.imu_retry_delay = float(imu_cfg.get("retry_delay_sec", 0.001))
    self.imu_max_failures = int(
      imu_cfg.get("maximum_consecutive_failures", 10)
    )
    self.imu_recovery_samples = int(imu_cfg.get("recovery_samples", 10))
    self.latest_imu_sample = None
    self.imu_consecutive_failures = 0
    self.imu_consecutive_recoveries = 0
    self.imu_faulted = False
    self.last_imu_warning_time = -math.inf
    hardware_cfg = config["hardware"]
    self.joint_read_attempts = int(hardware_cfg.get("state_read_attempts", 3))
    self.joint_retry_delay = float(
      hardware_cfg.get("state_retry_delay_sec", 0.002)
    )
    self.joint_max_failures = int(
      hardware_cfg.get("maximum_consecutive_state_failures", 10)
    )
    self.joint_recovery_samples = int(
      hardware_cfg.get("state_recovery_samples", 10)
    )
    self.joint_consecutive_failures = 0
    self.joint_consecutive_recoveries = 0
    self.joint_state_faulted = False
    self.last_joint_warning_time = -math.inf
    self.joint_pub = self.create_publisher(
      JointState,
      "/biped_rl/joint_states",
      1,
    )
    self.imu_pub = self.create_publisher(Imu, "/biped_rl/imu", 1)
    self.create_subscription(
      JointState,
      "/biped_rl/joint_targets",
      self.target_callback,
      1,
    )
    self.create_subscription(Bool, "/biped_rl/enable", self.enable_callback, 1)
    self.timer = self.create_timer(self.control_dt, self.cycle)
    self.get_logger().info("OpenCR ready; verified all 17 position motors")
    self.get_logger().info(
      "JointState.effort publishes an XL-load/XM-current torque estimate, not measured torque"
    )

  def move_to_ready_pose(self) -> None:
    """Enable torque at the measured pose, then smoothly assume ready posture."""
    self.bus.stop_velocity_motors(disable_torque=True)
    self.bus.configure_position_motors()
    tuning = self.bus.read_position_motor_tuning()
    for motor in self.bus.motors:
      values = tuning[motor.name]
      self.get_logger().info(
        f"DXL ID {motor.id:2d} {motor.name}: "
        f"P/I/D={values['position_p_gain']}/"
        f"{values['position_i_gain']}/{values['position_d_gain']} "
        f"PWM={values['pwm_limit']} "
        f"profile A/V={values['profile_acceleration']}/"
        f"{values['profile_velocity']}"
      )
    positions, velocities = self.bus.read_joint_state()
    self.latest_positions = positions
    self.latest_velocities = velocities
    self.latest_effort_ratios = {name: 0.0 for name in TARGET_JOINTS}
    start = [positions[name] for name in TARGET_JOINTS]
    ready = list(HARDWARE_READY_TARGET)
    self.bus.write_positions(dict(zip(TARGET_JOINTS, start, strict=True)))
    self.bus.set_torque(True)
    self.enabled = True
    self.get_logger().warn(
      f"Motor torque ENABLED; moving to ready pose over {self.ready_time:.1f}s"
    )
    started = time.monotonic()
    deadline = started
    dt = self.control_dt
    while True:
      elapsed = time.monotonic() - started
      linear = min(elapsed / max(self.ready_time, dt), 1.0)
      alpha = linear * linear * (3.0 - 2.0 * linear)
      target = {
        name: start[index] + alpha * (ready[index] - start[index])
        for index, name in enumerate(TARGET_JOINTS)
      }
      self.bus.write_positions(target)
      if linear >= 1.0:
        break
      deadline += dt
      time.sleep(max(0.0, deadline - time.monotonic()))
    self.pending_targets = dict(zip(TARGET_JOINTS, ready, strict=True))
    self.policy_active = False
    self.returning_to_ready = False
    self.last_command_time = None
    self.get_logger().warn("Ready pose reached; waiting for the policy")

  def target_callback(self, msg: JointState) -> None:
    if self.imu_faulted or self.joint_state_faulted or self.returning_to_ready:
      return
    incoming = dict(zip(msg.name, msg.position, strict=False))
    if not all(name in incoming for name in TARGET_JOINTS):
      self.get_logger().error("Rejected incomplete RL joint target")
      return
    values = {name: float(incoming[name]) for name in TARGET_JOINTS}
    if not all(math.isfinite(value) for value in values.values()):
      self.get_logger().error("Rejected non-finite RL joint target")
      return
    self.pending_targets = values
    self.last_command_time = time.monotonic()

  def enable_callback(self, msg: Bool) -> None:
    if msg.data:
      if (
        self.imu_faulted
        or self.joint_state_faulted
        or self.latest_imu_sample is None
      ):
        self.get_logger().error(
          "Refusing policy control until joint and IMU feedback are healthy"
        )
        return
      if self.pending_targets is None:
        self.get_logger().error("Refusing torque enable before the first joint target")
        return
      if not self.enabled:
        self.bus.configure_position_motors()
        self.bus.enable_torque_holding_current_pose()
        self.enabled = True
        self.get_logger().warn("RL motor torque ENABLED")
      if not self.policy_active:
        self.get_logger().warn("Policy control ENABLED")
      self.policy_active = True
      self.returning_to_ready = False
      self.return_start = None
      self.last_command_time = time.monotonic()
    elif self.policy_active:
      self.begin_return_to_ready("policy requested stop")

  def begin_return_to_ready(self, reason: str) -> None:
    """Blend from the measured pose to ready while keeping position torque on."""
    if not self.enabled or self.returning_to_ready:
      return
    if self.latest_positions is None:
      positions, _ = self.bus.read_joint_state()
      self.latest_positions = positions
    self.return_start = {
      name: self.latest_positions[name] for name in TARGET_JOINTS
    }
    self.pending_targets = dict(self.return_start)
    self.return_started = time.monotonic()
    self.returning_to_ready = True
    self.policy_active = False
    self.last_command_time = None
    self.get_logger().warn(
      f"{reason}; returning to ready pose over {self.return_time:.1f}s"
    )

  def disable_torque(self, reason: str) -> None:
    try:
      self.bus.safe_shutdown()
    finally:
      self.enabled = False
      self.get_logger().warn(f"Motor torque DISABLED: {reason}")

  def cycle(self) -> None:
    now = time.monotonic()
    if (
      self.enabled
      and self.policy_active
      and self.last_command_time is not None
      and now - self.last_command_time > self.command_timeout
    ):
      self.begin_return_to_ready("policy command watchdog timeout")

    try:
      positions, velocities, effort_ratios = self.bus.read_joint_state_with_effort(
        attempts=self.joint_read_attempts,
        retry_delay_sec=self.joint_retry_delay,
      )
    except RuntimeError as exc:
      self.joint_consecutive_failures += 1
      self.joint_consecutive_recoveries = 0
      if now - self.last_joint_warning_time >= 1.0:
        self.get_logger().warn(
          "Transient Dynamixel state read failure; using the last valid state "
          f"({self.joint_consecutive_failures}/{self.joint_max_failures}): {exc}"
        )
        self.last_joint_warning_time = now
      if (
        self.joint_consecutive_failures >= self.joint_max_failures
        and not self.joint_state_faulted
      ):
        self.joint_state_faulted = True
        self.get_logger().error(
          "Dynamixel state feedback fault persisted; policy control is inhibited"
        )
        self.begin_return_to_ready("persistent Dynamixel state read failure")
      if (
        self.latest_positions is None
        or self.latest_velocities is None
        or self.latest_effort_ratios is None
      ):
        return
      positions = self.latest_positions
      velocities = self.latest_velocities
      effort_ratios = self.latest_effort_ratios
    else:
      self.latest_positions = positions
      self.latest_velocities = velocities
      self.latest_effort_ratios = effort_ratios
      if self.joint_state_faulted:
        self.joint_consecutive_recoveries += 1
        if self.joint_consecutive_recoveries >= self.joint_recovery_samples:
          self.joint_state_faulted = False
          self.joint_consecutive_failures = 0
          self.joint_consecutive_recoveries = 0
          self.get_logger().warn(
            "Dynamixel state feedback recovered; policy may be started again"
          )
      else:
        if self.joint_consecutive_failures > 0:
          self.get_logger().info(
            "Dynamixel state feedback recovered after "
            f"{self.joint_consecutive_failures} failed cycle(s)"
          )
        self.joint_consecutive_failures = 0
    sample = None
    try:
      sample = self.bus.read_imu(
        attempts=self.imu_read_attempts,
        retry_delay_sec=self.imu_retry_delay,
      )
    except RuntimeError as exc:
      self.imu_consecutive_failures += 1
      self.imu_consecutive_recoveries = 0
      if now - self.last_imu_warning_time >= 1.0:
        self.get_logger().warn(
          "Transient OpenCR IMU read failure; using the last valid sample "
          f"({self.imu_consecutive_failures}/{self.imu_max_failures}): {exc}"
        )
        self.last_imu_warning_time = now
      if (
        self.imu_consecutive_failures >= self.imu_max_failures
        and not self.imu_faulted
      ):
        self.imu_faulted = True
        self.get_logger().error(
          "OpenCR IMU fault persisted; policy control is inhibited"
        )
        self.begin_return_to_ready("persistent OpenCR IMU read failure")
      sample = self.latest_imu_sample
    else:
      self.latest_imu_sample = sample
      if self.imu_faulted:
        self.imu_consecutive_recoveries += 1
        if self.imu_consecutive_recoveries >= self.imu_recovery_samples:
          self.imu_faulted = False
          self.imu_consecutive_failures = 0
          self.imu_consecutive_recoveries = 0
          self.get_logger().warn(
            "OpenCR IMU communication recovered; policy may be started again"
          )
      else:
        if self.imu_consecutive_failures > 0:
          self.get_logger().info(
            "OpenCR IMU communication recovered after "
            f"{self.imu_consecutive_failures} failed cycle(s)"
          )
        self.imu_consecutive_failures = 0
    stamp = self.get_clock().now().to_msg()

    joints = JointState()
    joints.header.stamp = stamp
    joints.name = list(TARGET_JOINTS)
    joints.position = [positions[name] for name in TARGET_JOINTS]
    joints.velocity = [velocities[name] for name in TARGET_JOINTS]
    joints.effort = [
      effort_ratios[name] * self.stall_torque_nm[name] for name in TARGET_JOINTS
    ]
    self.joint_pub.publish(joints)

    if sample is not None:
      imu = Imu()
      imu.header.stamp = stamp
      imu.header.frame_id = "opencr_imu"
      qx, qy, qz, qw = sample.quaternion_xyzw
      imu.orientation.x = qx
      imu.orientation.y = qy
      imu.orientation.z = qz
      imu.orientation.w = qw
      imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z = (
        sample.angular_velocity_rad_s
      )
      (
        imu.linear_acceleration.x,
        imu.linear_acceleration.y,
        imu.linear_acceleration.z,
      ) = sample.linear_acceleration_m_s2
      self.imu_pub.publish(imu)

    if self.enabled and self.returning_to_ready:
      assert self.return_start is not None
      elapsed = now - self.return_started
      linear = min(elapsed / max(self.return_time, self.control_dt), 1.0)
      alpha = linear * linear * (3.0 - 2.0 * linear)
      ready = HARDWARE_READY_TARGET
      self.pending_targets = {
        name: self.return_start[name]
        + alpha * (ready[index] - self.return_start[name])
        for index, name in enumerate(TARGET_JOINTS)
      }
      self.bus.write_positions(self.pending_targets)
      if linear >= 1.0:
        self.returning_to_ready = False
        self.return_start = None
        self.pending_targets = dict(zip(TARGET_JOINTS, ready, strict=True))
        self.get_logger().warn(
          "Ready pose restored; motor torque remains enabled while bridge waits"
        )
    elif self.enabled and self.pending_targets is not None:
      self.bus.write_positions(self.pending_targets)

  def close(self) -> None:
    error: RuntimeError | None = None
    try:
      self.bus.safe_shutdown()
    except RuntimeError as exc:
      error = exc
    finally:
      self.enabled = False
      self.policy_active = False
      self.bus.close()

    message = (
      "Bridge stopped; motor torque DISABLED"
      if error is None
      else f"Shutdown motor stop failed after all retries: {error}"
    )
    if rclpy.ok():
      if error is None:
        self.get_logger().warn(message)
      else:
        self.get_logger().error(message)
    else:
      print(message, file=sys.stderr)


def main() -> None:
  folder = Path(__file__).resolve().parent
  parser = argparse.ArgumentParser()
  parser.add_argument("--config", type=Path, default=folder / "config.yaml")
  args = parser.parse_args()
  with args.config.open(encoding="utf-8") as stream:
    config = yaml.safe_load(stream)
  hardware_path = Path(config["hardware"]["config_path"])
  if not hardware_path.is_absolute():
    config["hardware"]["config_path"] = str(
      (args.config.resolve().parent / hardware_path).resolve()
    )
  rclpy.init()
  node = RlHardwareBridge(config)
  try:
    node.move_to_ready_pose()
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass
  finally:
    node.close()
    node.destroy_node()
    if rclpy.ok():
      rclpy.shutdown()


if __name__ == "__main__":
  main()
