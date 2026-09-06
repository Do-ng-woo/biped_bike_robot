"""Deterministic sync-read/sync-write access to the OpenCR Dynamixel bus."""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path

import yaml
from dynamixel_sdk import (
  COMM_SUCCESS,
  GroupSyncRead,
  GroupSyncWrite,
  PacketHandler,
  PortHandler,
)

ADDR_TORQUE_ENABLE = 64
ADDR_OPERATING_MODE = 11
ADDR_PWM_LIMIT = 36
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124
# XL430 exposes Present Load here; XM430 exposes Present Current at the same address.
ADDR_PRESENT_EFFORT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
LEN_STATE_BLOCK = 12
ADDR_OPENCR_IMU_BLOCK = 100
LEN_OPENCR_IMU_BLOCK = 68
DXL_BROADCAST_ID = 0xFE


def _signed(value: int, bits: int = 32) -> int:
  sign_bit = 1 << (bits - 1)
  return value - (1 << bits) if value & sign_bit else value


@dataclass(frozen=True)
class Motor:
  id: int
  name: str
  center_tick: int
  direction: int
  tick_per_rad: float
  velocity_unit_rad_s: float
  operating_mode: int
  pwm_limit: int | None
  position_d_gain: int | None
  position_i_gain: int | None
  position_p_gain: int | None
  profile_acceleration: int | None
  profile_velocity: int | None
  effort_feedback: str
  current_unit_a: float
  stall_current_a: float
  def tick_to_position(self, tick: int) -> float:
    return (tick - self.center_tick) / (
      self.direction * self.tick_per_rad
    )

  def position_to_tick(self, position: float) -> int:
    tick = self.center_tick + self.direction * position * self.tick_per_rad
    return int(round(max(0.0, min(4095.0, tick))))

  def raw_to_velocity(self, raw: int) -> float:
    return self.direction * raw * self.velocity_unit_rad_s

  def raw_to_effort_ratio(self, raw: int) -> float:
    """Normalize XL load or XM current feedback by the nominal stall value."""
    if self.effort_feedback == "current":
      return self.direction * raw * self.current_unit_a / self.stall_current_a
    if self.effort_feedback == "load":
      return self.direction * raw * 0.001
    raise ValueError(
      f"Unsupported effort_feedback={self.effort_feedback!r} for {self.name}"
    )


@dataclass(frozen=True)
class ImuSample:
  time_ms: int
  quaternion_xyzw: tuple[float, float, float, float]
  angular_velocity_rad_s: tuple[float, float, float]
  linear_acceleration_m_s2: tuple[float, float, float]


class DynamixelBus:
  """Own the serial port; do not run dxl_joint_state_bridge concurrently."""

  def __init__(
    self,
    config_path: str | Path,
    imu_id: int = 200,
  ):
    with Path(config_path).open(encoding="utf-8") as stream:
      config = yaml.safe_load(stream)
    conversion = config["conversion"]
    tick_per_rad = float(conversion["tick_per_rad"])
    velocity_unit = float(conversion["velocity_unit_rad_per_sec"])
    defaults = config.get("defaults", {})
    def setting(item: dict, name: str) -> int | None:
      value = item.get(name, defaults.get(name))
      return None if value is None else int(value)

    self.motors = tuple(
      Motor(
        id=int(item["id"]),
        name=str(item["joint_name"]),
        center_tick=int(item.get("center_tick", 2048)),
        direction=int(item.get("direction", 1)),
        tick_per_rad=tick_per_rad,
        velocity_unit_rad_s=velocity_unit,
        operating_mode=int(
          item.get("operating_mode", defaults.get("position_mode", 3))
        ),
        pwm_limit=setting(item, "pwm_limit"),
        position_d_gain=setting(item, "position_d_gain"),
        position_i_gain=setting(item, "position_i_gain"),
        position_p_gain=setting(item, "position_p_gain"),
        profile_acceleration=setting(item, "profile_acceleration"),
        profile_velocity=setting(item, "profile_velocity"),
        effort_feedback=str(
          item.get("effort_feedback", defaults.get("effort_feedback", "load"))
        ),
        current_unit_a=float(
          item.get("current_unit_a", defaults.get("current_unit_a", 0.00269))
        ),
        stall_current_a=float(
          item.get("stall_current_a", defaults.get("stall_current_a", 1.0))
        ),
      )
      for item in config["joints"]
      if item.get("active", True) and item["control_mode"] == "position"
    )
    self.velocity_motors = tuple(
      (int(item["id"]), str(item["joint_name"]))
      for item in config["joints"]
      if item.get("active", True) and item["control_mode"] == "velocity"
    )
    self.motor_by_name = {motor.name: motor for motor in self.motors}
    self.imu_id = int(imu_id)
    self.port = PortHandler(str(config["bus"]["device"]))
    self.packet = PacketHandler(float(config["bus"]["protocol_version"]))
    self.baudrate = int(config["bus"]["baudrate"])
    self.reader = GroupSyncRead(
      self.port,
      self.packet,
      ADDR_PRESENT_PWM,
      LEN_STATE_BLOCK,
    )
    self.writer = GroupSyncWrite(
      self.port,
      self.packet,
      ADDR_GOAL_POSITION,
      4,
    )
    for motor in self.motors:
      if not self.reader.addParam(motor.id):
        raise RuntimeError(f"Failed to add DXL ID {motor.id} to sync reader")

  def open(self) -> None:
    if not self.port.openPort():
      raise RuntimeError(f"Cannot open Dynamixel port {self.port.port_name}")
    if not self.port.setBaudRate(self.baudrate):
      self.port.closePort()
      raise RuntimeError(f"Cannot set Dynamixel baudrate {self.baudrate}")
    # USB/OpenCR can accept open() before the TTL bridge is ready to answer.
    time.sleep(0.25)

  def verify_position_motors(self) -> None:
    """Ping every commanded motor and report missing IDs before control starts."""
    missing: list[str] = []
    for motor in self.motors:
      _model_number, result, error = self.packet.ping(self.port, motor.id)
      if result != COMM_SUCCESS or error != 0:
        missing.append(f"ID {motor.id} ({motor.name})")
    if missing:
      raise RuntimeError(
        "No Dynamixel response from: "
        + ", ".join(missing)
        + ". Check 12 V motor power, OpenCR TTL wiring, baudrate, and that the "
        "ROS hardware bridge is stopped."
      )

  def read_position_motor_tuning(self) -> dict[str, dict[str, int]]:
    """Read the active position-loop settings from every commanded motor."""
    registers = (
      ("pwm_limit", ADDR_PWM_LIMIT, 2),
      ("position_d_gain", ADDR_POSITION_D_GAIN, 2),
      ("position_i_gain", ADDR_POSITION_I_GAIN, 2),
      ("position_p_gain", ADDR_POSITION_P_GAIN, 2),
      ("profile_acceleration", ADDR_PROFILE_ACCELERATION, 4),
      ("profile_velocity", ADDR_PROFILE_VELOCITY, 4),
    )
    result: dict[str, dict[str, int]] = {}
    for motor in self.motors:
      values: dict[str, int] = {}
      for name, address, size in registers:
        reader = getattr(self.packet, f"read{size}ByteTxRx")
        value, comm_result, error = reader(self.port, motor.id, address)
        if comm_result != COMM_SUCCESS or error != 0:
          detail = (
            self.packet.getTxRxResult(comm_result)
            if comm_result != COMM_SUCCESS
            else self.packet.getRxPacketError(error)
          )
          raise RuntimeError(
            f"Failed reading {name} from DXL ID {motor.id} "
            f"({motor.name}): {detail}"
          )
        values[name] = int(value)
      result[motor.name] = values
    return result

  def close(self) -> None:
    self.port.closePort()

  def read_joint_state(self) -> tuple[dict[str, float], dict[str, float]]:
    positions, velocities, _ = self.read_joint_state_with_effort()
    return positions, velocities

  def read_joint_state_with_effort(
    self,
    attempts: int = 3,
    retry_delay_sec: float = 0.002,
  ) -> tuple[dict[str, float], dict[str, float], dict[str, float]]:
    attempts = max(1, int(attempts))
    detail = "unknown communication error"
    for attempt in range(attempts):
      result = self.reader.txRxPacket()
      unavailable = [
        motor
        for motor in self.motors
        if not self.reader.isAvailable(
          motor.id,
          ADDR_PRESENT_PWM,
          LEN_STATE_BLOCK,
        )
      ]
      if result == COMM_SUCCESS and not unavailable:
        break
      detail = (
        self.packet.getTxRxResult(result)
        if result != COMM_SUCCESS
        else "state unavailable for "
        + ", ".join(f"ID {motor.id} ({motor.name})" for motor in unavailable)
      )
      if attempt + 1 < attempts:
        clear_port = getattr(self.port, "clearPort", None)
        if clear_port is not None:
          clear_port()
        time.sleep(max(0.0, retry_delay_sec))
    else:
      raise RuntimeError(
        f"Dynamixel sync read failed after {attempts} attempts: {detail}"
      )

    positions: dict[str, float] = {}
    velocities: dict[str, float] = {}
    effort_ratios: dict[str, float] = {}
    for motor in self.motors:
      effort_raw = _signed(
        self.reader.getData(motor.id, ADDR_PRESENT_EFFORT, 2),
        bits=16,
      )
      velocity_raw = _signed(self.reader.getData(motor.id, ADDR_PRESENT_VELOCITY, 4))
      position_tick = self.reader.getData(motor.id, ADDR_PRESENT_POSITION, 4)
      positions[motor.name] = motor.tick_to_position(position_tick)
      velocities[motor.name] = motor.raw_to_velocity(velocity_raw)
      effort_ratios[motor.name] = motor.raw_to_effort_ratio(effort_raw)
    return positions, velocities, effort_ratios

  def read_imu(
    self,
    attempts: int = 3,
    retry_delay_sec: float = 0.001,
  ) -> ImuSample:
    """Read the virtual OpenCR IMU, tolerating an occasional malformed packet."""
    attempts = max(1, int(attempts))
    detail = "unknown communication error"
    data: list[int] = []
    for attempt in range(attempts):
      data, result, error = self.packet.readTxRx(
        self.port,
        self.imu_id,
        ADDR_OPENCR_IMU_BLOCK,
        LEN_OPENCR_IMU_BLOCK,
      )
      if (
        result == COMM_SUCCESS
        and error == 0
        and len(data) == LEN_OPENCR_IMU_BLOCK
      ):
        break
      if result != COMM_SUCCESS:
        detail = self.packet.getTxRxResult(result)
      elif error != 0:
        detail = self.packet.getRxPacketError(error)
      else:
        detail = (
          f"expected {LEN_OPENCR_IMU_BLOCK} bytes, received {len(data)}"
        )
      if attempt + 1 < attempts:
        # A malformed Protocol 2.0 frame can leave bytes queued for the next read.
        clear_port = getattr(self.port, "clearPort", None)
        if clear_port is not None:
          clear_port()
        time.sleep(max(0.0, retry_delay_sec))
    else:
      raise RuntimeError(
        f"OpenCR IMU read failed after {attempts} attempts: {detail}"
      )

    values = struct.unpack("<I13f6h", bytes(data))
    time_ms = int(values[0])
    qw, qx, qy, qz = values[1:5]
    gyro = tuple(math.radians(value) for value in values[8:11])
    accel = tuple(value * 9.80665 for value in values[11:14])
    return ImuSample(time_ms, (qx, qy, qz, qw), gyro, accel)

  def write_positions(self, positions: dict[str, float]) -> None:
    self.writer.clearParam()
    for name, position in positions.items():
      motor = self.motor_by_name.get(name)
      if motor is None or not math.isfinite(position):
        continue
      tick = motor.position_to_tick(position)
      payload = tick.to_bytes(4, byteorder="little", signed=False)
      if not self.writer.addParam(motor.id, list(payload)):
        raise RuntimeError(f"Failed to add DXL ID {motor.id} to sync writer")
    result = self.writer.txPacket()
    if result != COMM_SUCCESS:
      raise RuntimeError(
        f"Dynamixel sync write failed: {self.packet.getTxRxResult(result)}"
      )

  def set_torque(self, enabled: bool) -> None:
    value = 1 if enabled else 0
    for motor in self.motors:
      result, error = self.packet.write1ByteTxRx(
        self.port,
        motor.id,
        ADDR_TORQUE_ENABLE,
        value,
      )
      if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to set torque on DXL ID {motor.id}")

  def _write_setting(self, motor: Motor, address: int, size: int, value: int) -> None:
    write = {
      1: self.packet.write1ByteTxRx,
      2: self.packet.write2ByteTxRx,
      4: self.packet.write4ByteTxRx,
    }[size]
    result, error = write(self.port, motor.id, address, value)
    if result != COMM_SUCCESS or error != 0:
      detail = (
        self.packet.getTxRxResult(result)
        if result != COMM_SUCCESS
        else self.packet.getRxPacketError(error)
      )
      raise RuntimeError(
        f"Failed configuring DXL ID {motor.id} ({motor.name}) at "
        f"address {address}: {detail}"
      )

  def configure_position_motors(self) -> None:
    """Apply the same position-mode and tuning values as the ROS bridge."""
    self.set_torque(False)
    for motor in self.motors:
      self._write_setting(motor, ADDR_OPERATING_MODE, 1, motor.operating_mode)
      settings = (
        (ADDR_PWM_LIMIT, 2, motor.pwm_limit),
        (ADDR_POSITION_D_GAIN, 2, motor.position_d_gain),
        (ADDR_POSITION_I_GAIN, 2, motor.position_i_gain),
        (ADDR_POSITION_P_GAIN, 2, motor.position_p_gain),
        (ADDR_PROFILE_ACCELERATION, 4, motor.profile_acceleration),
        (ADDR_PROFILE_VELOCITY, 4, motor.profile_velocity),
      )
      for address, size, value in settings:
        if value is not None:
          self._write_setting(motor, address, size, value)

  def enable_torque_holding_current_pose(self) -> None:
    positions, _ = self.read_joint_state()
    self.write_positions(positions)
    self.set_torque(True)

  def stop_velocity_motors(self, disable_torque: bool = True) -> None:
    for motor_id, name in self.velocity_motors:
      result, error = self.packet.write4ByteTxRx(
        self.port,
        motor_id,
        104,
        0,
      )
      if result != COMM_SUCCESS or error != 0:
        raise RuntimeError(f"Failed to stop velocity motor ID {motor_id} ({name})")
      if disable_torque:
        result, error = self.packet.write1ByteTxRx(
          self.port,
          motor_id,
          ADDR_TORQUE_ENABLE,
          0,
        )
        if result != COMM_SUCCESS or error != 0:
          raise RuntimeError(f"Failed to disable velocity motor ID {motor_id} ({name})")

  def safe_shutdown(self) -> None:
    """Stop wheels and release every motor without requiring status packets."""
    errors: list[str] = []
    velocity_stopped = False
    torque_disabled = False

    # Broadcast writes intentionally have no status response. Repeating them is
    # safer at shutdown than aborting all remaining motors on one missed reply.
    for _ in range(3):
      velocity_result, _ = self.packet.write4ByteTxRx(
        self.port,
        DXL_BROADCAST_ID,
        104,
        0,
      )
      torque_result, _ = self.packet.write1ByteTxRx(
        self.port,
        DXL_BROADCAST_ID,
        ADDR_TORQUE_ENABLE,
        0,
      )
      velocity_stopped = velocity_stopped or velocity_result == COMM_SUCCESS
      torque_disabled = torque_disabled or torque_result == COMM_SUCCESS
      time.sleep(0.003)

    if not velocity_stopped:
      failed_velocity_ids: list[int] = []
      for motor_id, _name in self.velocity_motors:
        success = False
        for _ in range(3):
          result, error = self.packet.write4ByteTxRx(
            self.port,
            motor_id,
            104,
            0,
          )
          if result == COMM_SUCCESS and error == 0:
            success = True
            break
          time.sleep(0.003)
        if not success:
          failed_velocity_ids.append(motor_id)
      if failed_velocity_ids:
        errors.append(f"wheel stop failed for IDs {failed_velocity_ids}")

    if not torque_disabled:
      all_motor_ids = [motor.id for motor in self.motors]
      all_motor_ids.extend(motor_id for motor_id, _name in self.velocity_motors)
      failed_torque_ids: list[int] = []
      for motor_id in all_motor_ids:
        success = False
        for _ in range(3):
          result, error = self.packet.write1ByteTxRx(
            self.port,
            motor_id,
            ADDR_TORQUE_ENABLE,
            0,
          )
          if result == COMM_SUCCESS and error == 0:
            success = True
            break
          time.sleep(0.003)
        if not success:
          failed_torque_ids.append(motor_id)
      if failed_torque_ids:
        errors.append(f"torque disable failed for IDs {failed_torque_ids}")

    if errors:
      raise RuntimeError("; ".join(errors))
