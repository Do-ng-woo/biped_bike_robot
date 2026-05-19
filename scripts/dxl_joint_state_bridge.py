#!/usr/bin/env python3
import csv
import datetime as dt
import math
import os
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory

try:
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, GroupSyncWrite, PacketHandler, PortHandler
except ImportError:
    COMM_SUCCESS = None
    GroupSyncRead = None
    GroupSyncWrite = None
    PacketHandler = None
    PortHandler = None


ADDR_OPERATING_MODE = 11
ADDR_PWM_LIMIT = 36
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_PWM = 124
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_INPUT_VOLTAGE = 144
ADDR_PRESENT_TEMPERATURE = 146
LEN_GOAL_POSITION = 4
LEN_TELEMETRY_BLOCK = 23
TORQUE_DISABLE = 0
TORQUE_ENABLE = 1


class MotorConfig:
    def __init__(self, data: dict, tick_per_rad: float):
        self.id = int(data["id"])
        self.joint_name = data["joint_name"]
        self.control_mode = data["control_mode"]
        self.operating_mode = int(data["operating_mode"])
        self.center_tick = int(data.get("center_tick", 2048))
        self.direction = int(data.get("direction", 1))
        self.active = bool(data.get("active", True))
        self.tick_per_rad = tick_per_rad
        self.pwm_limit = data.get("pwm_limit")
        self.position_p_gain = data.get("position_p_gain")
        self.position_i_gain = data.get("position_i_gain")
        self.position_d_gain = data.get("position_d_gain")
        self.profile_acceleration = data.get("profile_acceleration")
        self.profile_velocity = data.get("profile_velocity")

    def position_to_tick(self, position_rad: float) -> int:
        tick = self.center_tick + self.direction * position_rad * self.tick_per_rad
        return int(round(max(0, min(4095, tick))))


class DxlJointStateBridge(Node):
    def __init__(self):
        super().__init__("dxl_joint_state_bridge")

        self.declare_parameter("config_path", "")
        self.declare_parameter("torque_on_start", True)
        self.declare_parameter("center_on_start", False)
        self.declare_parameter("max_abs_position_rad", 0.35)
        self.declare_parameter("min_tick_change", 2)
        self.declare_parameter("log_joint_states", True)
        self.declare_parameter("enable_joint_state_commands", True)
        self.declare_parameter("enable_trajectory_commands", True)
        self.declare_parameter(
            "trajectory_topic", "/joint_trajectory_controller/joint_trajectory"
        )
        self.declare_parameter("log_telemetry", False)
        self.declare_parameter("telemetry_log_path", "")
        self.declare_parameter("telemetry_rate_hz", 5.0)
        self.declare_parameter("telemetry_duration_sec", 10.0)
        self.declare_parameter("telemetry_motor_ids", "")

        if PacketHandler is None or PortHandler is None:
            raise RuntimeError(
                "dynamixel_sdk is not installed. Install the ROS/Python Dynamixel SDK first."
            )

        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        if not config_path:
            pkg_share = get_package_share_directory("biped_bike_robot")
            config_path = os.path.join(pkg_share, "config", "dynamixel_hardware.yaml")

        self.config = self._load_config(config_path)
        self.port_handler = PortHandler(self.config["bus"]["device"])
        self.packet_handler = PacketHandler(float(self.config["bus"]["protocol_version"]))
        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        self.group_sync_read = None
        self.last_goal_ticks: Dict[int, int] = {}
        self.received_joint_states = False
        self.active_trajectory: List[Tuple[float, Dict[str, float]]] = []
        self.trajectory_start_time = None
        self.telemetry_file = None
        self.telemetry_writer = None
        self.telemetry_start_time = None
        self.telemetry_duration_sec = 0.0
        self.telemetry_motors = []

        self.max_abs_position_rad = (
            self.get_parameter("max_abs_position_rad").get_parameter_value().double_value
        )
        self.min_tick_change = (
            self.get_parameter("min_tick_change").get_parameter_value().integer_value
        )

        self.all_motors = self._load_active_motors()
        self.position_motors = [
            motor for motor in self.all_motors if motor.control_mode == "position"
        ]
        self.joint_to_motor = {motor.joint_name: motor for motor in self.position_motors}

        self._open_bus()
        self._configure_motors(
            torque_on_start=self.get_parameter("torque_on_start").value
        )
        if self.get_parameter("center_on_start").value:
            self._center_position_motors()

        if self.get_parameter("enable_joint_state_commands").value:
            self.subscription = self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                10,
            )

        if self.get_parameter("enable_trajectory_commands").value:
            trajectory_topic = (
                self.get_parameter("trajectory_topic").get_parameter_value().string_value
            )
            self.trajectory_subscription = self.create_subscription(
                JointTrajectory,
                trajectory_topic,
                self.trajectory_callback,
                10,
            )
            self.trajectory_timer = self.create_timer(0.008, self.trajectory_timer_callback)

        if self.get_parameter("log_telemetry").value:
            self._setup_telemetry_logger()

        names = ", ".join(m.joint_name for m in self.position_motors)
        self.get_logger().info(f"Configured Dynamixel position motors: {names}")

    def destroy_node(self):
        try:
            for motor in self.position_motors:
                self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        finally:
            if self.telemetry_file is not None:
                self.telemetry_file.close()
            if hasattr(self, "port_handler"):
                self.port_handler.closePort()
        super().destroy_node()

    def _load_config(self, config_path: str) -> dict:
        with open(config_path, "r", encoding="utf-8") as stream:
            config = yaml.safe_load(stream)
        self.get_logger().info(f"Loaded Dynamixel config: {config_path}")
        return config

    def _load_active_motors(self) -> List[MotorConfig]:
        tick_per_rad = float(self.config["conversion"]["tick_per_rad"])
        defaults = self.config.get("defaults", {})
        motors = []
        for joint in self.config["joints"]:
            if not joint.get("active", True):
                continue
            for key in (
                "pwm_limit",
                "position_p_gain",
                "position_i_gain",
                "position_d_gain",
                "profile_acceleration",
                "profile_velocity",
            ):
                if key not in joint:
                    joint[key] = defaults.get(key)
            motors.append(MotorConfig(joint, tick_per_rad))
        return motors

    def _open_bus(self):
        device = self.config["bus"]["device"]
        baudrate = int(self.config["bus"]["baudrate"])
        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open Dynamixel port: {device}")
        if not self.port_handler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set Dynamixel baudrate: {baudrate}")
        self.get_logger().info(f"Opened Dynamixel bus {device} @ {baudrate}")

    def _configure_motors(self, torque_on_start: bool):
        for motor in self.position_motors:
            self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
            self._write1(motor.id, ADDR_OPERATING_MODE, motor.operating_mode)
            self._apply_motor_tuning(motor)
            if torque_on_start:
                self._write1(motor.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        torque_state = "enabled" if torque_on_start else "disabled"
        self.get_logger().info(f"Configured position motors, torque {torque_state}")

    def _apply_motor_tuning(self, motor: MotorConfig):
        writes_2byte = (
            ("pwm_limit", ADDR_PWM_LIMIT, motor.pwm_limit),
            ("position_d_gain", ADDR_POSITION_D_GAIN, motor.position_d_gain),
            ("position_i_gain", ADDR_POSITION_I_GAIN, motor.position_i_gain),
            ("position_p_gain", ADDR_POSITION_P_GAIN, motor.position_p_gain),
        )
        writes_4byte = (
            ("profile_acceleration", ADDR_PROFILE_ACCELERATION, motor.profile_acceleration),
            ("profile_velocity", ADDR_PROFILE_VELOCITY, motor.profile_velocity),
        )

        applied = []
        for name, address, value in writes_2byte:
            if value is None:
                continue
            self._write2(motor.id, address, int(value))
            applied.append(f"{name}={int(value)}")

        for name, address, value in writes_4byte:
            if value is None:
                continue
            self._write4(motor.id, address, int(value))
            applied.append(f"{name}={int(value)}")

        if applied:
            self.get_logger().info(
                f"DXL {motor.id} {motor.joint_name} tuning: {', '.join(applied)}"
            )

    def _center_position_motors(self):
        self.get_logger().warn(
            "center_on_start is true: sending center tick to all position motors"
        )
        self._sync_write_goal_ticks(
            {motor: motor.center_tick for motor in self.position_motors}
        )
        self.get_logger().info("Sent center ticks to all position motors")

    def joint_state_callback(self, msg: JointState):
        if not self.received_joint_states:
            self.received_joint_states = True
            matched = [name for name in msg.name if name in self.joint_to_motor]
            self.get_logger().info(
                f"Received /joint_states with {len(msg.name)} joints; "
                f"{len(matched)} mapped to Dynamixel motors"
            )

        for joint_name, position in zip(msg.name, msg.position):
            motor = self.joint_to_motor.get(joint_name)
            if motor is None:
                continue

            if not math.isfinite(position):
                continue
            if abs(position) > self.max_abs_position_rad:
                self.get_logger().warn(
                    f"Skip {joint_name}: {position:.3f} rad exceeds "
                    f"max_abs_position_rad={self.max_abs_position_rad:.3f}",
                    throttle_duration_sec=1.0,
                )
                continue

            goal_tick = motor.position_to_tick(position)
            prev_tick = self.last_goal_ticks.get(motor.id)
            if prev_tick is not None and abs(goal_tick - prev_tick) < self.min_tick_change:
                continue

            self._write_goal_tick(motor, goal_tick)
            if self.get_parameter("log_joint_states").value:
                self.get_logger().info(
                    f"{joint_name} -> ID {motor.id}: {position:.4f} rad, tick {goal_tick}",
                    throttle_duration_sec=0.5,
                )

    def trajectory_callback(self, msg: JointTrajectory):
        trajectory = []
        mapped_names = [name for name in msg.joint_names if name in self.joint_to_motor]

        for point in msg.points:
            positions = {}
            for joint_name, position in zip(msg.joint_names, point.positions):
                if joint_name in self.joint_to_motor:
                    positions[joint_name] = position

            if not positions:
                continue

            time_from_start = point.time_from_start.sec + (
                point.time_from_start.nanosec * 1e-9
            )
            trajectory.append((time_from_start, positions))

        self.active_trajectory = trajectory
        self.trajectory_start_time = self.get_clock().now()
        self.get_logger().warn(
            f"Received trajectory: {len(msg.points)} points, "
            f"{len(mapped_names)} mapped joints, {len(trajectory)} executable points"
        )

    def trajectory_timer_callback(self):
        if not self.active_trajectory or self.trajectory_start_time is None:
            return

        elapsed = (
            self.get_clock().now() - self.trajectory_start_time
        ).nanoseconds * 1e-9

        due_index = -1
        for index, (time_from_start, _) in enumerate(self.active_trajectory):
            if time_from_start <= elapsed:
                due_index = index
            else:
                break

        if due_index < 0:
            return

        _, positions = self.active_trajectory[due_index]
        self.active_trajectory = self.active_trajectory[due_index + 1 :]
        self._send_position_map(positions, source="trajectory")

        if not self.active_trajectory:
            self.get_logger().info("Finished trajectory playback")

    def _setup_telemetry_logger(self):
        if GroupSyncRead is None:
            raise RuntimeError(
                "dynamixel_sdk GroupSyncRead is unavailable; cannot log telemetry."
            )

        log_path = self.get_parameter("telemetry_log_path").get_parameter_value().string_value
        if not log_path:
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
            log_path = os.path.join(
                os.getcwd(),
                "src",
                "biped_bike_robot",
                "motor_logs",
                f"dxl_telemetry_{stamp}.csv",
            )

        os.makedirs(os.path.dirname(os.path.abspath(log_path)), exist_ok=True)
        self.telemetry_file = open(log_path, "w", newline="", encoding="utf-8")
        self.telemetry_writer = csv.writer(self.telemetry_file)
        self.telemetry_writer.writerow(
            [
                "time_sec",
                "id",
                "joint_name",
                "control_mode",
                "goal_tick",
                "present_position_tick",
                "position_error_tick",
                "present_velocity_raw",
                "present_pwm_raw",
                "present_pwm_percent",
                "present_load_raw",
                "present_load_percent",
                "voltage_v",
                "temperature_c",
            ]
        )
        self.telemetry_file.flush()

        motor_ids_text = (
            self.get_parameter("telemetry_motor_ids").get_parameter_value().string_value
        )
        if motor_ids_text.strip():
            wanted_ids = {
                int(item.strip())
                for item in motor_ids_text.split(",")
                if item.strip()
            }
            self.telemetry_motors = [
                motor for motor in self.all_motors if motor.id in wanted_ids
            ]
        else:
            self.telemetry_motors = list(self.all_motors)

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_PWM,
            LEN_TELEMETRY_BLOCK,
        )
        for motor in self.telemetry_motors:
            if not self.group_sync_read.addParam(motor.id):
                self.get_logger().error(f"Failed to add DXL {motor.id} to telemetry sync read")

        rate_hz = self.get_parameter("telemetry_rate_hz").get_parameter_value().double_value
        rate_hz = max(1.0, rate_hz)
        self.telemetry_duration_sec = (
            self.get_parameter("telemetry_duration_sec").get_parameter_value().double_value
        )
        self.telemetry_start_time = self.get_clock().now()
        self.telemetry_timer = self.create_timer(1.0 / rate_hz, self.telemetry_callback)
        duration_text = (
            "until shutdown"
            if self.telemetry_duration_sec <= 0.0
            else f"for {self.telemetry_duration_sec:.1f}s"
        )
        self.get_logger().warn(
            f"Logging Dynamixel telemetry for {len(self.telemetry_motors)} motors "
            f"at {rate_hz:.1f} Hz {duration_text}: {log_path}"
        )

    def telemetry_callback(self):
        if self.group_sync_read is None or self.telemetry_writer is None:
            return

        now = self.get_clock().now()
        time_sec = (now - self.telemetry_start_time).nanoseconds * 1e-9
        if self.telemetry_duration_sec > 0.0 and time_sec > self.telemetry_duration_sec:
            self._stop_telemetry_logger()
            return

        result = self.group_sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().warn(
                f"Telemetry sync read failed: {self.packet_handler.getTxRxResult(result)}",
                throttle_duration_sec=1.0,
            )
            return

        for motor in self.telemetry_motors:
            if not self.group_sync_read.isAvailable(
                motor.id, ADDR_PRESENT_PWM, LEN_TELEMETRY_BLOCK
            ):
                self.get_logger().warn(
                    f"Telemetry unavailable for DXL {motor.id}",
                    throttle_duration_sec=1.0,
                )
                continue

            present_pwm = self._sync_read_signed(motor.id, ADDR_PRESENT_PWM, 2)
            present_load = self._sync_read_signed(motor.id, ADDR_PRESENT_LOAD, 2)
            present_velocity = self._sync_read_signed(motor.id, ADDR_PRESENT_VELOCITY, 4)
            present_position = self._sync_read_signed(motor.id, ADDR_PRESENT_POSITION, 4)
            voltage_raw = self.group_sync_read.getData(
                motor.id, ADDR_PRESENT_INPUT_VOLTAGE, 2
            )
            temperature = self.group_sync_read.getData(
                motor.id, ADDR_PRESENT_TEMPERATURE, 1
            )

            goal_tick = self.last_goal_ticks.get(motor.id, "")
            position_error = ""
            if isinstance(goal_tick, int):
                position_error = goal_tick - present_position

            self.telemetry_writer.writerow(
                [
                    f"{time_sec:.4f}",
                    motor.id,
                    motor.joint_name,
                    motor.control_mode,
                    goal_tick,
                    present_position,
                    position_error,
                    present_velocity,
                    present_pwm,
                    f"{present_pwm * 0.113:.3f}",
                    present_load,
                    f"{present_load * 0.1:.3f}",
                    f"{voltage_raw * 0.1:.2f}",
                    temperature,
                ]
            )
        self.telemetry_file.flush()

    def _stop_telemetry_logger(self):
        if hasattr(self, "telemetry_timer"):
            self.telemetry_timer.cancel()
        if self.telemetry_file is not None:
            self.telemetry_file.flush()
            self.telemetry_file.close()
            self.telemetry_file = None
        self.telemetry_writer = None
        self.group_sync_read = None
        self.get_logger().info("Stopped Dynamixel telemetry logging")

    def _sync_read_signed(self, dxl_id: int, address: int, length: int) -> int:
        value = self.group_sync_read.getData(dxl_id, address, length)
        bits = length * 8
        sign_bit = 1 << (bits - 1)
        if value & sign_bit:
            value -= 1 << bits
        return value

    def _send_position_map(self, positions: Dict[str, float], source: str):
        goal_ticks = {}
        skipped = 0

        for joint_name, position in positions.items():
            motor = self.joint_to_motor.get(joint_name)
            if motor is None or not math.isfinite(position):
                continue
            if abs(position) > self.max_abs_position_rad:
                skipped += 1
                continue

            goal_tick = motor.position_to_tick(position)
            prev_tick = self.last_goal_ticks.get(motor.id)
            if prev_tick is not None and abs(goal_tick - prev_tick) < self.min_tick_change:
                continue
            goal_ticks[motor] = goal_tick

        if skipped > 0:
            self.get_logger().warn(
                f"Skipped {skipped} {source} joints over "
                f"max_abs_position_rad={self.max_abs_position_rad:.3f}",
                throttle_duration_sec=1.0,
            )

        if goal_ticks:
            self._sync_write_goal_ticks(goal_ticks)

    def _write_goal_tick(self, motor: MotorConfig, goal_tick: int):
        self._write4(motor.id, ADDR_GOAL_POSITION, goal_tick)
        self.last_goal_ticks[motor.id] = goal_tick

    def _sync_write_goal_ticks(self, goal_ticks: Dict[MotorConfig, int]):
        self.group_sync_write.clearParam()

        for motor, goal_tick in goal_ticks.items():
            param = int(goal_tick).to_bytes(4, byteorder="little", signed=False)
            if not self.group_sync_write.addParam(motor.id, list(param)):
                self.get_logger().error(f"Failed to add DXL {motor.id} to sync write")

        result = self.group_sync_write.txPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"Sync write failed: {self.packet_handler.getTxRxResult(result)}"
            )
            return

        for motor, goal_tick in goal_ticks.items():
            self.last_goal_ticks[motor.id] = goal_tick

    def _write1(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _write2(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _write4(self, dxl_id: int, address: int, value: int):
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, dxl_id, address, value
        )
        self._check_result(dxl_id, address, result, error)

    def _check_result(self, dxl_id: int, address: int, result: int, error: int):
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"DXL {dxl_id} write failed at {address}: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            self.get_logger().error(
                f"DXL {dxl_id} packet error at {address}: "
                f"{self.packet_handler.getRxPacketError(error)}"
            )


def main(args: Optional[List[str]] = None):
    rclpy.init(args=args)
    node = DxlJointStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
