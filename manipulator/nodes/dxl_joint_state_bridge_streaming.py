#!/usr/bin/env python3
"""Low-latency adapter for the existing biped DYNAMIXEL bridge.

The original bridge performs one blocking Present Position read per commanded
joint whenever a trajectory arrives.  That is appropriate for occasional
posture trajectories, but saturates the 1 Mbps bus during 30 Hz leader/follower
streaming.  This adapter keeps the original motor configuration and safety
checks while replacing those reads with one GroupSyncRead packet and a cache.
"""
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
from types import ModuleType
from typing import Sequence


POSITION_READ_GROUP_SIZE = 7


def load_bridge(path: Path) -> ModuleType:
    spec = importlib.util.spec_from_file_location("biped_base_dxl_bridge", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load DYNAMIXEL bridge: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cached_start_positions(bridge, joint_names):
    """Return trajectory start positions without issuing serial reads."""
    positions = {}
    cache = getattr(bridge, "latest_present_positions", {})
    for joint_name in joint_names:
        motor = bridge.joint_to_motor[joint_name]
        if joint_name in cache:
            positions[joint_name] = cache[joint_name]
            continue
        tick = bridge.last_goal_ticks.get(motor.id, motor.center_tick)
        positions[joint_name] = motor.tick_to_position(tick)
    return positions


def one_point_positions(msg, joint_to_motor):
    """Extract a streaming command, or return None for a timed trajectory."""
    if len(msg.points) != 1:
        return None
    return {
        name: position
        for name, position in zip(msg.joint_names, msg.points[0].positions)
        if name in joint_to_motor
    }


def chunked(values, size):
    return [values[index:index + size] for index in range(0, len(values), size)]


def add_streaming_support(module: ModuleType) -> None:
    original_trajectory_callback = module.DxlJointStateBridge.trajectory_callback

    def setup_present_joint_state_publisher(self, topic: str):
        self.present_joint_state_pub = self.create_publisher(module.JointState, topic, 10)
        self.latest_present_positions = {}
        self.position_sync_read_groups = []
        for motors in chunked(self.all_motors, POSITION_READ_GROUP_SIZE):
            reader = module.GroupSyncRead(
                self.port_handler,
                self.packet_handler,
                module.ADDR_PRESENT_POSITION,
                4,
            )
            for motor in motors:
                if not reader.addParam(motor.id):
                    raise RuntimeError(
                        f"Failed to add DXL {motor.id} to position GroupSyncRead"
                    )
            self.position_sync_read_groups.append((reader, motors))
        rate_hz = max(
            1.0,
            float(self.get_parameter("present_joint_state_rate_hz").value),
        )
        self.present_joint_state_timer = self.create_timer(
            1.0 / rate_hz, self.present_joint_state_callback
        )
        self.get_logger().warn(
            f"Publishing {len(self.all_motors)} DYNAMIXEL positions with "
            f"{len(self.position_sync_read_groups)} GroupSyncRead groups at "
            f"{rate_hz:.1f} Hz"
        )

    def present_joint_state_callback(self):
        msg = module.JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for reader, motors in self.position_sync_read_groups:
            result = reader.txRxPacket()
            if result != module.COMM_SUCCESS:
                ids = ",".join(str(motor.id) for motor in motors)
                self.get_logger().error(
                    f"Position GroupSyncRead failed for IDs [{ids}]: "
                    f"{self.packet_handler.getTxRxResult(result)}",
                    throttle_duration_sec=1.0,
                )
                continue
            for motor in motors:
                if not reader.isAvailable(
                    motor.id, module.ADDR_PRESENT_POSITION, 4
                ):
                    self.get_logger().error(
                        f"DXL {motor.id} missing from position GroupSyncRead",
                        throttle_duration_sec=1.0,
                    )
                    continue
                tick = int(
                    reader.getData(
                        motor.id, module.ADDR_PRESENT_POSITION, 4
                    )
                )
                position = motor.tick_to_position(tick)
                self.latest_present_positions[motor.joint_name] = position
                msg.name.append(motor.joint_name)
                msg.position.append(position)
                msg.velocity.append(0.0)
                msg.effort.append(0.0)
        if msg.name:
            self.present_joint_state_pub.publish(msg)

    def read_current_positions(self, joint_names):
        return cached_start_positions(self, joint_names)

    def trajectory_callback(self, msg):
        positions = one_point_positions(msg, self.joint_to_motor)
        if positions is None:
            original_trajectory_callback(self, msg)
            return
        if not positions:
            self.get_logger().warn(
                "Ignored one-point trajectory with no executable joints",
                throttle_duration_sec=1.0,
            )
            return
        # The arbiter already rate-limits and clamps streaming targets. Sending
        # the point immediately avoids restarting a 33 ms interpolation at every
        # 30 Hz callback and avoids the associated warning flood.
        self.active_trajectory = []
        self.trajectory_start_positions = {}
        self.trajectory_start_time = None
        self._send_position_map(positions, source="stream")
        gripper = positions.get("arm_gripper_jnt")
        if gripper is not None:
            motor = self.joint_to_motor["arm_gripper_jnt"]
            self.get_logger().info(
                f"Gripper stream target: {gripper:.3f} rad, "
                f"tick {motor.position_to_tick(gripper)}",
                throttle_duration_sec=1.0,
            )

    module.DxlJointStateBridge._setup_present_joint_state_publisher = (
        setup_present_joint_state_publisher
    )
    module.DxlJointStateBridge.present_joint_state_callback = (
        present_joint_state_callback
    )
    module.DxlJointStateBridge._read_current_positions = read_current_positions
    module.DxlJointStateBridge.trajectory_callback = trajectory_callback


def parse_args(argv: Sequence[str] | None = None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--base-bridge", required=True)
    return parser.parse_known_args(argv)


def main(argv: Sequence[str] | None = None) -> None:
    args, ros_args = parse_args(argv)
    bridge_path = Path(args.base_bridge).expanduser().resolve()
    if not bridge_path.is_file():
        raise FileNotFoundError(f"Base DYNAMIXEL bridge not found: {bridge_path}")
    module = load_bridge(bridge_path)
    add_streaming_support(module)
    module.main(args=list(ros_args))


if __name__ == "__main__":
    main()
