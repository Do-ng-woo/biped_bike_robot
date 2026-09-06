#!/usr/bin/env python3
from __future__ import annotations

import argparse

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.config import load_config
from common.dynamixel import position_tick_to_rad

try:
    from dynamixel_sdk import COMM_SUCCESS, GroupSyncRead, PacketHandler, PortHandler
except ImportError as exc:  # pragma: no cover - depends on the ROS installation
    raise RuntimeError(
        "dynamixel_sdk is required for the native OpenRB leader reader"
    ) from exc


PRESENT_POSITION_ADDRESS = 132
PRESENT_POSITION_LENGTH = 4


class OpenRBLeaderReader(Node):
    """Read the passive leader only; never send a position or torque command."""

    def __init__(self, config_path: str | None = None):
        super().__init__("manipulator_openrb_leader_reader")
        cfg = load_config(config_path)
        leader = cfg["leader_reader"]
        arm = cfg["arm"]
        self.device = str(cfg["devices"]["leader"])
        self.baudrate = int(leader["baudrate"])
        self.ids = [int(value) for value in leader["motor_ids"]]
        self.names = list(arm["leader_joint_names"])
        self.center_ticks = [int(value) for value in leader["center_ticks"]]
        self.directions = [float(value) for value in leader["direction"]]
        self.rate_hz = float(leader["publish_rate_hz"])
        if not (
            len(self.ids)
            == len(self.names)
            == len(self.center_ticks)
            == len(self.directions)
        ):
            raise ValueError("leader IDs, names, centers, and directions must match")
        if self.rate_hz <= 0.0:
            raise ValueError("leader publish rate must be positive")

        self.port = PortHandler(self.device)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort():
            raise RuntimeError(f"failed to open OpenRB leader port: {self.device}")
        if not self.port.setBaudRate(self.baudrate):
            self.port.closePort()
            raise RuntimeError(f"failed to set OpenRB baudrate: {self.baudrate}")

        self.reader = GroupSyncRead(
            self.port,
            self.packet,
            PRESENT_POSITION_ADDRESS,
            PRESENT_POSITION_LENGTH,
        )
        for dxl_id in self.ids:
            if not self.reader.addParam(dxl_id):
                self.port.closePort()
                raise RuntimeError(f"failed to register leader DXL ID {dxl_id}")

        self.trajectory_pub = self.create_publisher(
            JointTrajectory, cfg["topics"]["leader_trajectory"], 10
        )
        self.joint_state_pub = self.create_publisher(
            JointState, "/leader/joint_states", 10
        )
        self.timer = self.create_timer(1.0 / self.rate_hz, self.read_and_publish)
        self.get_logger().info(
            f"Opened passive OpenRB leader {self.device} @ {self.baudrate}; "
            f"reading IDs {self.ids} at {self.rate_hz:.1f} Hz"
        )

    def read_and_publish(self) -> None:
        result = self.reader.txRxPacket()
        if result != COMM_SUCCESS:
            self.get_logger().error(
                f"leader sync read failed: {self.packet.getTxRxResult(result)}",
                throttle_duration_sec=1.0,
            )
            return

        positions: list[float] = []
        for dxl_id, center, direction in zip(
            self.ids, self.center_ticks, self.directions, strict=True
        ):
            if not self.reader.isAvailable(
                dxl_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH
            ):
                self.get_logger().error(
                    f"leader DXL ID {dxl_id} returned no present position",
                    throttle_duration_sec=1.0,
                )
                return
            raw = self.reader.getData(
                dxl_id, PRESENT_POSITION_ADDRESS, PRESENT_POSITION_LENGTH
            )
            positions.append(direction * position_tick_to_rad(raw, center))

        stamp = self.get_clock().now().to_msg()
        state = JointState()
        state.header.stamp = stamp
        state.name = self.names
        state.position = positions
        self.joint_state_pub.publish(state)

        trajectory = JointTrajectory()
        trajectory.header.stamp = stamp
        trajectory.joint_names = self.names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=0, nanosec=0)
        trajectory.points = [point]
        self.trajectory_pub.publish(trajectory)

    def close(self) -> None:
        try:
            self.reader.clearParam()
        finally:
            self.port.closePort()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = OpenRBLeaderReader(args.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
