#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from common.config import load_config
from common.joints import ordered_positions
from common.postures import load_reference_postures
from common.status import decode_message, encode_status


ACTIVE_MODES = {"TELEOP", "RECORD", "INFERENCE"}
LOCKED_MODES = ACTIVE_MODES | {"STABLE"}
VALID_REQUESTS = ACTIVE_MODES | {"STABILIZE", "IDLE", "FAULT"}


class ModeManager(Node):
    def __init__(self, config_path: str | None = None):
        super().__init__("manipulator_mode_manager")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        lower = self.cfg["lower_body"]
        self.lower_names = list(lower["joint_names"])
        reference = load_reference_postures(self.cfg)
        self.kneeling = list(reference.kneeling[:len(self.lower_names)])
        if len(self.lower_names) != len(self.kneeling):
            raise ValueError("lower_body joint_names and kneeling_positions differ in length")
        self.settle_time = float(lower["settle_time_sec"])
        self.tolerance = float(lower["position_tolerance_rad"])
        self.timeout = float(lower["transition_timeout_sec"])
        self.drift_tolerance = float(lower["active_drift_tolerance_rad"])
        self.drift_fault_sec = float(lower["active_drift_fault_sec"])
        self.joint_state_timeout = float(lower["joint_state_timeout_sec"])
        self.mode = "IDLE"
        self.requested_mode = "IDLE"
        self.transition_started = 0.0
        self.in_tolerance_since: float | None = None
        self.latest_lower: list[float] | None = None
        self.last_joint_update = 0.0
        self.last_detail = "ready"
        self.drift_started: float | None = None
        self.inference_model_ready = False
        self.lower_error_by_joint: dict[str, float] = {}

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(String, topics["mode_state"], state_qos)
        self.wheel_pub = self.create_publisher(Float64MultiArray, topics["wheel_command"], 10)
        self.create_subscription(String, topics["mode_request"], self.on_request, 10)
        self.create_subscription(String, topics["inference_model_state"], self.on_model_state, 10)
        self.create_subscription(JointState, topics["joint_states"], self.on_joint_state, 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.state_timer = self.create_timer(0.5, self.publish_state)
        self.publish_state("ready")

    def on_joint_state(self, msg: JointState):
        values = ordered_positions(msg.name, msg.position, self.lower_names)
        if values is not None:
            self.latest_lower = values
            self.last_joint_update = time.monotonic()
            self.lower_error_by_joint = {
                name: abs(actual - target)
                for name, actual, target in zip(self.lower_names, values, self.kneeling)
            }

    def on_request(self, msg: String):
        request = decode_message(msg.data)
        requested = str(request.get("mode", request.get("command", ""))).upper()
        if requested not in VALID_REQUESTS:
            self.publish_state(f"invalid mode request: {requested}")
            return
        if requested == "FAULT":
            self.mode = "FAULT"
            self.requested_mode = "FAULT"
            self.stop_wheels()
            self.publish_state("fault requested")
            return
        if requested == "IDLE":
            self.mode = "IDLE"
            self.requested_mode = "IDLE"
            self.stop_wheels()
            self.publish_state("arm disabled; lower body remains at its last goal")
            return
        if requested == "STABILIZE":
            if self.mode == "STABLE":
                self.requested_mode = "STABLE"
                self.publish_state("fixed kneeling pose is already locked")
                return
            self.requested_mode = "STABLE"
            self.mode = "KNEELING"
            self.transition_started = time.monotonic()
            self.in_tolerance_since = None
            self.drift_started = None
            self.stop_wheels()
            self.publish_state("moving to DEEP_SQUAT_ARMS_UP fixed kneeling pose")
            return
        if self.mode not in LOCKED_MODES:
            self.publish_state(
                f"{requested.lower()} rejected: press Kneeling Stable and wait for STABLE first"
            )
            return
        if requested == "INFERENCE" and not self.inference_model_ready:
            self.publish_state("inference rejected: no compatible model is loaded")
            return
        self.requested_mode = requested
        self.mode = requested
        self.drift_started = None
        self.stop_wheels()
        self.publish_state(f"fixed kneeling pose confirmed; {requested.lower()} enabled")

    def on_model_state(self, msg: String):
        try:
            state = str(json.loads(msg.data).get("state", "ERROR")).upper()
        except (json.JSONDecodeError, AttributeError):
            state = "ERROR"
        self.inference_model_ready = state == "LOADED"
        if self.mode == "INFERENCE" and not self.inference_model_ready:
            self.mode = "FAULT"
            self.requested_mode = "FAULT"
            self.publish_state("loaded inference model became unavailable")

    def stop_wheels(self):
        msg = Float64MultiArray()
        msg.data = [0.0, 0.0]
        self.wheel_pub.publish(msg)

    def tick(self):
        # Manipulator mode never permits wheel motion.
        self.stop_wheels()
        now = time.monotonic()
        if self.mode in LOCKED_MODES:
            if self.latest_lower is None or now - self.last_joint_update > self.joint_state_timeout:
                self.mode = "FAULT"
                self.requested_mode = "FAULT"
                self.publish_state("lower-body joint state lost during active mode")
                return
            error = max(abs(a - b) for a, b in zip(self.latest_lower, self.kneeling))
            if error > self.drift_tolerance:
                if self.drift_started is None:
                    self.drift_started = now
                elif now - self.drift_started >= self.drift_fault_sec:
                    self.mode = "FAULT"
                    self.requested_mode = "FAULT"
                    self.publish_state(f"lower-body drift detected: {error:.3f} rad")
                    return
            else:
                self.drift_started = None
        if self.mode != "KNEELING":
            return
        if now - self.transition_started > self.timeout:
            self.mode = "FAULT"
            self.requested_mode = "FAULT"
            if self.lower_error_by_joint:
                largest = sorted(
                    self.lower_error_by_joint.items(), key=lambda item: item[1], reverse=True
                )[:4]
                summary = ", ".join(f"{name}={error:.3f}rad" for name, error in largest)
                self.publish_state(f"kneeling transition timeout; largest errors: {summary}")
            else:
                self.publish_state("kneeling transition timeout; no complete lower joint state")
            return
        if self.latest_lower is None or now - self.last_joint_update > 0.5:
            return
        error = max(abs(a - b) for a, b in zip(self.latest_lower, self.kneeling))
        if error <= self.tolerance:
            if self.in_tolerance_since is None:
                self.in_tolerance_since = now
            elif now - self.in_tolerance_since >= self.settle_time:
                self.mode = "STABLE"
                self.requested_mode = "STABLE"
                self.drift_started = None
                self.publish_state("DEEP_SQUAT_ARMS_UP locked; teleop is now available")
        else:
            self.in_tolerance_since = None

    def publish_state(self, detail: str | None = None):
        if detail is not None:
            self.last_detail = detail
        msg = String()
        msg.data = encode_status(
            mode=self.mode,
            requested_mode=self.requested_mode,
            detail=self.last_detail,
            lower_error_rad=self.lower_error_by_joint,
            stamp=time.time(),
        )
        self.state_pub.publish(msg)
        if detail is not None:
            self.get_logger().info(msg.data)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = ModeManager(args.config)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
