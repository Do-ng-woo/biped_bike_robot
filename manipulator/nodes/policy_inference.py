#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.config import load_config, resolve_under_root
from common.joints import ordered_positions
from common.status import encode_status


class PolicyInference(Node):
    def __init__(self, config_path: str | None = None, model_path: str | None = None):
        super().__init__("manipulator_policy_inference")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        arm = self.cfg["arm"]
        training = self.cfg["training"]
        self.names = list(arm["follower_joint_names"])
        self.minimum = np.asarray(arm["min_position"], dtype=np.float32)
        self.maximum = np.asarray(arm["max_position"], dtype=np.float32)
        self.duration = float(arm["command_duration_sec"])
        self.model_root = resolve_under_root(self.cfg, training["model_root"])
        self.model_root.mkdir(parents=True, exist_ok=True)
        self.mode = "IDLE"
        self.latest_image: bytes | None = None
        self.latest_state: np.ndarray | None = None
        self.image_stamp = 0.0
        self.state_stamp = 0.0
        self.model = None
        self.checkpoint = None
        self.device = None
        self.publisher = self.create_publisher(JointTrajectory, topics["inference_target"], 10)
        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.model_status_pub = self.create_publisher(
            String, topics["inference_model_state"], status_qos
        )
        self.create_subscription(CompressedImage, topics["camera_compressed"], self.on_image, 10)
        self.create_subscription(JointState, topics["joint_states"], self.on_state, 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(String, topics["inference_model_command"], self.on_model, 10)
        self.timer = self.create_timer(0.1, self.tick)
        if model_path:
            self.load_model(model_path)
        else:
            self.publish_model_status("UNLOADED", "no model selected")

    def on_image(self, msg: CompressedImage):
        self.latest_image = bytes(msg.data)
        self.image_stamp = time.monotonic()

    def on_state(self, msg: JointState):
        values = ordered_positions(msg.name, msg.position, self.names)
        if values is not None:
            self.latest_state = np.asarray(values, dtype=np.float32)
            self.state_stamp = time.monotonic()

    def on_mode(self, msg: String):
        try:
            self.mode = str(json.loads(msg.data).get("mode", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            self.mode = "IDLE"

    def on_model(self, msg: String):
        self.model = self.checkpoint = self.device = None
        try:
            self.load_model(msg.data)
        except Exception as exc:
            self.publish_model_status("ERROR", str(exc), path=msg.data)
            self.get_logger().error(f"Failed to load model: {exc}")

    def publish_model_status(self, state: str, detail: str, **extra):
        msg = String()
        msg.data = encode_status(state=state, detail=detail, stamp=time.time(), **extra)
        self.model_status_pub.publish(msg)

    def load_model(self, value: str):
        import torch
        from learning.model import VisualBCPolicy

        path = Path(value).expanduser()
        if not path.is_absolute():
            path = self.model_root / path
        path = path.resolve()
        if not path.is_relative_to(self.model_root.resolve()):
            raise ValueError(f"model must be under {self.model_root}")
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = torch.load(path, map_location=device, weights_only=True)
        if checkpoint.get("format") != "biped-bike-visual-bc-v1":
            raise ValueError("unsupported checkpoint format")
        model_joint_names = list(checkpoint.get("joint_names", []))
        if model_joint_names != self.names:
            raise ValueError(
                f"checkpoint joint order {model_joint_names} does not match robot {self.names}"
            )
        if int(checkpoint.get("state_dim", -1)) != len(self.names):
            raise ValueError("checkpoint state dimension does not match robot")
        if int(checkpoint.get("action_dim", -1)) != len(self.names):
            raise ValueError("checkpoint action dimension does not match robot")
        stats = checkpoint.get("stats", {})
        for key in ("state_mean", "state_std", "action_mean", "action_std"):
            values = np.asarray(stats.get(key, []), dtype=np.float32)
            if values.shape != (len(self.names),) or not np.all(np.isfinite(values)):
                raise ValueError(f"invalid checkpoint normalization field: {key}")
        if np.any(np.asarray(stats["state_std"]) <= 0) or np.any(np.asarray(stats["action_std"]) <= 0):
            raise ValueError("checkpoint normalization standard deviations must be positive")
        model = VisualBCPolicy(checkpoint["state_dim"], checkpoint["action_dim"])
        model.load_state_dict(checkpoint["model_state"])
        model.to(device).eval()
        self.model, self.checkpoint, self.device = model, checkpoint, device
        self.publish_model_status(
            "LOADED", "compatible model loaded", path=str(path),
            joint_names=self.names, device=str(device),
        )
        self.get_logger().info(f"Loaded policy {path} on {device}")

    def tick(self):
        if self.mode != "INFERENCE" or self.model is None:
            return
        now = time.monotonic()
        if self.latest_image is None or self.latest_state is None:
            return
        if now - self.image_stamp > 0.5 or now - self.state_stamp > 0.5:
            return
        import torch
        from learning.dataset import preprocess_image_bytes

        ckpt = self.checkpoint
        image = preprocess_image_bytes(
            self.latest_image, ckpt["image_width"], ckpt["image_height"]
        )
        stats = ckpt["stats"]
        state = (
            self.latest_state - np.asarray(stats["state_mean"], dtype=np.float32)
        ) / np.asarray(stats["state_std"], dtype=np.float32)
        with torch.no_grad():
            action_norm = self.model(
                torch.from_numpy(image).unsqueeze(0).to(self.device),
                torch.from_numpy(state).unsqueeze(0).to(self.device),
            )[0].cpu().numpy()
        action = action_norm * np.asarray(
            stats["action_std"], dtype=np.float32
        ) + np.asarray(stats["action_mean"], dtype=np.float32)
        if not np.all(np.isfinite(action)):
            self.model = self.checkpoint = self.device = None
            self.publish_model_status("ERROR", "model produced a non-finite action")
            return
        action = np.clip(action, self.minimum, self.maximum)
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = self.names
        point = JointTrajectoryPoint()
        point.positions = action.astype(float).tolist()
        sec = int(self.duration)
        point.time_from_start = Duration(
            sec=sec, nanosec=int((self.duration - sec) * 1e9)
        )
        msg.points = [point]
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    parser.add_argument("--model")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = PolicyInference(args.config, args.model)
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
