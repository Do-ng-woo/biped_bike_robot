#!/usr/bin/env python3
from __future__ import annotations

import argparse
from collections import deque
import json
from pathlib import Path
import re
import shutil
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory

from common.config import load_config, resolve_under_root
from common.joints import ordered_positions
from common.postures import load_reference_postures
from common.status import decode_message, encode_status


SAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")


def safe_name(value: str, fallback: str) -> str:
    value = SAFE_NAME.sub("_", value.strip()).strip("._")
    return value[:80] or fallback


class EpisodeRecorder(Node):
    def __init__(self, config_path: str | None = None):
        super().__init__("manipulator_episode_recorder")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        arm = self.cfg["arm"]
        rec = self.cfg["recording"]
        self.arm_names = list(arm["follower_joint_names"])
        self.data_root = resolve_under_root(self.cfg, rec["data_root"])
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.sample_timeout = float(rec["sample_timeout_sec"])
        self.max_sync_skew = float(rec["max_sync_skew_sec"])
        queue_size = int(rec["sync_queue_size"])
        reference = load_reference_postures(self.cfg)
        self.reset_position = list(reference.kneeling[len(self.cfg["lower_body"]["joint_names"]):])
        self.reset_tolerance = float(arm["reset_tolerance_rad"])
        self.reset_settle = float(arm["reset_settle_sec"])
        self.defaults = {
            "fps": float(rec["default_fps"]),
            "warmup_sec": float(rec["default_warmup_sec"]),
            "episode_sec": float(rec["default_episode_sec"]),
            "reset_sec": float(rec["default_reset_sec"]),
            "num_episodes": int(rec["default_num_episodes"]),
        }
        self.mode = "IDLE"
        self.state = "IDLE"
        self.detail = "ready"
        self.options = dict(self.defaults)
        self.dataset_dir: Path | None = None
        self.episode_index = 0
        self.state_started = time.monotonic()
        self.next_sample_time = 0.0
        self.latest_image: tuple[float, bytes] | None = None
        self.latest_state: tuple[float, list[float]] | None = None
        self.latest_action: tuple[float, list[float]] | None = None
        self.image_queue = deque(maxlen=queue_size)
        self.state_queue = deque(maxlen=queue_size)
        self.action_queue = deque(maxlen=queue_size)
        self.last_image_stamp = -1.0
        self.reset_in_tolerance_since: float | None = None
        self.samples: list[tuple[float, float, float, list[float], list[float], str]] = []
        self.partial_dir: Path | None = None

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(String, topics["record_state"], state_qos)
        self.create_subscription(String, topics["record_command"], self.on_command, 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(CompressedImage, topics["camera_compressed"], self.on_image, 10)
        self.create_subscription(JointState, topics["joint_states"], self.on_joint_state, 10)
        self.create_subscription(JointTrajectory, topics["teleop_target"], self.on_action, 10)
        self.timer = self.create_timer(0.01, self.tick)
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.publish_status()

    def on_mode(self, msg: String):
        try:
            self.mode = str(json.loads(msg.data).get("mode", "IDLE")).upper()
        except (json.JSONDecodeError, AttributeError):
            self.mode = "IDLE"
        if self.state in {"WARMUP", "RECORDING", "RESETTING"} and self.mode != "RECORD":
            self.stop_episode("record mode exited")

    def on_image(self, msg: CompressedImage):
        arrival = time.monotonic()
        value = bytes(msg.data)
        stamp = self.message_stamp(msg.header.stamp)
        self.latest_image = (arrival, value)
        self.image_queue.append((stamp, arrival, value))

    def on_joint_state(self, msg: JointState):
        values = ordered_positions(msg.name, msg.position, self.arm_names)
        if values is not None:
            arrival = time.monotonic()
            stamp = self.message_stamp(msg.header.stamp)
            self.latest_state = (arrival, values)
            self.state_queue.append((stamp, arrival, values))

    def on_action(self, msg: JointTrajectory):
        if not msg.points:
            return
        values = ordered_positions(msg.joint_names, msg.points[-1].positions, self.arm_names)
        if values is not None:
            arrival = time.monotonic()
            stamp = self.message_stamp(msg.header.stamp)
            self.latest_action = (arrival, values)
            self.action_queue.append((stamp, arrival, values))

    def message_stamp(self, stamp) -> float:
        value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        return value if value > 0.0 else self.get_clock().now().nanoseconds * 1e-9

    def on_command(self, msg: String):
        request = decode_message(msg.data)
        command = str(request.get("command", "")).lower()
        try:
            if command == "start":
                self.start_dataset(request)
            elif command == "stop":
                self.stop_episode("stopped by user")
            elif command == "next":
                self.finish_current_episode("next episode requested")
            elif command == "retry":
                self.retry_episode()
            elif command == "finish":
                self.finish_dataset()
            else:
                raise ValueError(f"unknown record command: {command}")
        except Exception as exc:
            self.state = "ERROR"
            self.detail = str(exc)
            self.get_logger().error(self.detail)
            self.publish_status()

    def start_dataset(self, request: dict):
        if self.mode != "RECORD":
            raise RuntimeError("robot must be in RECORD mode before recording")
        if self.state in {"WARMUP", "RECORDING", "RESETTING", "SAVING"}:
            raise RuntimeError("recording is already active")
        self.options = dict(self.defaults)
        for key in self.options:
            if key in request:
                self.options[key] = type(self.options[key])(request[key])
        if self.options["warmup_sec"] < 0 or self.options["reset_sec"] <= 0:
            raise ValueError("warmup_sec cannot be negative and reset_sec must be positive")
        if self.options["fps"] <= 0 or self.options["episode_sec"] <= 0 or self.options["num_episodes"] <= 0:
            raise ValueError("fps, episode_sec and num_episodes must be positive")
        user = safe_name(str(request.get("user_id", "local")), "local")
        name = safe_name(str(request.get("dataset", "dataset")), "dataset")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        self.dataset_dir = self.data_root / user / f"{name}_{timestamp}"
        self.dataset_dir.mkdir(parents=True, exist_ok=False)
        self.options["task"] = str(request.get("task", ""))
        self.options["created_at"] = time.strftime("%Y-%m-%dT%H:%M:%S%z")
        self.episode_index = 0
        self.samples = []
        self.begin_episode("WARMUP")

    def begin_episode(self, initial_state: str = "RECORDING"):
        assert self.dataset_dir is not None
        self.partial_dir = self.dataset_dir / f".episode_{self.episode_index:06d}.partial"
        if self.partial_dir.exists():
            shutil.rmtree(self.partial_dir)
        (self.partial_dir / "frames").mkdir(parents=True)
        self.samples = []
        self.last_image_stamp = -1.0
        self.reset_in_tolerance_since = None
        self.state = initial_state
        self.detail = f"episode {self.episode_index + 1}/{self.options['num_episodes']}"
        self.state_started = time.monotonic()
        self.next_sample_time = self.state_started
        self.publish_status()

    def tick(self):
        now = time.monotonic()
        if self.state == "WARMUP":
            if now - self.state_started >= float(self.options["warmup_sec"]):
                self.state = "RECORDING"
                self.state_started = now
                self.next_sample_time = now
                self.detail = "recording"
                self.publish_status()
            return
        if self.state == "RESETTING":
            if self.latest_state is not None and now - self.latest_state[0] <= self.sample_timeout:
                error = max(
                    abs(a - b) for a, b in zip(self.latest_state[1], self.reset_position)
                )
                if error <= self.reset_tolerance:
                    if self.reset_in_tolerance_since is None:
                        self.reset_in_tolerance_since = now
                    elif now - self.reset_in_tolerance_since >= self.reset_settle:
                        self.begin_episode("WARMUP")
                        return
                else:
                    self.reset_in_tolerance_since = None
                self.detail = f"resetting arm; max error {error:.3f} rad"
            if now - self.state_started >= float(self.options["reset_sec"]):
                self.state = "ERROR"
                self.detail = "arm reset pose timeout"
                self.publish_status()
            return
        if self.state != "RECORDING":
            return
        if now - self.state_started >= float(self.options["episode_sec"]):
            self.finish_current_episode("episode time reached")
            return
        if now < self.next_sample_time:
            return
        self.next_sample_time += 1.0 / float(self.options["fps"])
        self.capture_sample(now)

    def capture_sample(self, now: float):
        if self.partial_dir is None:
            return
        if not self.image_queue or not self.state_queue or not self.action_queue:
            self.detail = "waiting for fresh camera, follower state and leader action"
            return
        image_stamp, image_arrival, image_bytes = self.image_queue[-1]
        if image_stamp <= self.last_image_stamp or now - image_arrival > self.sample_timeout:
            self.detail = "waiting for a new camera frame"
            return
        state = min(self.state_queue, key=lambda item: abs(item[0] - image_stamp))
        action = min(self.action_queue, key=lambda item: abs(item[0] - image_stamp))
        if now - state[1] > self.sample_timeout or now - action[1] > self.sample_timeout:
            self.detail = "waiting for fresh follower state and leader action"
            return
        skew = max(abs(state[0] - image_stamp), abs(action[0] - image_stamp))
        if skew > self.max_sync_skew:
            self.detail = f"waiting for synchronized data; skew {skew * 1000.0:.1f} ms"
            return
        frame_name = f"{len(self.samples):06d}.jpg"
        frame_path = self.partial_dir / "frames" / frame_name
        frame_path.write_bytes(image_bytes)
        self.samples.append(
            (image_stamp, state[0], action[0], list(state[2]), list(action[2]), frame_name)
        )
        self.last_image_stamp = image_stamp
        self.detail = f"recording frame {len(self.samples)}"

    def finish_current_episode(self, reason: str):
        if self.state not in {"RECORDING", "WARMUP"}:
            return
        if not self.samples:
            self.detail = "episode has no synchronized samples"
            self.state = "STOPPED"
            self.publish_status()
            return
        self.save_episode()
        self.episode_index += 1
        if self.episode_index >= int(self.options["num_episodes"]):
            self.finish_dataset()
        else:
            self.state = "RESETTING"
            self.state_started = time.monotonic()
            self.reset_in_tolerance_since = None
            self.detail = reason
            self.publish_status()

    def stop_episode(self, reason: str):
        if self.state in {"RECORDING", "WARMUP"} and self.samples:
            self.save_episode()
            self.episode_index += 1
        self.state = "STOPPED"
        self.detail = reason
        self.publish_status()

    def retry_episode(self):
        if self.dataset_dir is None:
            raise RuntimeError("no dataset is active")
        if self.partial_dir and self.partial_dir.exists():
            shutil.rmtree(self.partial_dir)
        elif self.episode_index > 0:
            previous = self.dataset_dir / f"episode_{self.episode_index - 1:06d}"
            if previous.exists():
                discarded = self.dataset_dir / ".discarded"
                discarded.mkdir(exist_ok=True)
                previous.rename(discarded / f"{previous.name}_{int(time.time())}")
                self.episode_index -= 1
        self.begin_episode("WARMUP")
        self.detail = "retrying episode"

    def save_episode(self):
        assert self.partial_dir is not None
        stamps = np.asarray([sample[0] for sample in self.samples], dtype=np.float64)
        state_stamps = np.asarray([sample[1] for sample in self.samples], dtype=np.float64)
        action_stamps = np.asarray([sample[2] for sample in self.samples], dtype=np.float64)
        states = np.asarray([sample[3] for sample in self.samples], dtype=np.float32)
        actions = np.asarray([sample[4] for sample in self.samples], dtype=np.float32)
        np.savez_compressed(
            self.partial_dir / "samples.npz",
            timestamp=stamps,
            state_timestamp=state_stamps,
            action_timestamp=action_stamps,
            observation_state=states,
            action=actions,
            joint_names=np.asarray(self.arm_names),
        )
        (self.partial_dir / "episode.json").write_text(
            json.dumps({
                "episode_index": self.episode_index,
                "frames": len(self.samples),
                "fps": self.options["fps"],
                "task": self.options.get("task", ""),
                "max_sync_skew_sec": self.max_sync_skew,
            }, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.encode_video(self.partial_dir)
        final_dir = self.dataset_dir / f"episode_{self.episode_index:06d}"
        self.partial_dir.rename(final_dir)
        self.partial_dir = None
        self.samples = []

    def encode_video(self, episode_dir: Path):
        frames = sorted((episode_dir / "frames").glob("*.jpg"))
        if not frames:
            return
        first = cv2.imread(str(frames[0]))
        if first is None:
            return
        height, width = first.shape[:2]
        writer = cv2.VideoWriter(
            str(episode_dir / "camera.mp4"),
            cv2.VideoWriter_fourcc(*"mp4v"),
            float(self.options["fps"]),
            (width, height),
        )
        try:
            for frame_path in frames:
                frame = cv2.imread(str(frame_path))
                if frame is not None:
                    writer.write(frame)
        finally:
            writer.release()

    def finish_dataset(self):
        if self.dataset_dir is None:
            return
        if self.state in {"RECORDING", "WARMUP"} and self.samples:
            self.save_episode()
            self.episode_index += 1
        elif self.partial_dir and self.partial_dir.exists():
            shutil.rmtree(self.partial_dir)
            self.partial_dir = None
        metadata = dict(self.options)
        metadata.update({
            "format": "biped-bike-imitation-v1",
            "episodes": self.episode_index,
            "observation_joint_names": self.arm_names,
            "action_joint_names": self.arm_names,
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        })
        (self.dataset_dir / "dataset.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        self.state = "FINISHED"
        self.detail = str(self.dataset_dir)
        self.publish_status()

    def publish_status(self):
        msg = String()
        elapsed = max(0.0, time.monotonic() - self.state_started)
        msg.data = encode_status(
            state=self.state,
            detail=self.detail,
            mode=self.mode,
            episode=self.episode_index,
            num_episodes=int(self.options.get("num_episodes", 0)),
            frames=len(self.samples),
            elapsed=round(elapsed, 2),
            dataset=str(self.dataset_dir) if self.dataset_dir else "",
            stamp=time.time(),
        )
        self.status_pub.publish(msg)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = EpisodeRecorder(args.config)
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
