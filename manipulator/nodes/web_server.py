#!/usr/bin/env python3
from __future__ import annotations

import argparse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
import threading
import time
from urllib.parse import urlparse

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import String

from common.config import load_config, resolve_under_root


class NativeWebNode(Node):
    def __init__(self, config_path: str | None = None):
        super().__init__("manipulator_native_web")
        self.cfg = load_config(config_path)
        topics = self.cfg["topics"]
        self.root = Path(self.cfg["_root"])
        self.data_root = resolve_under_root(self.cfg, self.cfg["recording"]["data_root"])
        self.model_root = resolve_under_root(self.cfg, self.cfg["training"]["model_root"])
        self.training_python = resolve_under_root(self.cfg, self.cfg["training"]["python"])
        self.data_root.mkdir(parents=True, exist_ok=True)
        self.model_root.mkdir(parents=True, exist_ok=True)
        self.latest_image: bytes | None = None
        self.last_camera_time = 0.0
        self.last_follower_time = 0.0
        self.last_leader_time = 0.0
        self.mode_status = {"mode": "UNKNOWN", "detail": "waiting for ROS"}
        self.record_status = {"state": "IDLE", "detail": "waiting for recorder"}
        self.model_status = {"state": "UNLOADED", "detail": "no model selected"}
        self.train_lock = threading.Lock()
        self.train_process: subprocess.Popen | None = None
        self.train_log = None
        self.train_info = {"state": "IDLE", "detail": ""}
        self.mode_pub = self.create_publisher(String, topics["mode_request"], 10)
        self.record_pub = self.create_publisher(String, topics["record_command"], 10)
        self.model_pub = self.create_publisher(String, topics["inference_model_command"], 10)
        self.create_subscription(CompressedImage, topics["camera_compressed"], self.on_image, 10)
        self.create_subscription(JointState, topics["joint_states"], self.on_follower, 10)
        self.create_subscription(JointState, "/leader/joint_states", self.on_leader, 10)
        self.create_subscription(String, topics["mode_state"], self.on_mode, 10)
        self.create_subscription(String, topics["record_state"], self.on_record, 10)
        self.create_subscription(String, topics["inference_model_state"], self.on_model, 10)
        web = self.cfg["web"]
        self.httpd = ThreadingHTTPServer(
            (str(web["host"]), int(web["port"])), self.make_handler()
        )
        self.http_thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.http_thread.start()
        self.get_logger().info(f"Native manipulator UI: http://localhost:{web['port']}")

    def on_image(self, msg: CompressedImage):
        self.latest_image = bytes(msg.data)
        self.last_camera_time = time.monotonic()

    def on_follower(self, _msg: JointState):
        self.last_follower_time = time.monotonic()

    def on_leader(self, _msg: JointState):
        self.last_leader_time = time.monotonic()

    @staticmethod
    def decode_status(msg: String):
        try:
            value = json.loads(msg.data)
            return value if isinstance(value, dict) else {"value": value}
        except json.JSONDecodeError:
            return {"detail": msg.data}

    def on_mode(self, msg: String):
        self.mode_status = self.decode_status(msg)

    def on_record(self, msg: String):
        self.record_status = self.decode_status(msg)

    def on_model(self, msg: String):
        self.model_status = self.decode_status(msg)

    @staticmethod
    def publish_json(publisher, payload: dict):
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        publisher.publish(msg)

    def list_datasets(self):
        return [
            str(path.relative_to(self.data_root))
            for path in sorted(self.data_root.glob("*/*"))
            if (path / "dataset.json").exists()
        ]

    def list_models(self):
        return [
            str(path.relative_to(self.model_root))
            for path in sorted(self.model_root.glob("*/best.pt"))
        ]

    def training_status(self):
        with self.train_lock:
            if self.train_process is not None:
                code = self.train_process.poll()
                if code is None:
                    self.train_info["state"] = "RUNNING"
                else:
                    self.train_info["state"] = "FINISHED" if code == 0 else "ERROR"
                    self.train_info["return_code"] = code
                    self.train_process = None
                    if self.train_log:
                        self.train_log.close()
                        self.train_log = None
            return dict(self.train_info)

    def start_training(self, request: dict):
        with self.train_lock:
            if self.train_process is not None and self.train_process.poll() is None:
                raise RuntimeError("training is already running")
            dataset = (self.data_root / str(request.get("dataset", ""))).resolve()
            if not dataset.is_relative_to(self.data_root) or not (dataset / "dataset.json").exists():
                raise ValueError("select a valid local dataset")
            output_name = str(request.get("output", "")).strip()
            allowed = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_.-"
            if not output_name or any(char not in allowed for char in output_name):
                raise ValueError("output must use letters, numbers, dot, underscore or dash")
            output = (self.model_root / output_name).resolve()
            if output.exists() and any(output.iterdir()):
                raise FileExistsError(f"model output already exists: {output_name}")
            if not self.training_python.exists():
                raise FileNotFoundError(
                    f"training environment not found: {self.training_python}; run setup_native.sh"
                )
            output.mkdir(parents=True, exist_ok=True)
            command = [
                str(self.training_python), str(self.root / "learning" / "train_bc.py"),
                "--dataset", str(dataset), "--output", str(output),
                "--epochs", str(int(request.get("epochs", self.cfg["training"]["default_epochs"]))),
                "--batch-size", str(int(request.get("batch_size", self.cfg["training"]["default_batch_size"]))),
                "--learning-rate", str(float(request.get("learning_rate", self.cfg["training"]["default_learning_rate"]))),
                "--width", str(int(self.cfg["training"]["image_width"])),
                "--height", str(int(self.cfg["training"]["image_height"])),
                "--device", str(request.get("device", "auto")),
            ]
            self.train_log = (output / "console.log").open("w", encoding="utf-8")
            self.train_process = subprocess.Popen(
                command, cwd=self.root, stdout=self.train_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            self.train_info = {
                "state": "RUNNING", "detail": str(output),
                "pid": self.train_process.pid, "command": command,
            }
            return dict(self.train_info)

    def make_handler(self):
        node = self

        class Handler(BaseHTTPRequestHandler):
            server_version = "BipedManipulator/1.0"

            def log_message(self, fmt, *args):
                node.get_logger().debug(fmt % args)

            def json_response(self, value, status=HTTPStatus.OK):
                body = json.dumps(value, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def read_json(self):
                length = int(self.headers.get("Content-Length", "0"))
                if length > 1_000_000:
                    raise ValueError("request too large")
                data = self.rfile.read(length) if length else b"{}"
                value = json.loads(data.decode("utf-8"))
                if not isinstance(value, dict):
                    raise ValueError("JSON body must be an object")
                return value

            def do_GET(self):
                path = urlparse(self.path).path
                if path == "/":
                    body = (node.root / "web" / "index.html").read_bytes()
                    self.send_response(HTTPStatus.OK)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                elif path == "/api/status":
                    now = time.monotonic()
                    self.json_response({
                        "mode": node.mode_status,
                        "record": node.record_status,
                        "training": node.training_status(),
                        "model": node.model_status,
                        "camera": now - node.last_camera_time < 1.0,
                        "connections": {
                            "camera": now - node.last_camera_time < 1.0,
                            "leader": now - node.last_leader_time < 1.0,
                            "follower": now - node.last_follower_time < 1.0,
                        },
                    })
                elif path == "/api/datasets":
                    self.json_response({"datasets": node.list_datasets()})
                elif path == "/api/models":
                    self.json_response({"models": node.list_models()})
                elif path == "/stream.mjpg":
                    self.stream_camera()
                else:
                    self.send_error(HTTPStatus.NOT_FOUND)

            def stream_camera(self):
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        frame = node.latest_image
                        if frame is None:
                            time.sleep(0.1)
                            continue
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                            + str(len(frame)).encode() + b"\r\n\r\n" + frame + b"\r\n"
                        )
                        time.sleep(1.0 / 20.0)
                except (BrokenPipeError, ConnectionResetError):
                    pass

            def do_POST(self):
                path = urlparse(self.path).path
                try:
                    request = self.read_json()
                    if path == "/api/mode":
                        requested = str(request.get("mode", "IDLE")).upper()
                        current = str(node.mode_status.get("mode", "UNKNOWN")).upper()
                        if (
                            requested in {"TELEOP", "RECORD", "INFERENCE"}
                            and current not in {"STABLE", "TELEOP", "RECORD", "INFERENCE"}
                        ):
                            raise RuntimeError(
                                "press Kneeling Stable and wait for STABLE before enabling the arm"
                            )
                        node.publish_json(
                            node.mode_pub,
                            {"mode": requested},
                        )
                        self.json_response({"success": True})
                    elif path == "/api/record":
                        node.publish_json(node.record_pub, request)
                        self.json_response({"success": True})
                    elif path == "/api/training/start":
                        self.json_response(
                            {"success": True, "training": node.start_training(request)}
                        )
                    elif path == "/api/inference/model":
                        model = (node.model_root / str(request.get("model", ""))).resolve()
                        if not model.is_relative_to(node.model_root) or not model.exists():
                            raise ValueError("select a valid model")
                        message = String()
                        message.data = str(model)
                        node.model_pub.publish(message)
                        self.json_response({"success": True, "model": str(model.resolve())})
                    else:
                        self.send_error(HTTPStatus.NOT_FOUND)
                except Exception as exc:
                    self.json_response(
                        {"success": False, "error": str(exc)},
                        HTTPStatus.BAD_REQUEST,
                    )

        return Handler

    def destroy_node(self):
        self.httpd.shutdown()
        self.httpd.server_close()
        if self.train_process is not None and self.train_process.poll() is None:
            self.train_process.terminate()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = NativeWebNode(args.config)
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
