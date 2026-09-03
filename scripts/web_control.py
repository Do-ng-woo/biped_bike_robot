#!/usr/bin/env python3
"""Small local web panel for biped_bike hardware, walking, and transform commands."""

import html
import json
import mimetypes
import os
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs

try:
    import rclpy
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage
except ImportError:
    rclpy = None
    Float64MultiArray = None
    JointState = None
    TFMessage = None


HOST = "127.0.0.1"
PORT = 8080
PACKAGE_ROOT = Path(__file__).resolve().parents[1]
URDF_PATH = PACKAGE_ROOT / "urdf" / "biped_bike_robot.urdf"
MESH_DIR = PACKAGE_ROOT / "meshes"


HARDWARE_COMMAND = [
    "ros2",
    "launch",
    "biped_bike_robot",
    "hardware_display.launch.py",
    "enable_opencr_imu:=true",
    "enable_imu_tf:=true",
    "use_joint_state_gui:=false",
    "publish_present_joint_states:=true",
    "enable_joint_state_commands:=false",
    "enable_trajectory_commands:=true",
    "startup_ready_posture_on_start:=true",
    "center_on_start:=false",
]


def walk_command(num_cycles: int) -> list[str]:
    return [
        "ros2",
        "run",
        "biped_bike_robot",
        "ik_walker.py",
        "--ros-args",
        "-p",
        f"num_cycles:={num_cycles}",
    ]


def transform_command(stage_duration_sec: float) -> list[str]:
    return [
        "ros2",
        "run",
        "biped_bike_robot",
        "transform_bike.py",
        "--ros-args",
        "-p",
        f"stage_duration_sec:={stage_duration_sec:.2f}",
    ]


def revert_command(stage_duration_sec: float) -> list[str]:
    return [
        "ros2",
        "run",
        "biped_bike_robot",
        "revert_bike.py",
        "--ros-args",
        "-p",
        f"stage_duration_sec:={stage_duration_sec:.2f}",
    ]


class WebBikeTeleop:
    def __init__(self):
        self.lock = threading.Lock()
        self.enabled = False
        self.pressed: set[str] = set()
        self.max_wheel_speed = 2.0
        self.min_wheel_speed = 0.1
        self.max_allowed_wheel_speed = 5.0
        self.node = None
        self.publisher = None
        self.thread = None
        self.message = "Teleop is off."

    def start(self):
        if rclpy is None or Float64MultiArray is None:
            self.message = "rclpy is not available. Source ROS before starting web_control.py."
            return
        with self.lock:
            if self.enabled:
                self.message = "Bike teleop is already on."
                return
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node("web_bike_teleop")
            self.publisher = self.node.create_publisher(
                Float64MultiArray,
                "/wheel_velocity_controller/commands",
                10,
            )
            self.enabled = True
            self.pressed.clear()
            self.thread = threading.Thread(target=self._loop, daemon=True)
            self.thread.start()
            self.message = "Bike teleop is on."

    def stop(self):
        with self.lock:
            if not self.enabled:
                self.message = "Bike teleop is already off."
                return
            self.enabled = False
            self.pressed.clear()
            publisher = self.publisher
            node = self.node
        if publisher is not None:
            self._publish(publisher, 0.0, 0.0)
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        if node is not None:
            node.destroy_node()
        with self.lock:
            self.node = None
            self.publisher = None
            self.thread = None
            self.message = "Bike teleop is off."

    def set_key(self, key: str, is_down: bool):
        key = key.lower()
        if key == "space":
            key = " "
        if key not in {"w", "a", "s", "d", "q", " "}:
            return
        with self.lock:
            if key in {"q", " "}:
                self.pressed.clear()
                self.message = "Bike teleop stop key pressed."
                return
            if is_down:
                self.pressed.add(key)
            else:
                self.pressed.discard(key)
            self.message = f"Pressed: {self._pressed_label_unlocked() or 'none'}"

    def set_speed(self, speed: float):
        with self.lock:
            self.max_wheel_speed = max(
                self.min_wheel_speed,
                min(self.max_allowed_wheel_speed, speed),
            )
            self.message = f"Teleop speed set to {self.max_wheel_speed:.2f} rad/s."

    def pressed_label(self) -> str:
        with self.lock:
            return self._pressed_label_unlocked()

    def _pressed_label_unlocked(self) -> str:
        keys = sorted("space" if key == " " else key for key in self.pressed)
        return " + ".join(keys)

    def status(self):
        with self.lock:
            return {
                "enabled": self.enabled,
                "pressed": self._pressed_label_unlocked(),
                "message": self.message,
                "speed": self.max_wheel_speed,
                "min_speed": self.min_wheel_speed,
                "max_speed": self.max_allowed_wheel_speed,
            }

    def _loop(self):
        while True:
            with self.lock:
                if not self.enabled:
                    break
                publisher = self.publisher
                pressed = set(self.pressed)
                max_speed = self.max_wheel_speed

            linear = 0.0
            angular = 0.0
            if "w" in pressed:
                linear += max_speed
            if "s" in pressed:
                linear -= max_speed
            if "a" in pressed:
                angular += max_speed
            if "d" in pressed:
                angular -= max_speed

            if publisher is not None:
                self._publish(publisher, linear, angular)
            time.sleep(0.1)

    @staticmethod
    def _publish(publisher, linear_vel: float, angular_vel: float):
        raw_l = linear_vel - angular_vel
        raw_r = linear_vel + angular_vel
        msg = Float64MultiArray()
        msg.data = [raw_l * -1.0, raw_r * 1.0]
        publisher.publish(msg)


class RosStateMonitor:
    def __init__(self):
        self.lock = threading.Lock()
        self.node = None
        self.thread = None
        self.running = False
        self.joints: dict[str, float] = {}
        self.base_translation = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.base_rotation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.last_joint_time = 0.0
        self.last_tf_time = 0.0

    def start(self):
        if rclpy is None or JointState is None or TFMessage is None:
            return
        with self.lock:
            if self.running:
                return
            if not rclpy.ok():
                rclpy.init(args=None)
            self.node = rclpy.create_node("web_robot_state_monitor")
            self.node.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                10,
            )
            self.node.create_subscription(TFMessage, "/tf", self.tf_callback, 10)
            self.running = True
            self.thread = threading.Thread(target=self._spin, daemon=True)
            self.thread.start()

    def stop(self):
        with self.lock:
            self.running = False
            node = self.node
        if node is not None:
            node.destroy_node()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        with self.lock:
            self.node = None
            self.thread = None

    def joint_state_callback(self, msg):
        now = time.time()
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                self.joints[name] = float(position)
            self.last_joint_time = now

    def tf_callback(self, msg):
        now = time.time()
        with self.lock:
            for transform in msg.transforms:
                if (
                    transform.header.frame_id == "world"
                    and transform.child_frame_id == "base_link"
                ):
                    t = transform.transform.translation
                    r = transform.transform.rotation
                    self.base_translation = {"x": t.x, "y": t.y, "z": t.z}
                    self.base_rotation = {"x": r.x, "y": r.y, "z": r.z, "w": r.w}
                    self.last_tf_time = now

    def _spin(self):
        while True:
            with self.lock:
                running = self.running
                node = self.node
            if not running or node is None:
                break
            rclpy.spin_once(node, timeout_sec=0.1)

    def snapshot(self):
        with self.lock:
            return {
                "joints": dict(self.joints),
                "base_translation": dict(self.base_translation),
                "base_rotation": dict(self.base_rotation),
                "last_joint_age_sec": time.time() - self.last_joint_time
                if self.last_joint_time
                else None,
                "last_tf_age_sec": time.time() - self.last_tf_time
                if self.last_tf_time
                else None,
            }


class ProcessRecord:
    def __init__(self, name: str, command: list[str]):
        self.name = name
        self.command = command
        self.started_at = time.time()
        self.output: list[str] = []
        self.process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
        )
        self.thread = threading.Thread(target=self._read_output, daemon=True)
        self.thread.start()

    def _read_output(self):
        if self.process.stdout is None:
            return
        for line in self.process.stdout:
            self.output.append(line.rstrip())
            if len(self.output) > 120:
                self.output = self.output[-120:]

    def is_running(self) -> bool:
        return self.process.poll() is None

    def returncode(self):
        return self.process.poll()

    def stop(self):
        if not self.is_running():
            return
        os.killpg(self.process.pid, signal.SIGTERM)
        try:
            self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)


class ControlState:
    def __init__(self):
        self.lock = threading.Lock()
        self.hardware: ProcessRecord | None = None
        self.jobs: list[ProcessRecord] = []
        self.teleop = WebBikeTeleop()
        self.monitor = RosStateMonitor()
        self.message = "Ready."

    def start_hardware(self):
        with self.lock:
            if self.hardware is not None and self.hardware.is_running():
                self.message = "Hardware bridge is already running."
                return
            self.hardware = ProcessRecord("hardware", HARDWARE_COMMAND)
            self.message = "Hardware bridge started."

    def stop_hardware(self):
        with self.lock:
            if self.hardware is None or not self.hardware.is_running():
                self.message = "Hardware bridge is not running."
                return
            self.hardware.stop()
            self.message = "Hardware bridge stopped."

    def start_job(self, name: str, command: list[str]):
        with self.lock:
            self.jobs.append(ProcessRecord(name, command))
            self.jobs = self.jobs[-8:]
            self.message = f"{name} command started."


STATE = ControlState()


def parse_vector(text: str | None, default: tuple[float, float, float]):
    if not text:
        return list(default)
    parts = [float(part) for part in text.split()]
    if len(parts) != 3:
        return list(default)
    return parts


def mesh_url(filename: str) -> str:
    name = filename.rsplit("/", 1)[-1]
    return f"/meshes/{name}"


def load_robot_model():
    root = ET.parse(URDF_PATH).getroot()
    links = {}
    joints = []

    for link in root.findall("link"):
        name = link.attrib["name"]
        visual = link.find("visual")
        visual_data = None
        if visual is not None:
            origin = visual.find("origin")
            mesh = visual.find("geometry/mesh")
            if mesh is not None:
                visual_data = {
                    "xyz": parse_vector(
                        origin.attrib.get("xyz") if origin is not None else None,
                        (0.0, 0.0, 0.0),
                    ),
                    "rpy": parse_vector(
                        origin.attrib.get("rpy") if origin is not None else None,
                        (0.0, 0.0, 0.0),
                    ),
                    "mesh": mesh_url(mesh.attrib["filename"]),
                }
        links[name] = {"name": name, "visual": visual_data}

    for joint in root.findall("joint"):
        origin = joint.find("origin")
        parent = joint.find("parent")
        child = joint.find("child")
        axis = joint.find("axis")
        if parent is None or child is None:
            continue
        joints.append(
            {
                "name": joint.attrib["name"],
                "type": joint.attrib.get("type", "fixed"),
                "parent": parent.attrib["link"],
                "child": child.attrib["link"],
                "xyz": parse_vector(
                    origin.attrib.get("xyz") if origin is not None else None,
                    (0.0, 0.0, 0.0),
                ),
                "rpy": parse_vector(
                    origin.attrib.get("rpy") if origin is not None else None,
                    (0.0, 0.0, 0.0),
                ),
                "axis": parse_vector(
                    axis.attrib.get("xyz") if axis is not None else None,
                    (1.0, 0.0, 0.0),
                ),
            }
        )

    return {"root": "base_link", "links": links, "joints": joints}


ROBOT_MODEL = load_robot_model()


def clamp_int(value: str, minimum: int, maximum: int) -> int:
    parsed = int(value)
    return max(minimum, min(maximum, parsed))


def clamp_float(value: str, minimum: float, maximum: float) -> float:
    parsed = float(value)
    return max(minimum, min(maximum, parsed))


def format_command(command: list[str]) -> str:
    return " ".join(html.escape(part) for part in command)


class ControlHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/robot_model.json":
            self._send_json(ROBOT_MODEL)
            return
        if self.path == "/state.json":
            self._send_json(STATE.monitor.snapshot())
            return
        if self.path.startswith("/meshes/"):
            self._send_mesh(self.path.removeprefix("/meshes/"))
            return
        self._send_html()

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8")
        form = parse_qs(body)

        try:
            if self.path == "/hardware/start":
                STATE.start_hardware()
            elif self.path == "/hardware/stop":
                STATE.stop_hardware()
            elif self.path == "/walk":
                cycles = clamp_int(form.get("num_cycles", ["1"])[0], 1, 50)
                STATE.start_job(f"walk {cycles} cycle(s)", walk_command(cycles))
            elif self.path == "/transform":
                seconds = clamp_float(
                    form.get("stage_duration_sec", ["5.0"])[0],
                    0.5,
                    30.0,
                )
                STATE.start_job(
                    f"transform {seconds:.1f}s/stage",
                    transform_command(seconds),
                )
            elif self.path == "/revert":
                seconds = clamp_float(
                    form.get("stage_duration_sec", ["5.0"])[0],
                    0.5,
                    30.0,
                )
                STATE.start_job(
                    f"revert {seconds:.1f}s/stage",
                    revert_command(seconds),
                )
            elif self.path == "/teleop/start":
                STATE.teleop.start()
                STATE.message = STATE.teleop.message
            elif self.path == "/teleop/stop":
                STATE.teleop.stop()
                STATE.message = STATE.teleop.message
            elif self.path == "/teleop/speed":
                speed = clamp_float(form.get("speed", ["2.0"])[0], 0.1, 5.0)
                STATE.teleop.set_speed(speed)
                STATE.message = STATE.teleop.message
            elif self.path == "/teleop/key":
                key = form.get("key", [""])[0]
                action = form.get("action", [""])[0]
                STATE.teleop.set_key(key, action == "down")
                STATE.message = STATE.teleop.message
            else:
                STATE.message = f"Unknown action: {self.path}"
        except Exception as exc:  # keep the panel alive for bad form input
            STATE.message = f"Command failed to start: {exc}"

        self.send_response(303)
        self.send_header("Location", "/")
        self.end_headers()

    def _send_html(self):
        with STATE.lock:
            hardware = STATE.hardware
            jobs = list(STATE.jobs)
            message = STATE.message
            teleop_status = STATE.teleop.status()

        hardware_running = hardware is not None and hardware.is_running()
        hardware_status = "ON" if hardware_running else "OFF"
        hardware_class = "on" if hardware_running else "off"
        hardware_log = "\n".join(hardware.output[-40:]) if hardware else ""
        teleop_enabled = teleop_status["enabled"]
        teleop_class = "on" if teleop_enabled else "off"
        teleop_label = "ON" if teleop_enabled else "OFF"
        pressed_keys = teleop_status["pressed"] or "none"
        teleop_speed = float(teleop_status["speed"])
        teleop_min_speed = float(teleop_status["min_speed"])
        teleop_max_speed = float(teleop_status["max_speed"])

        job_html = []
        for job in reversed(jobs):
            status = "running" if job.is_running() else f"done ({job.returncode()})"
            output = "\n".join(job.output[-30:])
            job_html.append(
                f"""
                <section class="job">
                  <div class="job-head">
                    <strong>{html.escape(job.name)}</strong>
                    <span>{html.escape(status)}</span>
                  </div>
                  <code>{format_command(job.command)}</code>
                  <pre>{html.escape(output)}</pre>
                </section>
                """
            )

        page = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Biped Bike Control</title>
  <style>
    :root {{
      color-scheme: light;
      --bg: #f6f7f9;
      --panel: #ffffff;
      --text: #20242a;
      --muted: #667085;
      --line: #d8dde6;
      --accent: #1f7a5a;
      --danger: #b42318;
      --blue: #2459a6;
      --key: #f1f5f9;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    main {{
      width: min(1720px, calc(100vw - 24px));
      height: calc(100vh - 24px);
      margin: 12px auto;
      display: grid;
      grid-template-columns: minmax(640px, 1fr) 430px;
      gap: 12px;
      align-items: stretch;
    }}
    h1 {{ font-size: 22px; margin: 0; }}
    h2 {{ font-size: 16px; margin: 0 0 12px; }}
    .rviz-panel {{
      min-height: 0;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 10px;
    }}
    .rviz-head {{
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 12px;
    }}
    .rviz-frame {{
      width: 100%;
      min-height: 0;
      height: 100%;
      border: 1px solid var(--line);
      border-radius: 8px;
      overflow: hidden;
      background: #101828;
      position: relative;
    }}
    .rviz-frame canvas {{
      width: 100%;
      height: 100%;
      display: block;
      background: #101828;
      cursor: grab;
      touch-action: none;
    }}
    .rviz-note {{
      position: absolute;
      left: 14px;
      bottom: 14px;
      right: 14px;
      color: #d0d5dd;
      font-size: 12px;
      line-height: 1.45;
      pointer-events: none;
      text-shadow: 0 1px 2px rgba(0,0,0,.45);
    }}
    .controls {{
      min-height: 0;
      display: grid;
      grid-template-rows: auto 1fr;
      gap: 10px;
      overflow: hidden;
    }}
    .control-scroll {{
      min-height: 0;
      overflow: auto;
      display: grid;
      gap: 10px;
      align-content: start;
      padding-right: 2px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
    }}
    .wide-grid {{
      display: grid;
      grid-template-columns: 1fr;
      gap: 10px;
    }}
    section {{
      background: var(--panel);
      border: 1px solid var(--line);
      border-radius: 8px;
      padding: 14px;
    }}
    .status {{
      display: inline-flex;
      align-items: center;
      height: 28px;
      padding: 0 10px;
      border-radius: 999px;
      font-weight: 700;
      margin-bottom: 12px;
    }}
    .status.on {{ background: #dff6eb; color: var(--accent); }}
    .status.off {{ background: #fee4e2; color: var(--danger); }}
    form {{ display: flex; gap: 8px; align-items: end; flex-wrap: wrap; }}
    label {{ display: grid; gap: 6px; color: var(--muted); font-size: 13px; }}
    input {{
      width: 120px;
      height: 38px;
      border: 1px solid var(--line);
      border-radius: 6px;
      padding: 0 10px;
      font-size: 15px;
    }}
    button {{
      height: 38px;
      border: 0;
      border-radius: 6px;
      padding: 0 14px;
      color: white;
      background: var(--blue);
      font-weight: 700;
      cursor: pointer;
    }}
    button.stop {{ background: var(--danger); }}
    button.start {{ background: var(--accent); }}
    .message {{
      margin: 12px 0;
      color: var(--muted);
    }}
    .logs {{
      display: grid;
      gap: 10px;
    }}
    .job-head {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      margin-bottom: 8px;
    }}
    code {{
      display: block;
      color: #344054;
      font-size: 12px;
      overflow-wrap: anywhere;
      margin-bottom: 8px;
    }}
    pre {{
      min-height: 80px;
      max-height: 260px;
      overflow: auto;
      margin: 0;
      padding: 10px;
      background: #101828;
      color: #e6edf3;
      border-radius: 6px;
      font-size: 12px;
      line-height: 1.4;
      white-space: pre-wrap;
    }}
    .keys {{
      display: grid;
      grid-template-columns: repeat(3, 48px);
      gap: 6px;
      justify-content: start;
      margin-top: 8px;
    }}
    .key {{
      height: 42px;
      display: grid;
      place-items: center;
      border: 1px solid var(--line);
      border-radius: 6px;
      background: var(--key);
      font-weight: 800;
    }}
    .key.active {{
      background: #dbeafe;
      border-color: var(--blue);
      color: var(--blue);
    }}
    .key.blank {{ visibility: hidden; }}
    .hint {{
      margin: 8px 0 0;
      color: var(--muted);
      font-size: 13px;
      line-height: 1.5;
    }}
    .metric {{
      margin: 8px 0;
      color: var(--muted);
      font-size: 13px;
    }}
    @media (max-width: 760px) {{
      main {{
        height: auto;
        grid-template-columns: 1fr;
      }}
      .rviz-frame {{ height: 58vh; }}
      input {{ width: 100%; }}
      form {{ align-items: stretch; }}
      button {{ width: 100%; }}
    }}
  </style>
  <script>
    const activeKeys = new Set();
    const keyMap = new Map([
      ['w', 'w'], ['a', 'a'], ['s', 's'], ['d', 'd'],
      ['q', 'q'], [' ', 'space']
    ]);

    async function sendKey(key, action) {{
      const body = new URLSearchParams({{ key, action }});
      await fetch('/teleop/key', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
        body
      }});
    }}

    function renderKeys() {{
      document.querySelectorAll('[data-key]').forEach((node) => {{
        node.classList.toggle('active', activeKeys.has(node.dataset.key));
      }});
      const pressed = Array.from(activeKeys).sort().join(' + ') || 'none';
      const label = document.getElementById('pressedKeys');
      if (label) label.textContent = pressed;
    }}

    window.addEventListener('keydown', (event) => {{
      const mapped = keyMap.get(event.key.toLowerCase());
      if (!mapped || event.repeat) return;
      event.preventDefault();
      if (mapped === 'q' || mapped === 'space') {{
        activeKeys.clear();
        sendKey(mapped, 'down');
        renderKeys();
        return;
      }}
      activeKeys.add(mapped);
      sendKey(mapped, 'down');
      renderKeys();
    }});

    window.addEventListener('keyup', (event) => {{
      const mapped = keyMap.get(event.key.toLowerCase());
      if (!mapped) return;
      event.preventDefault();
      activeKeys.delete(mapped);
      sendKey(mapped, 'up');
      renderKeys();
    }});

    window.addEventListener('blur', () => {{
      for (const key of Array.from(activeKeys)) sendKey(key, 'up');
      activeKeys.clear();
      renderKeys();
    }});
  </script>
  <script type="module">
    import * as THREE from 'https://unpkg.com/three@0.160.0/build/three.module.js';

    const viewer = {{
      scene: null,
      camera: null,
      renderer: null,
      robotRoot: null,
      linkGroups: new Map(),
      joints: [],
      ready: false,
      target: new THREE.Vector3(0, 0, 0.08),
      orbit: {{
        distance: 1.15,
        yaw: Math.PI,
        pitch: 0.28,
        dragging: false,
        lastX: 0,
        lastY: 0
      }}
    }};

    function clamp(value, minimum, maximum) {{
      return Math.max(minimum, Math.min(maximum, value));
    }}

    function updateCamera() {{
      if (!viewer.camera) return;
      const orbit = viewer.orbit;
      const cp = Math.cos(orbit.pitch);
      const sp = Math.sin(orbit.pitch);
      viewer.camera.up.set(0, 0, 1);
      viewer.camera.position.set(
        viewer.target.x + orbit.distance * cp * Math.cos(orbit.yaw),
        viewer.target.y + orbit.distance * cp * Math.sin(orbit.yaw),
        viewer.target.z + orbit.distance * sp
      );
      viewer.camera.lookAt(viewer.target);
    }}

    function resetViewerCamera() {{
      viewer.target.set(0, 0, 0.08);
      viewer.orbit.distance = 1.15;
      viewer.orbit.yaw = Math.PI;
      viewer.orbit.pitch = 0.28;
      updateCamera();
    }}

    function setupViewerControls(canvas) {{
      canvas.addEventListener('pointerdown', (event) => {{
        viewer.orbit.dragging = true;
        viewer.orbit.lastX = event.clientX;
        viewer.orbit.lastY = event.clientY;
        canvas.setPointerCapture(event.pointerId);
        canvas.style.cursor = 'grabbing';
      }});

      canvas.addEventListener('pointermove', (event) => {{
        if (!viewer.orbit.dragging) return;
        const dx = event.clientX - viewer.orbit.lastX;
        const dy = event.clientY - viewer.orbit.lastY;
        viewer.orbit.lastX = event.clientX;
        viewer.orbit.lastY = event.clientY;
        viewer.orbit.yaw -= dx * 0.006;
        viewer.orbit.pitch = clamp(viewer.orbit.pitch + dy * 0.006, -1.35, 1.35);
        updateCamera();
      }});

      function stopDrag(event) {{
        viewer.orbit.dragging = false;
        canvas.style.cursor = 'grab';
        if (event && canvas.hasPointerCapture(event.pointerId)) {{
          canvas.releasePointerCapture(event.pointerId);
        }}
      }}

      canvas.addEventListener('pointerup', stopDrag);
      canvas.addEventListener('pointercancel', stopDrag);
      canvas.addEventListener('wheel', (event) => {{
        event.preventDefault();
        viewer.orbit.distance = clamp(
          viewer.orbit.distance * Math.exp(event.deltaY * 0.001),
          0.18,
          4.0
        );
        updateCamera();
      }}, {{ passive: false }});
      canvas.addEventListener('dblclick', resetViewerCamera);
    }}

    function qFromRpy(rpy) {{
      const roll = rpy[0];
      const pitch = rpy[1];
      const yaw = rpy[2];
      const cr = Math.cos(roll * 0.5);
      const sr = Math.sin(roll * 0.5);
      const cp = Math.cos(pitch * 0.5);
      const sp = Math.sin(pitch * 0.5);
      const cy = Math.cos(yaw * 0.5);
      const sy = Math.sin(yaw * 0.5);
      return new THREE.Quaternion(
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy
      ).normalize();
    }}

    function parseBinaryStl(buffer) {{
      const view = new DataView(buffer);
      const faceCount = view.getUint32(80, true);
      const positions = new Float32Array(faceCount * 9);
      const normals = new Float32Array(faceCount * 9);
      let offset = 84;
      for (let face = 0; face < faceCount; face++) {{
        const nx = view.getFloat32(offset, true);
        const ny = view.getFloat32(offset + 4, true);
        const nz = view.getFloat32(offset + 8, true);
        offset += 12;
        for (let vertex = 0; vertex < 3; vertex++) {{
          const base = face * 9 + vertex * 3;
          positions[base] = view.getFloat32(offset, true);
          positions[base + 1] = view.getFloat32(offset + 4, true);
          positions[base + 2] = view.getFloat32(offset + 8, true);
          normals[base] = nx;
          normals[base + 1] = ny;
          normals[base + 2] = nz;
          offset += 12;
        }}
        offset += 2;
      }}
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
      geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
      geometry.computeBoundingSphere();
      return geometry;
    }}

    async function loadStl(url) {{
      const response = await fetch(url);
      if (!response.ok) throw new Error(`mesh load failed: ${{url}}`);
      return parseBinaryStl(await response.arrayBuffer());
    }}

    async function initRobotViewer() {{
      const canvas = document.getElementById('robotViewer');
      const frame = canvas.parentElement;
      viewer.scene = new THREE.Scene();
      viewer.scene.background = new THREE.Color(0x252525);
      viewer.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 10);
      viewer.camera.up.set(0, 0, 1);
      resetViewerCamera();
      viewer.renderer = new THREE.WebGLRenderer({{ canvas, antialias: true }});
      viewer.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
      setupViewerControls(canvas);

      const hemi = new THREE.HemisphereLight(0xffffff, 0x2f3440, 1.8);
      viewer.scene.add(hemi);
      const dir = new THREE.DirectionalLight(0xffffff, 1.2);
      dir.position.set(1, -1, 1);
      viewer.scene.add(dir);
      const grid = new THREE.GridHelper(1.2, 24, 0x6b7280, 0x475467);
      grid.rotation.x = Math.PI / 2;
      viewer.scene.add(grid);

      viewer.robotRoot = new THREE.Group();
      viewer.scene.add(viewer.robotRoot);

      const model = await (await fetch('/robot_model.json')).json();
      const material = new THREE.MeshStandardMaterial({{
        color: 0xb8bec8,
        metalness: 0.15,
        roughness: 0.55
      }});

      for (const [name, link] of Object.entries(model.links)) {{
        const group = new THREE.Group();
        group.name = name;
        viewer.linkGroups.set(name, group);
        if (link.visual && link.visual.mesh) {{
          const geometry = await loadStl(link.visual.mesh);
          const mesh = new THREE.Mesh(geometry, material);
          mesh.position.fromArray(link.visual.xyz);
          mesh.quaternion.copy(qFromRpy(link.visual.rpy));
          group.add(mesh);
        }}
      }}

      viewer.robotRoot.add(viewer.linkGroups.get(model.root));
      for (const joint of model.joints) {{
        const parent = viewer.linkGroups.get(joint.parent);
        const child = viewer.linkGroups.get(joint.child);
        if (!parent || !child) continue;
        parent.add(child);
        joint.group = child;
        joint.originQ = qFromRpy(joint.rpy);
        joint.axisV = new THREE.Vector3(...joint.axis).normalize();
        viewer.joints.push(joint);
      }}

      viewer.ready = true;
      document.getElementById('viewerStatus').textContent =
        'Web robot view running. Drag: orbit, wheel: zoom, double click: reset.';
      resizeViewer();
      animateViewer();
      pollRobotState();
    }}

    function resizeViewer() {{
      if (!viewer.renderer) return;
      const canvas = viewer.renderer.domElement;
      const rect = canvas.parentElement.getBoundingClientRect();
      viewer.camera.aspect = Math.max(1, rect.width) / Math.max(1, rect.height);
      viewer.camera.updateProjectionMatrix();
      viewer.renderer.setSize(rect.width, rect.height, false);
    }}

    function applyRobotState(state) {{
      if (!viewer.ready) return;
      viewer.robotRoot.position.set(
        state.base_translation?.x || 0,
        state.base_translation?.y || 0,
        state.base_translation?.z || 0
      );
      const q = state.base_rotation || {{ x: 0, y: 0, z: 0, w: 1 }};
      viewer.robotRoot.quaternion.set(q.x, q.y, q.z, q.w);

      const joints = state.joints || {{}};
      for (const joint of viewer.joints) {{
        const angle = Number(joints[joint.name] || 0);
        joint.group.position.fromArray(joint.xyz);
        const jointQ = new THREE.Quaternion();
        if (joint.type !== 'fixed') {{
          jointQ.setFromAxisAngle(joint.axisV, angle);
        }}
        joint.group.quaternion.copy(joint.originQ).multiply(jointQ).normalize();
      }}
    }}

    async function pollRobotState() {{
      try {{
        const state = await (await fetch('/state.json', {{ cache: 'no-store' }})).json();
        applyRobotState(state);
      }} catch (error) {{
        document.getElementById('viewerStatus').textContent = `state wait: ${{error.message}}`;
      }} finally {{
        setTimeout(pollRobotState, 80);
      }}
    }}

    function animateViewer() {{
      requestAnimationFrame(animateViewer);
      resizeViewer();
      updateCamera();
      viewer.renderer.render(viewer.scene, viewer.camera);
    }}

    initRobotViewer().catch((error) => {{
      document.getElementById('viewerStatus').textContent = `viewer error: ${{error.message}}`;
    }});
  </script>
</head>
<body>
  <main>
    <div class="rviz-panel">
      <div class="rviz-head">
        <h1>Biped Bike Control</h1>
        <span class="status {hardware_class}">{hardware_status}</span>
      </div>
      <div class="rviz-frame">
        <canvas id="robotViewer"></canvas>
        <div class="rviz-note" id="viewerStatus">Loading web robot view...</div>
      </div>
    </div>
    <aside class="controls">
      <div>
        <p class="message">{html.escape(message)}</p>
      </div>
      <div class="control-scroll">
        <div class="grid">
          <section>
            <h2>1. Hardware</h2>
            <form method="post" action="/hardware/start">
              <button class="start" type="submit">ON</button>
            </form>
            <form method="post" action="/hardware/stop" style="margin-top:8px">
              <button class="stop" type="submit">OFF</button>
            </form>
          </section>
          <section>
            <h2>2. Walk</h2>
            <form method="post" action="/walk">
              <label>cycles
                <input name="num_cycles" type="number" min="1" max="50" value="1">
              </label>
              <button type="submit">Run Walk</button>
            </form>
          </section>
          <section>
            <h2>3. Transform</h2>
            <form method="post" action="/transform">
              <label>sec/stage
                <input name="stage_duration_sec" type="number" min="0.5" max="30" step="0.5" value="5.0">
              </label>
              <button type="submit">Run Transform</button>
            </form>
            <form method="post" action="/revert" style="margin-top:8px">
              <label>sec/stage
                <input name="stage_duration_sec" type="number" min="0.5" max="30" step="0.5" value="5.0">
              </label>
              <button type="submit">Run Revert</button>
            </form>
          </section>
        </div>
        <div class="wide-grid">
          <section>
            <h2>4. Bike Teleop</h2>
            <div class="status {teleop_class}">{teleop_label}</div>
            <form method="post" action="/teleop/start">
              <button class="start" type="submit">Teleop ON</button>
            </form>
            <form method="post" action="/teleop/stop" style="margin-top:8px">
              <button class="stop" type="submit">Teleop OFF</button>
            </form>
            <form method="post" action="/teleop/speed" style="margin-top:12px">
              <label>speed rad/s
                <input name="speed" type="number" min="{teleop_min_speed:.1f}" max="{teleop_max_speed:.1f}" step="0.1" value="{teleop_speed:.1f}">
              </label>
              <button type="submit">Set Speed</button>
            </form>
            <p class="metric">Speed range: {teleop_min_speed:.1f} to {teleop_max_speed:.1f} rad/s</p>
            <p class="hint">Current pressed: <strong id="pressedKeys">{html.escape(pressed_keys)}</strong></p>
          </section>
          <section>
            <h2>Keyboard Guide</h2>
            <div class="keys">
              <div class="key blank"></div>
              <div class="key" data-key="w">W</div>
              <div class="key blank"></div>
              <div class="key" data-key="a">A</div>
              <div class="key" data-key="s">S</div>
              <div class="key" data-key="d">D</div>
            </div>
            <p class="hint">
              W/S: forward/backward, A/D: turn left/right, Q or Space: stop wheels.
              Current teleop speed is {teleop_speed:.1f} rad/s.
              Click anywhere on this page, then use the keyboard.
            </p>
          </section>
        </div>
        <div class="logs">
          <section>
            <div class="job-head">
              <strong>Hardware Log</strong>
              <span>{hardware_status}</span>
            </div>
            <code>{format_command(HARDWARE_COMMAND)}</code>
            <pre>{html.escape(hardware_log)}</pre>
          </section>
          {''.join(job_html)}
        </div>
      </div>
    </aside>
  </main>
</body>
</html>"""
        encoded = page.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_json(self, data):
        encoded = json.dumps(data).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_mesh(self, name: str):
        safe_name = os.path.basename(name)
        path = MESH_DIR / safe_name
        if not path.exists() or not path.is_file():
            self.send_error(404, "Mesh not found")
            return
        content_type = mimetypes.guess_type(str(path))[0] or "application/octet-stream"
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "public, max-age=3600")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt, *args):
        return


def main():
    STATE.monitor.start()
    server = ThreadingHTTPServer((HOST, PORT), ControlHandler)
    print(f"Biped Bike web control: http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STATE.stop_hardware()
        STATE.teleop.stop()
        STATE.monitor.stop()
        server.server_close()


if __name__ == "__main__":
    main()
