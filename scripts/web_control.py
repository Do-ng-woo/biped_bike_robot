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
from urllib.parse import parse_qs, urlparse
from urllib.request import Request, urlopen

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


HOST = "0.0.0.0"
PORT = 8080
SOURCE_ROOT = Path(__file__).resolve().parents[1]
if (SOURCE_ROOT / "urdf" / "biped_bike_robot.urdf").is_file():
    PACKAGE_ROOT = SOURCE_ROOT
else:
    try:
        from ament_index_python.packages import get_package_share_directory

        PACKAGE_ROOT = Path(get_package_share_directory("biped_bike_robot"))
    except (ImportError, LookupError):
        PACKAGE_ROOT = SOURCE_ROOT
URDF_PATH = PACKAGE_ROOT / "urdf" / "biped_bike_robot.urdf"
MESH_DIR = PACKAGE_ROOT / "meshes"
RL_ROOT = PACKAGE_ROOT / "RL_walking"
MANIPULATOR_ROOT = PACKAGE_ROOT / "manipulator"
RL_CONTROL_URL = "http://127.0.0.1:8081"
OPENCR_DEVICE = Path("/dev/opencr")
OPENRB_DEVICE = Path(
    "/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_"
    "AF46F26750323952582E3120FF070E1E-if00"
)
CAMERA_DEVICE = Path("/dev/video4")


RL_COMMAND = [
    "/usr/bin/python3",
    "-u",
    str(RL_ROOT / "run_rl_stack.py"),
]

MANIPULATOR_COMMAND = [
    str(MANIPULATOR_ROOT / "run_native.sh"),
    "--hardware",
    "--leader",
    "--camera",
    "--ready-on-start",
]


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
        self.joint_rate_hz = 0.0
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
            self.node.create_subscription(
                JointState,
                "/biped_rl/joint_states",
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
            if self.last_joint_time:
                interval = now - self.last_joint_time
                if interval > 0.0:
                    instant_rate = min(1000.0, 1.0 / interval)
                    self.joint_rate_hz = (
                        instant_rate
                        if self.joint_rate_hz == 0.0
                        else self.joint_rate_hz * 0.8 + instant_rate * 0.2
                    )
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
                "joint_rate_hz": self.joint_rate_hz,
                "last_tf_age_sec": time.time() - self.last_tf_time
                if self.last_tf_time
                else None,
            }


class ProcessRecord:
    def __init__(
        self,
        name: str,
        command: list[str],
        *,
        supervisor: bool = False,
    ):
        self.name = name
        self.command = command
        self.started_at = time.time()
        self.output: list[str] = []
        self.supervisor = supervisor
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
        if self.supervisor:
            self.process.terminate()
        else:
            os.killpg(self.process.pid, signal.SIGTERM)
        try:
            self.process.wait(timeout=12.0 if self.supervisor else 5.0)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL)


class ControlState:
    def __init__(self):
        self.lock = threading.Lock()
        self.hardware: ProcessRecord | None = None
        self.rl_walking: ProcessRecord | None = None
        self.manipulator: ProcessRecord | None = None
        self.jobs: list[ProcessRecord] = []
        self.teleop = WebBikeTeleop()
        self.monitor = RosStateMonitor()
        self.message = "Ready."

    def start_hardware(self):
        with self.lock:
            if self._exclusive_owner_unlocked() not in {None, "hardware"}:
                self.message = (
                    "Stop RL Walking or Manipulator first; OpenCR has one owner."
                )
                return
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
            if self._exclusive_owner_unlocked() in {"rl_walking", "manipulator"}:
                self.message = "Stop RL Walking or Manipulator before legacy motion."
                return
            self.jobs.append(ProcessRecord(name, command))
            self.jobs = self.jobs[-8:]
            self.message = f"{name} command started."

    def _exclusive_owner_unlocked(self) -> str | None:
        for name in ("hardware", "rl_walking", "manipulator"):
            process = getattr(self, name)
            if process is not None and process.is_running():
                return name
        return None

    def start_rl_walking(self):
        with self.lock:
            owner = self._exclusive_owner_unlocked()
            if owner == "rl_walking":
                self.message = "RL Walking is already running."
                return
            if owner is not None:
                self.message = f"Stop {owner} first; OpenCR has one owner."
                return
            if any(job.is_running() for job in self.jobs):
                self.message = "Wait for the current motion job to finish first."
                return
            if self.teleop.status()["enabled"]:
                self.message = "Turn Bike Teleop off before RL Walking."
                return
            self.rl_walking = ProcessRecord(
                "RL walking",
                RL_COMMAND,
                supervisor=True,
            )
            self.message = (
                "RL Walking started. Wait for READY and IMU calibration, then hold a key."
            )

    def stop_rl_walking(self):
        with self.lock:
            process = self.rl_walking
        if process is None or not process.is_running():
            with self.lock:
                self.message = "RL Walking is not running."
            return
        try:
            self.send_rl_command("stop")
        except OSError:
            pass
        process.stop()
        with self.lock:
            self.message = "RL Walking stopped; OpenCR released after READY return."

    def send_rl_command(self, command: str):
        if command not in {"stop", "forward", "left", "right", "ccw", "cw"}:
            raise ValueError(f"Invalid RL command: {command}")
        with self.lock:
            running = self.rl_walking is not None and self.rl_walking.is_running()
        if not running:
            raise RuntimeError("RL Walking is not running")
        request = Request(f"{RL_CONTROL_URL}/command/{command}", method="POST")
        with urlopen(request, timeout=0.3) as response:
            response.read()

    def start_manipulator(self):
        with self.lock:
            owner = self._exclusive_owner_unlocked()
            if owner == "manipulator":
                self.message = "Manipulator is already running."
                return
            if owner is not None:
                self.message = f"Stop {owner} first; OpenCR has one owner."
                return
            if any(job.is_running() for job in self.jobs):
                self.message = "Wait for the current motion job to finish first."
                return
            if self.teleop.status()["enabled"]:
                self.message = "Turn Bike Teleop off before Manipulator mode."
                return
            self.manipulator = ProcessRecord(
                "manipulator",
                MANIPULATOR_COMMAND,
            )
            self.message = "Manipulator hardware, leader, camera, and UI started."

    def stop_manipulator(self):
        with self.lock:
            process = self.manipulator
        if process is None or not process.is_running():
            with self.lock:
                self.message = "Manipulator is not running."
            return
        process.stop()
        with self.lock:
            self.message = "Manipulator stopped and ports released."

    def stop_all(self):
        self.stop_rl_walking()
        self.stop_manipulator()
        self.stop_hardware()
        self.teleop.stop()

    def system_status(self):
        with self.lock:
            hardware = self.hardware is not None and self.hardware.is_running()
            rl_walking = self.rl_walking is not None and self.rl_walking.is_running()
            manipulator = self.manipulator is not None and self.manipulator.is_running()
            message = self.message
        monitor = self.monitor.snapshot()
        manipulator_detail = None
        manipulator_ui_ready = False
        try:
            request = Request("http://127.0.0.1:8000/api/status")
            with urlopen(request, timeout=0.12) as response:
                manipulator_detail = json.loads(response.read().decode("utf-8"))
            manipulator_ui_ready = True
        except (OSError, ValueError, json.JSONDecodeError):
            pass
        age = monitor["last_joint_age_sec"]
        feedback = age is not None and age < 1.0
        if rl_walking:
            mode, owner = "RL 보행", "RL 제어기"
        elif manipulator or manipulator_ui_ready:
            mode, owner = "매니퓰레이터", "매니퓰레이터"
        elif hardware:
            mode, owner = "기본 하드웨어", "ROS 브리지"
        else:
            mode, owner = "대기", "없음"
        return {
            "mode": mode,
            "opencr_owner": owner,
            "joint_feedback": feedback,
            "joint_age_sec": age,
            "joint_rate_hz": monitor["joint_rate_hz"],
            "hardware": hardware,
            "rl_walking": rl_walking,
            "manipulator": manipulator,
            "manipulator_ui_ready": manipulator_ui_ready,
            "manipulator_detail": manipulator_detail,
            "devices": {
                "opencr": OPENCR_DEVICE.exists(),
                "openrb": OPENRB_DEVICE.exists(),
                "camera": CAMERA_DEVICE.exists(),
            },
            "message": message,
        }


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
        path = urlparse(self.path).path
        if path == "/robot_model.json":
            self._send_json(ROBOT_MODEL)
            return
        if path == "/state.json":
            self._send_json(STATE.monitor.snapshot())
            return
        if path == "/system_status.json":
            self._send_json(STATE.system_status())
            return
        if path.startswith("/meshes/"):
            self._send_mesh(path.removeprefix("/meshes/"))
            return
        if path == "/manipulator":
            self._send_manipulator_html()
            return
        self._send_html(viewer_only=path == "/robot-view")

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
            elif self.path == "/rl/start":
                STATE.start_rl_walking()
            elif self.path == "/rl/stop":
                STATE.stop_rl_walking()
            elif self.path == "/rl/command":
                STATE.send_rl_command(form.get("command", ["stop"])[0])
            elif self.path == "/manipulator/start":
                STATE.start_manipulator()
            elif self.path == "/manipulator/stop":
                STATE.stop_manipulator()
            else:
                STATE.message = f"Unknown action: {self.path}"
        except Exception as exc:  # keep the panel alive for bad form input
            STATE.message = f"Command failed to start: {exc}"

        self.send_response(303)
        redirect = "/manipulator" if self.path.startswith("/manipulator/") else "/"
        self.send_header("Location", redirect)
        self.end_headers()

    def _send_html(self, viewer_only: bool = False):
        with STATE.lock:
            hardware = STATE.hardware
            rl_walking = STATE.rl_walking
            manipulator = STATE.manipulator
            jobs = list(STATE.jobs)
            message = STATE.message
            teleop_status = STATE.teleop.status()
        monitor_snapshot = STATE.monitor.snapshot()

        hardware_running = hardware is not None and hardware.is_running()
        hardware_status = "ON" if hardware_running else "OFF"
        hardware_class = "on" if hardware_running else "off"
        hardware_log = "\n".join(hardware.output[-40:]) if hardware else ""
        rl_running = rl_walking is not None and rl_walking.is_running()
        rl_status = "ON" if rl_running else "OFF"
        rl_class = "on" if rl_running else "off"
        rl_log = "\n".join(rl_walking.output[-50:]) if rl_walking else ""
        rl_controls_active = rl_running and not viewer_only
        manipulator_running = manipulator is not None and manipulator.is_running()
        joint_age = monitor_snapshot["last_joint_age_sec"]
        feedback_online = joint_age is not None and joint_age < 1.0
        if rl_running:
            operation_mode = "RL 보행"
            opencr_owner = "RL 제어기"
        elif manipulator_running:
            operation_mode = "매니퓰레이터"
            opencr_owner = "매니퓰레이터"
        elif hardware_running:
            operation_mode = "기본 하드웨어"
            opencr_owner = "ROS 브리지"
        else:
            operation_mode = "대기"
            opencr_owner = "없음"
        joint_rate_hz = float(monitor_snapshot.get("joint_rate_hz", 0.0))
        feedback_text = f"{joint_rate_hz:.1f} Hz" if feedback_online else "대기"
        feedback_class = "ok" if feedback_online else "wait"
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
  <title>BIPED OPS · 통합 관제</title>
  <style>
    :root {{
      color-scheme: dark;
      --bg: #081018;
      --panel: #111c28;
      --text: #f3f7fb;
      --muted: #91a4b8;
      --line: #26384b;
      --accent: #31c48d;
      --danger: #f05252;
      --blue: #2687ff;
      --key: #182737;
    }}
    * {{ box-sizing: border-box; }}
    body {{
      margin: 0;
      background: var(--bg);
      color: var(--text);
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    }}
    body.viewer-only main {{ width: 100vw; height: 100vh; margin: 0;
      padding: 12px; grid-template-columns: 1fr; }}
    body.viewer-only .controls {{ display: none; }}
    body.viewer-only .rviz-head {{ padding: 0 4px; }}
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
    .status.on {{ background: #123b31; color: #73e2ba; }}
    .status.off {{ background: #3d2428; color: #ff9b9b; }}
    form {{ display: flex; gap: 8px; align-items: end; flex-wrap: wrap; }}
    label {{ display: grid; gap: 6px; color: var(--muted); font-size: 13px; }}
    input {{
      width: 120px;
      height: 38px;
      border: 1px solid var(--line);
      background: #0a131d;
      color: var(--text);
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
      color: #9fb5ca;
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
    .overview {{ display:grid; grid-template-columns:repeat(2,1fr); gap:8px; }}
    .overview-card {{ background:var(--panel); border:1px solid var(--line);
      border-radius:8px; padding:11px 12px; }}
    .overview-card small {{ display:block; color:var(--muted); margin-bottom:4px; }}
    .overview-card strong {{ font-size:15px; }}
    .dot {{ display:inline-block; width:8px; height:8px; margin-right:6px;
      border-radius:50%; background:#98a2b3; }}
    .dot.ok {{ background:#12b76a; box-shadow:0 0 0 4px #123b31; }}
    .dot.wait {{ background:#f79009; }}
    .nav {{ display: flex; gap: 8px; align-items: center; }}
    .nav a {{ color: var(--blue); font-weight: 700; text-decoration: none; }}
    .rl-pad {{ display: grid; grid-template-columns: repeat(3, 1fr); gap: 7px; }}
    .rl-pad button {{ min-width: 0; padding: 0 6px; touch-action: none; }}
    .rl-pad button.active {{ background: #123f7a; outline: 3px solid #bfdbfe; }}
    .diagnostics {{ border:1px solid var(--line); border-radius:8px;
      background:var(--panel); overflow:hidden; }}
    .diagnostics > summary {{ padding:13px 14px; cursor:pointer; font-weight:800; }}
    .diagnostics .logs {{ padding:0 10px 10px; }}
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
    const rlActive = {str(rl_controls_active).lower()};
    const activeKeys = new Set();
    const bikeKeyMap = new Map([
      ['w', 'w'], ['a', 'a'], ['s', 's'], ['d', 'd'],
      ['q', 'q'], [' ', 'space']
    ]);
    const rlKeyMap = new Map([
      ['arrowup', 'forward'], ['w', 'forward'],
      ['arrowleft', 'left'], ['a', 'left'],
      ['arrowright', 'right'], ['d', 'right'],
      ['q', 'ccw'], ['e', 'cw'], [' ', 'stop']
    ]);
    let rlCommand = 'stop';
    let rlHeartbeat = null;

    async function sendKey(key, action) {{
      const body = new URLSearchParams({{ key, action }});
      await fetch('/teleop/key', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
        body
      }});
    }}

    async function sendRl(command) {{
      const body = new URLSearchParams({{ command }});
      await fetch('/rl/command', {{
        method: 'POST',
        headers: {{ 'Content-Type': 'application/x-www-form-urlencoded' }},
        body
      }});
    }}

    function startRl(command) {{
      if (!rlActive) return;
      stopRl(false);
      rlCommand = command;
      const hudCommand = document.getElementById('hudCommand');
      if (hudCommand) hudCommand.textContent = command.toUpperCase();
      sendRl(command);
      if (command !== 'stop') rlHeartbeat = setInterval(() => sendRl(command), 100);
      document.querySelectorAll('[data-rl-command]').forEach((node) =>
        node.classList.toggle('active', node.dataset.rlCommand === command));
    }}

    function stopRl(send = true) {{
      if (rlHeartbeat !== null) clearInterval(rlHeartbeat);
      rlHeartbeat = null;
      if (send && rlActive && rlCommand !== 'stop') sendRl('stop');
      rlCommand = 'stop';
      const hudCommand = document.getElementById('hudCommand');
      if (hudCommand) hudCommand.textContent = 'STOP';
      document.querySelectorAll('[data-rl-command]').forEach((node) =>
        node.classList.toggle('active', node.dataset.rlCommand === 'stop'));
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
      const key = event.key.toLowerCase();
      if (rlActive) {{
        const command = rlKeyMap.get(key);
        if (!command || event.repeat) return;
        event.preventDefault();
        command === 'stop' ? stopRl() : startRl(command);
        return;
      }}
      const mapped = bikeKeyMap.get(key);
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
      if (rlActive) {{
        if (rlKeyMap.has(event.key.toLowerCase())) {{
          event.preventDefault();
          stopRl();
        }}
        return;
      }}
      const mapped = bikeKeyMap.get(event.key.toLowerCase());
      if (!mapped) return;
      event.preventDefault();
      activeKeys.delete(mapped);
      sendKey(mapped, 'up');
      renderKeys();
    }});

    window.addEventListener('blur', () => {{
      stopRl();
      for (const key of Array.from(activeKeys)) sendKey(key, 'up');
      activeKeys.clear();
      renderKeys();
    }});
    document.addEventListener('visibilitychange', () => {{
      if (document.hidden) stopRl();
    }});
    window.addEventListener('DOMContentLoaded', () => {{
      document.querySelectorAll('[data-rl-command]').forEach((button) => {{
        const command = button.dataset.rlCommand;
        button.addEventListener('pointerdown', (event) => {{
          event.preventDefault();
          command === 'stop' ? stopRl() : startRl(command);
        }});
        button.addEventListener('pointerup', () => stopRl());
        button.addEventListener('pointercancel', () => stopRl());
      }});
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
      viewer.scene.background = new THREE.Color(0x12171c);
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
        'VIEWER CONNECTED · DRAG ORBIT · WHEEL ZOOM · DOUBLE CLICK RESET';
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
      const sinr = 2 * (q.w * q.x + q.y * q.z);
      const cosr = 1 - 2 * (q.x * q.x + q.y * q.y);
      const roll = Math.atan2(sinr, cosr);
      const sinp = Math.max(-1, Math.min(1, 2 * (q.w * q.y - q.z * q.x)));
      const pitch = Math.asin(sinp);
      const yaw = Math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
      );
      const signed = (value, digits=2) => `${{value >= 0 ? '+' : ''}}${{value.toFixed(digits)}}`;
      const degrees = 180 / Math.PI;
      const hudPitch = document.getElementById('hudPitch');
      if (hudPitch) {{
        hudPitch.textContent = signed(pitch * degrees) + '°';
        document.getElementById('hudRoll').textContent = signed(roll * degrees) + '°';
        document.getElementById('hudYaw').textContent = signed(yaw * degrees) + '°';
        document.getElementById('hudX').textContent = signed(state.base_translation?.x || 0, 3) + ' m';
        document.getElementById('hudY').textContent = signed(state.base_translation?.y || 0, 3) + ' m';
      }}

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

    let fpsFrames = 0;
    let fpsStarted = performance.now();
    function animateViewer(now) {{
      requestAnimationFrame(animateViewer);
      resizeViewer();
      updateCamera();
      viewer.renderer.render(viewer.scene, viewer.camera);
      fpsFrames += 1;
      if (now - fpsStarted >= 1000) {{
        const fps = document.getElementById('viewerFps');
        if (fps) fps.textContent = Math.round(fpsFrames * 1000 / (now - fpsStarted));
        fpsFrames = 0;
        fpsStarted = now;
      }}
    }}

    initRobotViewer().catch((error) => {{
      document.getElementById('viewerStatus').textContent = `viewer error: ${{error.message}}`;
    }});
  </script>
  <style>
    body.console {{ background:#0b0e11; color:#e6e9ed; overflow:hidden;
      font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif; }}
    .console-top {{ height:64px; display:flex; align-items:center; justify-content:space-between;
      padding:0 18px; border-bottom:1px solid #262d34; background:#0d1115; }}
    .console-brand {{ display:flex; align-items:baseline; gap:10px; letter-spacing:.08em; }}
    .console-brand strong {{ font-size:20px; }}
    .console-brand span {{ color:#62717e; font:11px "JetBrains Mono",monospace; }}
    .console-tabs {{ height:100%; display:flex; align-items:stretch; margin-left:26px; }}
    .console-tabs a {{ min-width:132px; display:grid; place-items:center; border-bottom:3px solid transparent;
      color:#7f8b96; text-decoration:none; font-size:13px; font-weight:750; }}
    .console-tabs a.active {{ border-color:#4da3ff; color:#eef4fa; background:#111922; }}
    .console-tabs a:hover {{ color:#fff; }}
    .status-strip {{ display:flex; align-items:center; gap:18px; font:12px "JetBrains Mono",monospace; }}
    .strip-item {{ display:flex; gap:7px; align-items:center; white-space:nowrap; }}
    .strip-item label {{ color:#687784; letter-spacing:.08em; }}
    .indicator {{ width:7px; height:7px; border:1px solid #65717b; border-radius:50%; }}
    .indicator.online {{ border-color:#40c878; background:#40c878; box-shadow:0 0 8px #40c87888; }}
    .indicator.active {{ border-color:#4da3ff; background:#4da3ff; box-shadow:0 0 8px #4da3ff88; }}
    .clock {{ color:#aab3bb; min-width:48px; text-align:right; }}
    .console-layout {{ height:calc(100vh - 140px); display:grid;
      grid-template-columns:minmax(620px,1fr) 400px; }}
    .viewer-shell {{ position:relative; min-width:0; min-height:0; margin:0; padding:0;
      border:0; border-right:1px solid #262d34; border-radius:0; background:#12171c; }}
    .viewer-shell .rviz-frame {{ position:absolute; inset:0; border:0; border-radius:0; }}
    .viewer-shell canvas {{ background:#12171c !important; }}
    .rviz-note {{ left:16px; right:auto; bottom:12px; color:#71808d;
      font:10px "JetBrains Mono",monospace; }}
    .hud {{ position:absolute; z-index:3; pointer-events:none; color:#aeb9c3;
      font:13px/1.7 "JetBrains Mono","Roboto Mono",monospace; letter-spacing:.02em; }}
    .hud strong {{ display:block; color:#eef2f5; font-size:14px; letter-spacing:.1em; }}
    .hud-top {{ top:16px; left:18px; }}
    .hud-bottom-left {{ left:18px; bottom:42px; }}
    .hud-bottom-right {{ right:18px; bottom:20px; text-align:right; }}
    .hud-row {{ display:grid; grid-template-columns:58px 78px; }}
    .hud-row span:first-child {{ color:#64727e; }}
    .operator {{ min-height:0; overflow:auto; background:#10151a; }}
    .operator-title {{ height:45px; display:flex; align-items:center; justify-content:space-between;
      padding:0 16px; border-bottom:1px solid #262d34; font-size:14px;
      font-weight:800; letter-spacing:.14em; }}
    .operator section {{ margin:0; padding:16px; border:0; border-radius:0;
      border-bottom:1px solid #262d34; background:transparent; }}
    .operator h2 {{ margin:0 0 12px; color:#dfe5ea; font-size:15px; letter-spacing:.09em; }}
    .op-line {{ display:flex; align-items:center; justify-content:space-between; gap:8px;
      min-height:28px; color:#7f8b96; font:12px "JetBrains Mono",monospace; }}
    .op-value {{ color:#cbd3da; }}
    .compact-actions {{ display:flex; gap:9px; margin-top:10px; }}
    .operator form {{ display:flex; gap:6px; margin:0; align-items:center; }}
    .operator button {{ height:42px; min-width:96px; padding:0 14px; border:1px solid #35404a;
      border-radius:3px; color:#cbd3da; background:#192027; font-size:13px;
      letter-spacing:.04em; font-weight:750; }}
    .operator button:hover {{ border-color:#4da3ff; color:#fff; }}
    .operator button.stop {{ border-color:#633438; background:#251719; color:#ff8b8d; }}
    .operator button.active {{ border-color:#4da3ff; background:#19334e; color:#77baff; outline:0; }}
    .operator input {{ height:40px; width:96px; padding:0 9px; border:1px solid #35404a;
      border-radius:2px; color:#e6e9ed; background:#0b0e11; font:13px "JetBrains Mono",monospace; }}
    .motion-pad {{ width:286px; margin:12px auto 3px; display:grid;
      grid-template-columns:repeat(3,90px); gap:8px; }}
    .motion-pad button {{ height:46px; min-width:90px; padding:0; font:700 13px "JetBrains Mono",monospace; }}
    .motion-pad .blank {{ visibility:hidden; }}
    .motion-meta {{ display:flex; justify-content:space-between; color:#65727e;
      font:11px "JetBrains Mono",monospace; margin-top:9px; }}
    .transform-track {{ display:flex; align-items:center; gap:7px; margin:7px 0;
      color:#7f8b96; font:10px "JetBrains Mono",monospace; }}
    .track {{ flex:1; height:1px; background:#35404a; position:relative; }}
    .track::before {{ content:''; position:absolute; left:0; top:-3px; width:7px; height:7px;
      border-radius:50%; background:#4da3ff; }}
    .operator-link {{ display:block; padding:13px 14px; color:#cbd3da; text-decoration:none;
      border:1px solid #35404a; border-radius:3px; text-align:center; font-size:13px; font-weight:750; }}
    .operator-link:hover {{ border-color:#4da3ff; color:#fff; }}
    .operator details {{ border:0; background:transparent; }}
    .operator summary {{ padding:0; color:#7f8b96; font-size:10px; cursor:pointer; }}
    .operator pre {{ margin-top:8px; max-height:160px; min-height:0; border-radius:0;
      background:#090c0f; font:9px/1.5 "JetBrains Mono",monospace; }}
    .console-footer {{ height:76px; display:grid; grid-template-columns:82px 1fr; align-content:center;
      gap:3px 10px; padding:7px 16px;
      border-top:1px solid #262d34; background:#0d1115; color:#77838d;
      font:11px/1.35 "JetBrains Mono",monospace; }}
    .console-footer time {{ color:#4da3ff; }}
    .event-line {{ display:contents; }}
    .event-message {{ overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }}
    body.viewer-only .console-top, body.viewer-only .operator, body.viewer-only .console-footer {{ display:none; }}
    body.viewer-only .console-layout {{ height:100vh; grid-template-columns:1fr; }}
    body.viewer-only .viewer-shell {{ border:0; }}
    @media(max-width:850px) {{ body.console {{ overflow:auto; }} .console-top {{ padding:0 10px; }}
      .status-strip .optional {{ display:none; }} .console-layout {{ height:auto; grid-template-columns:1fr; }}
      .viewer-shell {{ height:62vh; border-right:0; border-bottom:1px solid #262d34; }}
      .operator {{ overflow:visible; }} }}
  </style>
</head>
<body class="console {'viewer-only' if viewer_only else ''}">
  <header class="console-top">
    <div style="display:flex;height:100%;align-items:center">
      <div class="console-brand"><strong>ACTUATE / BIPED OPS</strong><span>BIPED_01</span></div>
      <nav class="console-tabs"><a class="active" href="/">OPERATIONS</a><a href="/manipulator">MANIPULATION</a></nav>
    </div>
    <div class="status-strip">
      <span class="strip-item optional"><label>MODE</label><b id="operationMode">{operation_mode}</b></span>
      <span class="strip-item"><i id="feedbackDot" class="indicator {'online' if feedback_online else ''}"></i><label>JOINT</label><b id="feedbackText">{feedback_text}</b></span>
      <span class="strip-item"><i id="hardwareDot" class="indicator {'online' if opencr_owner != '없음' else ''}"></i><label>OpenCR</label><b id="opencrOwner">{opencr_owner}</b></span>
      <span class="strip-item"><i id="policyDot" class="indicator {'active' if rl_running else ''}"></i><label>POLICY</label><b id="rlState">{'ON' if rl_running else 'OFF'}</b></span>
      <time class="clock" id="consoleClock">--:--</time>
    </div>
  </header>
  <div class="console-layout">
    <section class="viewer-shell">
      <div class="rviz-frame">
        <canvas id="robotViewer"></canvas>
        <div class="rviz-note" id="viewerStatus">VIEWER INITIALIZING</div>
      </div>
      <div class="hud hud-top"><strong>BIPED_01</strong><div>MODE&nbsp;&nbsp;&nbsp;&nbsp;<span id="hudMode">{operation_mode}</span></div><div>FPS&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;<span id="viewerFps">--</span></div><div>JOINT&nbsp;&nbsp;&nbsp;<span id="hudJoint">{'ONLINE' if feedback_online else 'WAITING'}</span></div><div>OpenCR&nbsp;&nbsp;<span id="hudOpencr">{opencr_owner}</span></div><div>COMMAND&nbsp;<span id="hudCommand">STOP</span></div></div>
      <div class="hud hud-bottom-left"><strong>BASE ORIENTATION</strong><div class="hud-row"><span>PITCH</span><b id="hudPitch">+00.00°</b></div><div class="hud-row"><span>ROLL</span><b id="hudRoll">+00.00°</b></div><div class="hud-row"><span>YAW</span><b id="hudYaw">+00.00°</b></div></div>
      <div class="hud hud-bottom-right"><strong>BASE POSITION</strong><div>X&nbsp;&nbsp;<b id="hudX">+0.000 m</b></div><div>Y&nbsp;&nbsp;<b id="hudY">+0.000 m</b></div></div>
    </section>
    <aside class="operator">
      <div class="operator-title"><span>OPERATION</span><span id="systemModePill">{operation_mode}</span></div>
      <section>
        <h2>RL WALK</h2>
        <div class="op-line"><span>STATUS</span><b class="op-value">{rl_status}</b></div>
        <div class="op-line"><span>LINEAR X</span><b class="op-value">+0.08 m/s</b></div>
        <div class="op-line"><span>ANGULAR Z</span><b class="op-value">+0.25 rad/s</b></div>
        <div class="compact-actions"><form method="post" action="/rl/start"><button type="submit">START POLICY</button></form><form method="post" action="/rl/stop"><button class="stop" type="submit">STOP / READY</button></form></div>
        <div class="motion-pad"><button type="button" data-rl-command="ccw">Q · YAW−</button><button type="button" data-rl-command="forward">W · FWD</button><button type="button" data-rl-command="cw">E · YAW+</button><button type="button" data-rl-command="left">A · LEFT</button><button class="stop" type="button" data-rl-command="stop">■ STOP</button><button type="button" data-rl-command="right">D · RIGHT</button></div>
        <div class="motion-meta"><span>HOLD TO MOVE</span><span>DEADMAN 350 ms</span></div>
      </section>
      <section>
        <h2>TRANSFORM</h2>
        <div class="transform-track"><b>WALK</b><span class="track"></span><b>BIKE</b></div>
        <form method="post">
          <span class="op-line" style="flex:1"><span>STAGE</span><input name="stage_duration_sec" type="number" min="0.5" max="30" step="0.5" value="5.0"></span>
          <button type="submit" formaction="/revert">TO WALK</button><button type="submit" formaction="/transform">TO BIKE</button>
        </form>
      </section>
      <section>
        <h2>BIKE DRIVE</h2>
        <div class="op-line"><span>STATUS</span><b class="op-value">{teleop_label}</b></div>
        <form method="post" action="/teleop/speed"><span class="op-line" style="flex:1"><span>SPEED</span><input name="speed" type="number" min="{teleop_min_speed:.1f}" max="{teleop_max_speed:.1f}" step="0.1" value="{teleop_speed:.1f}"></span><button type="submit">SET</button></form>
        <div class="compact-actions"><form method="post" action="/teleop/start"><button type="submit">START DRIVE</button></form><form method="post" action="/teleop/stop"><button class="stop" type="submit">STOP</button></form></div>
        <span id="pressedKeys" hidden>{html.escape(pressed_keys)}</span>
      </section>
      <section><a class="operator-link" href="/manipulator">매니퓰레이터 작업 화면 열기 →</a></section>
      <section><details><summary>실험/유틸리티 · IK 및 브리지</summary><div class="compact-actions"><form method="post" action="/hardware/start"><button type="submit">BRIDGE ON</button></form><form method="post" action="/hardware/stop"><button type="submit">BRIDGE OFF</button></form></div><form method="post" action="/walk" style="margin-top:9px"><span class="op-line" style="flex:1"><span>IK CYCLES</span><input name="num_cycles" type="number" min="1" max="50" value="1"></span><button type="submit">RUN IK</button></form></details></section>
      <section><details><summary>SYSTEM DIAGNOSTICS</summary><pre>{html.escape(rl_log)}\n{html.escape(hardware_log)}</pre></details></section>
    </aside>
  </div>
  <footer class="console-footer" id="eventLog">
    <span class="event-line"><time>--:--:--</time><span class="event-message">VIEWER INITIALIZING</span></span>
    <span class="event-line"><time>--:--:--</time><span class="event-message">JOINT FEEDBACK {feedback_text}</span></span>
    <span class="event-line"><time>--:--:--</time><span class="event-message">POLICY {'ACTIVE' if rl_running else 'DISABLED'}</span></span>
    <span class="event-line"><time id="eventTime">--:--:--</time><span class="event-message" id="systemMessage">{html.escape(message)}</span></span>
  </footer>
  <script>
    const systemEvents = [];
    let previousSystemMessage = '';
    let previousFeedback = null;
    function addSystemEvent(message) {{
      if (!message) return;
      systemEvents.push({{time:new Date().toLocaleTimeString('ko-KR',{{hour12:false}}),message}});
      while (systemEvents.length > 4) systemEvents.shift();
      const log = document.getElementById('eventLog');
      log.replaceChildren(...systemEvents.flatMap(event => {{
        const time=document.createElement('time'); time.textContent=event.time;
        const text=document.createElement('span'); text.className='event-message'; text.textContent=event.message;
        return [time,text];
      }}));
    }}
    async function refreshSystemStatus() {{
      try {{
        const s = await (await fetch('/system_status.json', {{cache:'no-store'}})).json();
        document.getElementById('operationMode').textContent = s.mode;
        document.getElementById('opencrOwner').textContent = s.opencr_owner;
        const jointRate = Number(s.joint_rate_hz || 0);
        document.getElementById('feedbackText').textContent = s.joint_feedback ? jointRate.toFixed(1) + ' Hz' : '대기';
        document.getElementById('feedbackDot').className = 'indicator ' + (s.joint_feedback ? 'online' : '');
        document.getElementById('hardwareDot').className = 'indicator ' + (s.opencr_owner === '없음' ? '' : 'online');
        document.getElementById('policyDot').className = 'indicator ' + (s.rl_walking ? 'active' : '');
        document.getElementById('rlState').textContent = s.rl_walking ? 'ON' : 'OFF';
        const pill = document.getElementById('systemModePill');
        pill.textContent = s.mode;
        document.getElementById('hudMode').textContent = s.mode;
        document.getElementById('hudJoint').textContent = s.joint_feedback ? jointRate.toFixed(1) + ' Hz' : 'WAITING';
        document.getElementById('hudOpencr').textContent = s.opencr_owner;
        if (s.message !== previousSystemMessage) {{ addSystemEvent(s.message); previousSystemMessage=s.message; }}
        if (s.joint_feedback !== previousFeedback) {{ addSystemEvent('JOINT FEEDBACK ' + (s.joint_feedback ? 'CONNECTED' : 'WAITING')); previousFeedback=s.joint_feedback; }}
      }} catch (_) {{}}
    }}
    function updateClock() {{ const now=new Date(); document.getElementById('consoleClock').textContent=now.toLocaleTimeString('ko-KR',{{hour:'2-digit',minute:'2-digit',hour12:false}}); }}
    addSystemEvent('VIEWER CONNECTED'); addSystemEvent('POLICY {'ACTIVE' if rl_running else 'DISABLED'}'); refreshSystemStatus(); setInterval(refreshSystemStatus, 500); updateClock(); setInterval(updateClock,1000);
  </script>
</body>
</html>"""
        encoded = page.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def _send_manipulator_html(self):
        with STATE.lock:
            process = STATE.manipulator
            message = STATE.message
        running = process is not None and process.is_running()
        status = "연결됨" if running else "정지"
        status_class = "on" if running else "off"
        output = "\n".join(process.output[-80:]) if process else ""
        page = f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>BIPED OPS · 매니퓰레이터</title>
  <style>
    :root {{ color-scheme:dark; --bg:#0b0e11; --panel:#10151a; --line:#262d34;
      --text:#e6e9ed; --muted:#7f8b96; --blue:#4da3ff; --green:#40c878;
      --amber:#ffb020; --red:#ff4d4f; }}
    * {{ box-sizing:border-box; }}
    [hidden] {{ display:none !important; }}
    body {{ margin:0; background:var(--bg); color:var(--text);
      font-family:Inter,Pretendard,"Noto Sans KR",system-ui,sans-serif; }}
    header {{ height:68px; display:flex; align-items:center; justify-content:space-between;
      padding:0 20px; border-bottom:1px solid var(--line); background:#0d1115; }}
    .brand {{ display:flex; align-items:center; height:100%; gap:24px; }}
    h1 {{ font-size:20px; margin:0; letter-spacing:.05em; }}
    h2 {{ font-size:15px; margin:0; }}
    .tabs {{ height:100%; display:flex; align-items:stretch; }}
    .tabs a {{ min-width:142px; display:grid; place-items:center; border-bottom:3px solid transparent;
      color:var(--muted); text-decoration:none; font-size:13px; font-weight:750; }}
    .tabs a.active {{ border-color:var(--blue); color:#fff; background:#111922; }}
    .head-right {{ display:flex; align-items:center; gap:14px; }}
    .status {{ display:inline-flex; align-items:center; gap:7px; padding:6px 11px;
      border-radius:3px; font-size:13px; font-weight:800; }}
    .status::before {{ content:''; width:7px; height:7px; border-radius:50%; }}
    .status.on {{ background:#123b31; color:#73e2ba; }}
    .status.on::before {{ background:var(--green); box-shadow:0 0 9px var(--green); }}
    .status.off {{ background:#3d2428; color:#ff9b9b; }}
    .status.off::before {{ background:var(--red); }}
    .system-strip {{ height:64px; display:grid; grid-template-columns:repeat(5,1fr);
      border-bottom:1px solid var(--line); background:#0e1318; }}
    .system-item {{ padding:11px 16px; border-right:1px solid var(--line); }}
    .system-item span {{ display:block; color:var(--muted); font:11px "JetBrains Mono",monospace; }}
    .system-item strong {{ display:block; margin-top:5px; font:14px "JetBrains Mono",monospace; }}
    .led {{ display:inline-block; width:8px; height:8px; margin-right:7px; border-radius:50%;
      border:1px solid #66717b; }} .led.on {{ background:var(--green); border-color:var(--green); box-shadow:0 0 8px #40c87888; }}
    .led.warn {{ background:var(--amber); border-color:var(--amber); }} .led.bad {{ background:var(--red); border-color:var(--red); }}
    main {{ height:calc(100vh - 132px); padding:14px; display:grid;
      grid-template-columns:minmax(420px,46%) minmax(600px,54%); gap:14px; }}
    .left {{ min-height:0; display:grid; grid-template-rows:minmax(360px,1fr) auto; gap:12px; }}
    .panel {{ min-height:0; background:var(--panel); border:1px solid var(--line);
      border-radius:3px; overflow:hidden; position:relative; }}
    .panel-head {{ height:48px; display:flex; align-items:center; justify-content:space-between;
      padding:0 16px; border-bottom:1px solid var(--line); color:var(--muted); font-size:12px; }}
    iframe {{ width:100%; height:calc(100% - 48px); border:0; background:#10151a; }}
    .manip-frame {{ height:100%; position:relative; }}
    .actions {{ padding:16px; display:grid; grid-template-columns:1fr 1fr; gap:10px; }}
    form {{ margin:0; }}
    button {{ height:46px; min-width:120px; border:1px solid #35404a; border-radius:3px;
      padding:0 18px; color:#e6e9ed; background:#192027; font-size:14px; font-weight:800; cursor:pointer; }}
    button:hover {{ border-color:var(--blue); }}
    button.primary {{ border-color:#317ac1; background:#173452; color:#fff; }}
    button.stop {{ border-color:#76383b; background:#2c1719; color:#ff8b8d; }}
    .message {{ flex:1 1 100%; margin:4px 0 0; color:var(--muted); font-size:12px; }}
    .actions .message,.actions details {{ grid-column:1/-1; }}
    details {{ color:var(--muted); font-size:12px; }}
    pre {{ max-height:180px; overflow:auto; white-space:pre-wrap; background:#070d13;
      padding:10px; border-radius:0; }}
    .offline {{ position:absolute; inset:48px 0 0; z-index:2; display:flex; flex-direction:column;
      align-items:center; justify-content:center; text-align:center; padding:28px; background:#0e141a; }}
    .offline-icon {{ width:52px; height:40px; margin-bottom:18px; border:2px solid #52606c;
      display:grid; place-items:center; color:#87939e; font-size:20px; }}
    .offline h2 {{ margin:0 0 8px; font-size:19px; }} .offline p {{ max-width:420px; margin:0 0 18px;
      color:var(--muted); line-height:1.6; font-size:13px; }}
    .offline-actions {{ display:flex; gap:10px; }}
    .connection-list {{ width:min(410px,100%); margin:20px 0; display:grid; grid-template-columns:repeat(3,1fr); border:1px solid var(--line); }}
    .connection-list div {{ padding:10px; border-right:1px solid var(--line); }}
    .connection-list div:last-child {{ border:0; }} .connection-list span {{ display:block; color:var(--muted); font-size:11px; }}
    .connection-list b {{ display:block; margin-top:5px; font-size:12px; }}
    @media(max-width:1050px) {{ main {{ height:auto; grid-template-columns:1fr; }}
      .left {{ grid-template-rows:520px auto; }} .manip-frame {{ height:850px; }}
      .system-strip {{ grid-template-columns:repeat(2,1fr); height:auto; }} }}
  </style>
</head>
<body>
  <header>
    <div class="brand"><h1>ACTUATE / BIPED OPS</h1><nav class="tabs"><a href="/">OPERATIONS</a><a class="active" href="/manipulator">MANIPULATION</a></nav></div>
    <div class="head-right"><span>매니퓰레이터 운영</span><span id="stackStatus" class="status {status_class}">{status}</span></div>
  </header>
  <div class="system-strip">
    <div class="system-item"><span>MODE</span><strong id="manipMode">STANDBY</strong></div>
    <div class="system-item"><span>LOWER BODY</span><strong><i id="lowerLed" class="led warn"></i><span id="lowerState">NOT LOCKED</span></strong></div>
    <div class="system-item"><span>POSE ERROR</span><strong id="poseError">-- rad</strong></div>
    <div class="system-item"><span>LEADER / OpenRB</span><strong><i id="leaderLed" class="led"></i><span id="leaderState">WAITING</span></strong></div>
    <div class="system-item"><span>FOLLOWER / OpenCR</span><strong><i id="followerLed" class="led"></i><span id="followerState">WAITING</span></strong></div>
  </div>
  <main>
    <div class="left">
      <section class="panel">
        <div class="panel-head"><h2>실시간 로봇 자세</h2><span>관절 피드백 · 12.5 Hz</span></div>
        <iframe src="/robot-view" title="실시간 로봇 3D 모델"></iframe>
      </section>
      <section class="panel actions">
        <form method="post" action="/manipulator/start"><button class="primary" type="submit">시스템 시작</button></form>
        <form method="post" action="/manipulator/stop"><button class="stop" type="submit">시스템 정지</button></form>
        <p class="message">{html.escape(message)}</p>
        <details><summary>시스템 로그</summary><pre>{html.escape(output)}</pre></details>
      </section>
    </div>
    <section class="panel manip-frame">
      <div class="panel-head"><h2>매니퓰레이터 작업 공간</h2><span id="workspaceStatus">CONTROL SERVER WAITING</span></div>
      <div class="offline" id="workspaceOffline">
        <div class="offline-icon">CAM</div>
        <h2>작업 시스템 오프라인</h2>
        <p>카메라 및 매니퓰레이터 제어 서버를 기다리는 중입니다. 장치를 연결한 뒤 시스템을 시작하세요. 브라우저 오류 화면은 표시되지 않습니다.</p>
        <div class="connection-list"><div><span>CAMERA</span><b id="offlineCamera">WAITING</b></div><div><span>OpenRB</span><b id="offlineLeader">WAITING</b></div><div><span>OpenCR</span><b id="offlineFollower">WAITING</b></div></div>
        <div class="offline-actions"><form method="post" action="/manipulator/start"><button class="primary" type="submit">시스템 시작</button></form><button type="button" onclick="refreshStackStatus()">상태 다시 확인</button></div>
      </div>
      <iframe id="manipulatorUi" title="매니퓰레이터 제어 UI" hidden></iframe>
    </section>
  </main>
  <script>
    let workspaceLoaded = false;
    async function refreshStackStatus() {{
      try {{
        const s = await (await fetch('/system_status.json', {{cache:'no-store'}})).json();
        const pill = document.getElementById('stackStatus');
        pill.textContent = s.manipulator_ui_ready ? '연결됨' : (s.manipulator ? '시작 중' : '정지');
        pill.className = 'status ' + (s.manipulator_ui_ready ? 'on' : 'off');
        const devices = s.devices || {{}};
        const detail = s.manipulator_detail || {{}};
        const mode = detail.mode?.mode || (s.manipulator ? 'STARTING' : 'STANDBY');
        const locked = ['STABLE','TELEOP','RECORD','INFERENCE'].includes(mode);
        const errors = Object.values(detail.mode?.lower_error_rad || {{}}).map(Number).filter(Number.isFinite);
        document.getElementById('manipMode').textContent = mode;
        document.getElementById('lowerState').textContent = locked ? 'KNEELING LOCKED' : (mode === 'KNEELING' ? 'MOVING' : 'NOT LOCKED');
        document.getElementById('lowerLed').className = 'led ' + (locked ? 'on' : mode === 'FAULT' ? 'bad' : 'warn');
        document.getElementById('poseError').textContent = errors.length ? Math.max(...errors).toFixed(3) + ' rad' : '-- rad';
        const connections = detail.connections || {{}};
        const setDevice = (name, online, detected) => {{
          document.getElementById(name+'State').textContent=online?'CONNECTED':detected?'DETECTED':'WAITING';
          document.getElementById(name+'Led').className='led '+(online?'on':'warn');
        }};
        setDevice('leader', !!connections.leader, !!devices.openrb);
        setDevice('follower', !!connections.follower, !!devices.opencr);
        const stateLabel = (online, detected) => online ? 'CONNECTED' : detected ? 'DETECTED' : 'WAITING';
        document.getElementById('offlineCamera').textContent = stateLabel(!!connections.camera, !!devices.camera);
        document.getElementById('offlineLeader').textContent = stateLabel(!!connections.leader, !!devices.openrb);
        document.getElementById('offlineFollower').textContent = stateLabel(!!connections.follower, !!devices.opencr);
        const frame = document.getElementById('manipulatorUi');
        const offline = document.getElementById('workspaceOffline');
        if (s.manipulator_ui_ready) {{
          offline.hidden = true; frame.hidden = false;
          document.getElementById('workspaceStatus').textContent = 'CAMERA · CONTROL ONLINE';
          if (!workspaceLoaded) {{ frame.src=`http://${{window.location.hostname}}:8000`; workspaceLoaded=true; }}
        }} else {{
          offline.hidden = false; frame.hidden = true;
          document.getElementById('workspaceStatus').textContent = s.manipulator ? 'STARTING SERVICES' : 'CONTROL SERVER OFFLINE';
          if (workspaceLoaded) {{ frame.src='about:blank'; workspaceLoaded=false; }}
        }}
      }} catch (_) {{}}
    }}
    refreshStackStatus(); setInterval(refreshStackStatus, 500);
  </script>
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
    server = None
    try:
        server = ThreadingHTTPServer((HOST, PORT), ControlHandler)
        print(f"BIPED OPS 통합 관제: http://{HOST}:{PORT}")
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    except OSError as exc:
        if exc.errno == 98:
            print(f"포트 {PORT}이 이미 사용 중입니다. 기존 웹 서버를 먼저 종료하세요.")
        else:
            raise
    finally:
        STATE.stop_all()
        STATE.monitor.stop()
        if server is not None:
            server.server_close()


if __name__ == "__main__":
    main()
