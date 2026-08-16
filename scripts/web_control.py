#!/usr/bin/env python3
"""Small local web panel for biped_bike hardware, walking, and transform commands."""

import html
import os
import signal
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs

try:
    import rclpy
    from std_msgs.msg import Float64MultiArray
except ImportError:
    rclpy = None
    Float64MultiArray = None


HOST = "127.0.0.1"
PORT = 8080


HARDWARE_COMMAND = [
    "ros2",
    "launch",
    "biped_bike_robot",
    "hardware_display.launch.py",
    "max_abs_position_rad:=2.2",
    "center_on_start:=false",
    "startup_ready_posture_on_start:=true",
    "startup_forward_lean_deg:=10.0",
    "startup_shoulder_pitch_deg:=-70.0",
    "enable_joint_state_commands:=false",
    "enable_trajectory_commands:=true",
]


def walk_command(num_cycles: int) -> list[str]:
    return [
        "ros2",
        "run",
        "biped_bike_robot",
        "op3_walker.py",
        "--ros-args",
        "-p",
        f"num_cycles:={num_cycles}",
        "-p",
        "support_hip_roll_lift_deg:=20.0",
        "-p",
        "support_ankle_roll_lift_deg:=10.0",
        "-p",
        "support_ankle_roll_lift_sign:=1.0",
        "-p",
        "pelvis_pitch_forward_lift_deg:=30.0",
        "-p",
        "pelvis_pitch_forward_lift_sign:=1.0",
        "-p",
        "trajectory_time_scale:=4.0",
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
      width: min(980px, calc(100vw - 32px));
      margin: 28px auto;
    }}
    h1 {{ font-size: 24px; margin: 0 0 18px; }}
    h2 {{ font-size: 16px; margin: 0 0 12px; }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
    }}
    .wide-grid {{
      display: grid;
      grid-template-columns: 1fr 2fr;
      gap: 12px;
      margin-top: 12px;
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
      margin-top: 12px;
      display: grid;
      gap: 12px;
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
      .grid {{ grid-template-columns: 1fr; }}
      input {{ width: 100%; }}
      form {{ align-items: stretch; }}
      button {{ width: 100%; }}
      .wide-grid {{ grid-template-columns: 1fr; }}
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
</head>
<body>
  <main>
    <h1>Biped Bike Control</h1>
    <p class="message">{html.escape(message)}</p>
    <div class="grid">
      <section>
        <h2>1. Hardware</h2>
        <div class="status {hardware_class}">{hardware_status}</div>
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
  </main>
</body>
</html>"""
        encoded = page.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def log_message(self, fmt, *args):
        return


def main():
    server = ThreadingHTTPServer((HOST, PORT), ControlHandler)
    print(f"Biped Bike web control: http://{HOST}:{PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        STATE.stop_hardware()
        STATE.teleop.stop()
        server.server_close()


if __name__ == "__main__":
    main()
