#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from common.config import DEFAULT_CONFIG, load_config
from common.hardware_config import materialize_hardware_config


ROOT = Path(__file__).resolve().parent


def node(
    name: str, config: Path, *extra: str, python: str = sys.executable
) -> tuple[str, list[str]]:
    return (
        name,
        [python, str(ROOT / "nodes" / f"{name}.py"), "--config", str(config), *extra],
    )


def parse_args():
    parser = argparse.ArgumentParser(description="Native biped manipulator supervisor")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--hardware", action="store_true", help="start the existing OpenCR hardware bridge")
    parser.add_argument("--leader", action="store_true", help="start the existing OMX-L OpenRB driver")
    parser.add_argument("--camera", action="store_true", help="start usb_cam")
    parser.add_argument(
        "--ready-on-start", action="store_true",
        help="command the existing biped ready posture when hardware starts",
    )
    parser.add_argument("--model", help="initial inference checkpoint")
    return parser.parse_args()


def main():
    args = parse_args()
    config_path = Path(args.config).expanduser().resolve()
    cfg = load_config(config_path)
    commands: list[tuple[str, list[str]]] = []

    if args.hardware:
        device = Path(cfg["devices"]["lower_body"])
        if not device.exists():
            raise FileNotFoundError(f"OpenCR device not found: {device}")
        bridge_script = Path(cfg["hardware"]["lower_bridge_script"])
        bridge_config_base = Path(cfg["hardware"]["lower_config"])
        if not bridge_script.is_file() or not bridge_config_base.is_file():
            raise FileNotFoundError(
                f"current biped bridge/config not found: {bridge_script}, {bridge_config_base}"
            )
        bridge_config = materialize_hardware_config(
            bridge_config_base,
            cfg["gripper_motor"],
            ROOT / "data" / "runtime" / "dynamixel_hardware_with_gripper.yaml",
        )
        commands.append((
            "biped_hardware",
            [
                sys.executable,
                str(ROOT / "nodes" / "dxl_joint_state_bridge_streaming.py"),
                "--base-bridge", str(bridge_script),
                "--ros-args",
                "-p", f"config_path:={bridge_config}",
                "-p", "torque_on_start:=true",
                "-p", "torque_off_on_shutdown:=true",
                "-p", "publish_present_joint_states:=true",
                "-p", "present_joint_state_rate_hz:=20.0",
                "-p", "min_tick_change:=1",
                "-p", "enable_joint_state_commands:=false",
                "-p", "enable_trajectory_commands:=true",
                "-p", "enable_velocity_commands:=true",
                "-p", "startup_ready_posture_on_start:=false",
                "-p", "center_on_start:=false",
                "-p", "enable_opencr_imu:=false",
                "-p", "max_abs_position_rad:=2.2",
            ],
        ))
        if args.ready_on_start:
            commands.append((
                "smooth_ready_once",
                [
                    sys.executable,
                    str(ROOT / "nodes" / "smooth_ready.py"),
                    "--config", str(config_path),
                    "--duration", "3.0",
                    "--rate", "50.0",
                ],
            ))
    if args.leader:
        device = Path(cfg["devices"]["leader"])
        if not device.exists():
            raise FileNotFoundError(f"OpenRB leader device not found: {device}")
        commands.append((
            "openrb_leader",
            node("leader_openrb_reader", config_path)[1],
        ))
    if args.camera:
        camera = cfg["camera"]
        device = Path(camera["device"])
        if not device.exists():
            raise FileNotFoundError(f"camera device not found: {device}")
        commands.append((
            "usb_camera",
            [
                "ros2", "run", "usb_cam", "usb_cam_node_exe", "--ros-args",
                "-r", "__ns:=/camera1",
                "-p", f"video_device:={device}",
                "-p", f"pixel_format:={camera['pixel_format']}",
                "-p", f"image_width:={camera['width']}",
                "-p", f"image_height:={camera['height']}",
                "-p", f"framerate:={camera['fps']}",
                "-p", "camera_name:=camera1",
                "-p", "frame_id:=camera1_link",
            ],
        ))

    training_python = Path(cfg["_root"]) / cfg["training"]["python"]
    inference_python = str(training_python) if training_python.exists() else sys.executable
    commands.extend([
        node("mode_manager", config_path),
        node("leader_mapper", config_path),
        node("arm_command_arbiter", config_path),
        node("episode_recorder", config_path),
        node(
            "policy_inference", config_path,
            *(["--model", args.model] if args.model else []),
            python=inference_python,
        ),
        node("web_server", config_path),
    ])

    processes: list[tuple[str, subprocess.Popen]] = []
    stopping = False

    def stop_all(*_):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        for name, process in reversed(processes):
            if process.poll() is None:
                print(f"[supervisor] stopping {name}", flush=True)
                os.killpg(process.pid, signal.SIGTERM)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and any(p.poll() is None for _, p in processes):
            time.sleep(0.1)
        for _, process in processes:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)

    signal.signal(signal.SIGINT, stop_all)
    signal.signal(signal.SIGTERM, stop_all)
    try:
        for name, command in commands:
            print(f"[supervisor] starting {name}: {' '.join(command)}", flush=True)
            if name == "smooth_ready_once":
                # The helper waits for measured /joint_states, publishes one
                # dense smoothstep trajectory, and returns after playback time.
                result = subprocess.run(command, cwd=ROOT, check=False)
                if result.returncode != 0:
                    raise RuntimeError(
                        f"smooth_ready.py exited with status {result.returncode}"
                    )
                continue
            process = subprocess.Popen(command, cwd=ROOT, start_new_session=True)
            processes.append((name, process))
            time.sleep(0.15)
        print(f"[supervisor] web UI: http://localhost:{cfg['web']['port']}", flush=True)
        while not stopping:
            for name, process in processes:
                code = process.poll()
                if code is not None:
                    raise RuntimeError(f"{name} exited with status {code}")
            time.sleep(0.5)
    finally:
        stop_all()


if __name__ == "__main__":
    main()
