#!/usr/bin/env python3
from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import shutil
import yaml

from common.config import load_config, resolve_under_root
from common.hardware_config import merge_gripper_motor
from common.postures import load_reference_postures


def check(label: str, ok: bool, detail: str) -> bool:
    print(f"[{'OK' if ok else 'FAIL'}] {label}: {detail}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser(description="Check the native manipulator setup")
    parser.add_argument("--hardware", action="store_true")
    parser.add_argument("--leader", action="store_true")
    parser.add_argument("--camera", action="store_true")
    parser.add_argument("--training", action="store_true")
    args = parser.parse_args()
    cfg = load_config()
    root = Path(cfg["_root"])
    results = [
        check("module root", root.name == "manipulator", str(root)),
        check("ros2", shutil.which("ros2") is not None, str(shutil.which("ros2"))),
        check("rclpy", importlib.util.find_spec("rclpy") is not None, "ROS Python environment"),
    ]
    try:
        reference = load_reference_postures(cfg)
        posture_ok = True
        posture_detail = (
            f"{cfg['postures']['kneeling_symbol']} from {cfg['postures']['source']} "
            f"({len(reference.joint_names)} joints)"
        )
    except Exception as exc:
        posture_ok = False
        posture_detail = str(exc)
    results.append(check("reference postures", posture_ok, posture_detail))
    requested = {
        "lower-body OpenCR": args.hardware and cfg["devices"]["lower_body"],
        "leader OpenRB": args.leader and cfg["devices"]["leader"],
        "camera": args.camera and cfg["camera"]["device"],
    }
    for label, value in requested.items():
        if value:
            path = Path(str(value))
            results.append(check(label, path.exists(), str(path)))
    if args.hardware:
        for label, key in (
            ("current lower bridge", "lower_bridge_script"),
            ("current lower config", "lower_config"),
        ):
            path = Path(cfg["hardware"][key])
            results.append(check(label, path.is_file(), str(path)))
        try:
            with Path(cfg["hardware"]["lower_config"]).open(
                "r", encoding="utf-8"
            ) as stream:
                merged = merge_gripper_motor(yaml.safe_load(stream), cfg["gripper_motor"])
            gripper_ok = any(
                int(joint.get("id", -1)) == 20
                and joint.get("joint_name") == "arm_gripper_jnt"
                for joint in merged["joints"]
            )
            gripper_detail = "ID 20 -> arm_gripper_jnt"
        except Exception as exc:
            gripper_ok = False
            gripper_detail = str(exc)
        results.append(check("follower gripper mapping", gripper_ok, gripper_detail))
    if args.leader:
        sdk_ok = importlib.util.find_spec("dynamixel_sdk") is not None
        results.append(check("Dynamixel SDK", sdk_ok, "native passive OpenRB reader"))
    if args.training:
        python = resolve_under_root(cfg, cfg["training"]["python"])
        results.append(check("training venv", python.exists(), str(python)))
    return 0 if all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
