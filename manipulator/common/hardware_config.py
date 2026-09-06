from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any, Mapping

import yaml


def merge_gripper_motor(base: Mapping[str, Any], motor: Mapping[str, Any]) -> dict:
    """Return a copied Dynamixel config with one unambiguous gripper motor."""
    result = deepcopy(dict(base))
    joints = result.get("joints")
    if not isinstance(joints, list):
        raise ValueError("base Dynamixel config has no joints list")
    gripper = deepcopy(dict(motor))
    gripper_id = int(gripper["id"])
    gripper_name = str(gripper["joint_name"])
    conflicts = [
        joint for joint in joints
        if int(joint.get("id", -1)) == gripper_id
        or str(joint.get("joint_name", "")) == gripper_name
    ]
    if conflicts:
        raise ValueError(
            f"gripper ID/name already exists in base config: {gripper_id}/{gripper_name}"
        )
    joints.append(gripper)
    return result


def materialize_hardware_config(
    base_path: str | Path,
    motor: Mapping[str, Any],
    output_path: str | Path,
) -> Path:
    base_path = Path(base_path).expanduser().resolve()
    output_path = Path(output_path).expanduser().resolve()
    with base_path.open("r", encoding="utf-8") as stream:
        base = yaml.safe_load(stream)
    if not isinstance(base, dict):
        raise ValueError(f"invalid Dynamixel config: {base_path}")
    merged = merge_gripper_motor(base, motor)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(merged, stream, sort_keys=False, allow_unicode=True)
    return output_path
