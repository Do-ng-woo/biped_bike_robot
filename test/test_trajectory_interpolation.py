import importlib.util
import math
from pathlib import Path

import numpy as np
import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_script(name):
    path = PACKAGE_ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


bridge = load_script("dxl_joint_state_bridge.py")
walker = load_script("op3_walker.py")
sequence = load_script("bike_transform_sequence.py")


def test_single_point_trajectory_is_monotonic_and_reaches_target():
    trajectory = [(3.0, {"knee": -0.7})]
    samples = [
        bridge.interpolate_trajectory({"knee": 0.0}, trajectory, time)[0]["knee"]
        for time in np.linspace(0.0, 3.0, 31)
    ]

    assert samples[0] == 0.0
    assert samples[-1] == -0.7
    assert all(current >= following for current, following in zip(samples, samples[1:]))
    assert samples[15] == pytest.approx(-0.35)


def test_multi_point_trajectory_is_continuous_at_segment_boundary():
    trajectory = [
        (1.0, {"knee": -0.4}),
        (3.0, {"knee": -0.8}),
    ]

    before, _ = bridge.interpolate_trajectory({"knee": 0.0}, trajectory, 0.999)
    boundary, _ = bridge.interpolate_trajectory({"knee": 0.0}, trajectory, 1.0)
    after, _ = bridge.interpolate_trajectory({"knee": 0.0}, trajectory, 1.001)

    assert before["knee"] < 0.0
    assert boundary["knee"] == pytest.approx(-0.4)
    assert abs(after["knee"] - before["knee"]) < 0.001


def test_walker_cycle_has_no_large_command_discontinuity():
    param = walker.WalkingParam()
    param.period_time = 2.0
    param.support_hip_roll_lift = math.radians(20.0)
    param.support_ankle_roll_lift = math.radians(10.0)
    param.pelvis_pitch_forward_lift = math.radians(30.0)
    engine = walker.OP3WalkingEngine(param)
    engine.start()

    samples = np.array(
        [engine.step(0.008) for _ in range(int(param.period_time / 0.008) + 2)]
    )
    assert np.max(np.abs(np.diff(samples, axis=0))) < 0.05


def test_physical_knee_directions_remain_mirrored():
    config_path = PACKAGE_ROOT / "config" / "dynamixel_hardware.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    directions = {
        joint["joint_name"]: joint["direction"] for joint in config["joints"]
    }

    assert directions["l_knee_pitch_jnt"] == -1
    assert directions["r_knee_pitch_jnt"] == 1


def test_arm_base_yaw_can_override_walking_command_limit():
    config_path = PACKAGE_ROOT / "config" / "dynamixel_hardware.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    arm_base = next(
        joint
        for joint in config["joints"]
        if joint["joint_name"] == "arm_base_yaw_jnt"
    )
    motor = bridge.MotorConfig(arm_base, config["conversion"]["tick_per_rad"])

    assert motor.command_abs_limit(2.2) == pytest.approx(3.14159)
    assert motor.command_abs_limit(2.2) > math.radians(179.0)
    assert motor.position_to_tick(3.14159) == 4095


def test_physical_wheel_velocity_conversion_preserves_signed_commands():
    config_path = PACKAGE_ROOT / "config" / "dynamixel_hardware.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    wheel_configs = [
        joint for joint in config["joints"] if joint["control_mode"] == "velocity"
    ]
    wheels = [
        bridge.MotorConfig(joint, config["conversion"]["tick_per_rad"])
        for joint in wheel_configs
    ]
    raw_per_rad_s = config["conversion"]["velocity_raw_per_rad_per_sec"]

    assert [wheel.id for wheel in wheels] == [7, 14]
    assert wheels[0].velocity_to_raw(-1.0, raw_per_rad_s) == -42
    assert wheels[1].velocity_to_raw(1.0, raw_per_rad_s) == 42


def test_transform_replays_stable_revert_path_in_reverse():
    expected_reverse_path = tuple(reversed(sequence.REVERT_SEQUENCE[:-1]))

    assert sequence.TRANSFORM_SEQUENCE[:-1] == expected_reverse_path
    assert sequence.TRANSFORM_SEQUENCE[-1] == sequence.BIKE_FINAL
    assert all(
        len(point) == len(sequence.JOINT_NAMES)
        for point in sequence.REVERT_SEQUENCE + sequence.TRANSFORM_SEQUENCE
    )


def test_transform_respects_shoulder_mechanical_back_limit():
    shoulder_index = sequence.JOINT_NAMES.index("arm_shoulder_pitch_jnt")
    all_points = sequence.REVERT_SEQUENCE + sequence.TRANSFORM_SEQUENCE

    assert all(
        point[shoulder_index] >= sequence.SHOULDER_BACK_LIMIT_RAD
        for point in all_points
    )

    config_path = PACKAGE_ROOT / "config" / "dynamixel_hardware.yaml"
    with config_path.open(encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    shoulder = next(
        joint
        for joint in config["joints"]
        if joint["joint_name"] == "arm_shoulder_pitch_jnt"
    )
    assert shoulder["min_position_rad"] == sequence.SHOULDER_BACK_LIMIT_RAD

    urdf = (PACKAGE_ROOT / "urdf" / "biped_bike_robot.urdf").read_text(
        encoding="utf-8"
    )
    shoulder_joint = urdf.split('name="arm_shoulder_pitch_jnt"', 1)[1].split(
        "</joint>", 1
    )[0]
    assert 'lower="-0.436332"' in shoulder_joint
