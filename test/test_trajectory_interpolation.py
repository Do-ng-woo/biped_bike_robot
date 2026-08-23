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
walker = load_script("ik_walker.py")
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
    engine = walker.IKWalkerEngine(param)
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
    assert motor.position_to_tick(3.14159) == 0
    assert motor.position_to_tick(-3.14159) == 4095


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


def test_revert_restores_shoulder_before_arm_yaw_returns():
    yaw_index = sequence.JOINT_NAMES.index("arm_base_yaw_jnt")
    left_hip_pitch_index = sequence.JOINT_NAMES.index("l_hip_pitch_jnt")
    left_knee_pitch_index = sequence.JOINT_NAMES.index("l_knee_pitch_jnt")
    left_ankle_pitch_index = sequence.JOINT_NAMES.index("l_ankle_pitch_jnt")
    right_hip_pitch_index = sequence.JOINT_NAMES.index("r_hip_pitch_jnt")
    right_knee_pitch_index = sequence.JOINT_NAMES.index("r_knee_pitch_jnt")
    right_ankle_pitch_index = sequence.JOINT_NAMES.index("r_ankle_pitch_jnt")
    shoulder_index = sequence.JOINT_NAMES.index("arm_shoulder_pitch_jnt")
    wrist_index = sequence.JOINT_NAMES.index("arm_wrist_pitch_jnt")

    wrist_return_start = sequence.REVERT_SEQUENCE[5]
    wrist_return_end = sequence.REVERT_SEQUENCE[6]
    yaw_return_end = sequence.REVERT_SEQUENCE[7]
    ready = sequence.REVERT_SEQUENCE[-1]

    assert len(sequence.REVERT_SEQUENCE) == 10
    assert sequence.REVERT_SEQUENCE[-1] == sequence.HARDWARE_READY
    assert sequence.HARDWARE_READY == sequence.READY
    assert wrist_return_start[yaw_index] == pytest.approx(math.pi, abs=1e-5)
    assert wrist_return_start[shoulder_index] == pytest.approx(
        sequence.SHOULDER_YAWED_SUPPORT_RAD
    )
    assert wrist_return_start[wrist_index] == pytest.approx(
        sequence.WRIST_PITCH_DOWN_RAD
    )
    assert wrist_return_end[yaw_index] == pytest.approx(math.pi, abs=1e-5)
    assert wrist_return_end[shoulder_index] == pytest.approx(
        sequence.SHOULDER_READY_RAD
    )
    assert wrist_return_end[wrist_index] == pytest.approx(0.0)
    assert yaw_return_end[yaw_index] == pytest.approx(0.0)
    assert yaw_return_end[shoulder_index] == pytest.approx(
        sequence.SHOULDER_READY_RAD
    )
    assert yaw_return_end[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD + sequence.REVERT_HIP_BACK_OFFSET_RAD
    )
    assert yaw_return_end[right_hip_pitch_index] == pytest.approx(
        sequence.DEEP_SQUAT_HIP_PITCH_RAD - sequence.REVERT_HIP_BACK_OFFSET_RAD
    )
    assert ready[shoulder_index] == pytest.approx(sequence.SHOULDER_READY_RAD)
    assert ready[left_hip_pitch_index] == pytest.approx(
        -sequence.READY_HIP_PITCH_RAD - sequence.READY_HIP_FORWARD_OFFSET_RAD
    )
    assert ready[right_hip_pitch_index] == pytest.approx(
        sequence.READY_HIP_PITCH_RAD + sequence.READY_HIP_FORWARD_OFFSET_RAD
    )
    assert ready[left_knee_pitch_index] == pytest.approx(
        sequence.READY_KNEE_PITCH_RAD
    )
    assert ready[right_knee_pitch_index] == pytest.approx(
        sequence.READY_KNEE_PITCH_RAD
    )
    assert ready[left_ankle_pitch_index] == pytest.approx(
        sequence.READY_ANKLE_PITCH_RAD
    )
    assert ready[right_ankle_pitch_index] == pytest.approx(
        sequence.READY_ANKLE_PITCH_RAD
    )
    assert sequence.READY_ANKLE_FORWARD_OFFSET_RAD == pytest.approx(
        math.radians(10.0),
        abs=1e-6,
    )
    assert sequence.READY_HIP_FORWARD_OFFSET_RAD == pytest.approx(
        0.0
    )
    assert all(
        len(point) == len(sequence.JOINT_NAMES)
        for point in sequence.REVERT_SEQUENCE + sequence.TRANSFORM_SEQUENCE
    )


def test_transform_respects_shoulder_mechanical_back_limit():
    shoulder_index = sequence.JOINT_NAMES.index("arm_shoulder_pitch_jnt")
    all_points = sequence.REVERT_SEQUENCE + sequence.TRANSFORM_SEQUENCE

    assert all(
        point[shoulder_index] >= sequence.SHOULDER_YAWED_SUPPORT_RAD
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
    assert shoulder["min_position_rad"] == sequence.SHOULDER_YAWED_SUPPORT_RAD

    urdf = (PACKAGE_ROOT / "urdf" / "biped_bike_robot.urdf").read_text(
        encoding="utf-8"
    )
    shoulder_joint = urdf.split('name="arm_shoulder_pitch_jnt"', 1)[1].split(
        "</joint>", 1
    )[0]
    assert 'lower="-2.0944"' in shoulder_joint


def test_startup_ready_ignores_forward_lean_parameter():
    source = (
        PACKAGE_ROOT / "scripts" / "dxl_joint_state_bridge.py"
    ).read_text(encoding="utf-8")
    startup_ready = source.split("def _send_startup_ready_posture", 1)[1].split(
        "def joint_state_callback", 1
    )[0]

    assert 'get_parameter("startup_forward_lean_deg")' not in startup_ready
    assert '"l_hip_pitch_jnt": -READY_HIP_PITCH_RAD' in startup_ready
    assert '"r_hip_pitch_jnt": READY_HIP_PITCH_RAD' in startup_ready
