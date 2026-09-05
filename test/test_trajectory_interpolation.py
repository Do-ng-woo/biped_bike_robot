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
web_control = load_script("web_control.py")


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
    engine = walker.IKWalkerEngine(param)
    engine.start()

    samples = np.array(
        [engine.step(0.008) for _ in range(int(param.period_time / 0.008) + 2)]
    )
    assert np.max(np.abs(np.diff(samples, axis=0))) < 0.05


def test_walker_uses_physical_leg_dimensions_and_base_stance():
    param = walker.WalkingParam()

    assert walker.THIGH_LENGTH == pytest.approx(0.059)
    assert walker.CALF_LENGTH == pytest.approx(0.1125)
    assert walker.LEG_LENGTH == pytest.approx(0.1715)
    assert walker.HIP_SPACING == pytest.approx(0.125)
    assert param.init_y_offset == pytest.approx(0.005)
    assert walker.HIP_SPACING + param.init_y_offset == pytest.approx(0.130)


def test_swing_lift_starts_and_lands_at_zero_with_zero_boundary_velocity():
    engine = walker.IKWalkerEngine()
    start = engine.l_ssp_start_time
    end = engine.l_ssp_end_time
    dt = 1e-5

    assert engine._ssp_lift_profile(start, start, end) == 0.0
    assert engine._ssp_lift_profile((start + end) / 2, start, end) == pytest.approx(1.0)
    assert engine._ssp_lift_profile(end, start, end) == 0.0
    assert engine._ssp_lift_profile(start + dt, start, end) < 1e-8
    assert engine._ssp_lift_profile(end - dt, start, end) < 1e-8


def test_left_swing_endpoint_lifts_once_and_returns_to_support_height():
    engine = walker.IKWalkerEngine()
    start = engine.l_ssp_start_time
    middle = (engine.l_ssp_start_time + engine.l_ssp_end_time) / 2
    end = engine.l_ssp_end_time

    relative_heights = []
    for time in (start, middle, end):
        engine.time = time
        engine._compute_leg_angles()
        endpoints = engine.last_foot_endpoints
        relative_heights.append(endpoints[8] - endpoints[2])

    assert relative_heights == pytest.approx([0.0, engine.param.z_move_amplitude, 0.0])


def test_roll_correction_does_not_change_base_ik_or_foot_targets():
    base_param = walker.WalkingParam(
        roll_correction=0.0,
        hip_pitch_forward=0.0,
    )
    base_engine = walker.IKWalkerEngine(base_param)
    corrected_engine = walker.IKWalkerEngine(
        walker.WalkingParam(hip_pitch_forward=0.0)
    )

    for time in (0.0, base_engine.l_ssp_end_time, base_engine.r_ssp_end_time):
        base_engine.time = time
        corrected_engine.time = time
        base_angles = base_engine._compute_leg_angles()
        corrected_angles = corrected_engine._compute_leg_angles()

        expected_delta = (
            walker.LEG_ROLL_CORRECTION_SIGNS
            * corrected_engine.param.roll_correction
        )
        assert corrected_angles - base_angles == pytest.approx(expected_delta)
        assert corrected_engine.last_foot_endpoints == pytest.approx(
            base_engine.last_foot_endpoints
        )


def test_forward_hip_pitch_is_an_independent_default_posture_offset():
    base_engine = walker.IKWalkerEngine(
        walker.WalkingParam(hip_pitch_forward=0.0)
    )
    forward_engine = walker.IKWalkerEngine()

    base_angles = base_engine._compute_leg_angles()
    forward_angles = forward_engine._compute_leg_angles()
    expected_delta = (
        walker.LEG_HIP_PITCH_FORWARD_SIGNS
        * math.radians(10.0)
    )

    assert forward_angles - base_angles == pytest.approx(expected_delta)
    assert forward_engine.last_foot_endpoints == pytest.approx(
        base_engine.last_foot_endpoints
    )


def test_walker_end_transition_removes_only_roll_correction():
    base_command = np.linspace(-0.8, 0.8, 17)
    correction = math.radians(3.0)
    corrected_command = base_command.copy()
    corrected_command[list(walker.ROLL_JOINT_INDICES)] += (
        walker.PUBLISHED_ROLL_CORRECTION_SIGNS * correction
    )

    normalized = walker.without_roll_correction(corrected_command, correction)

    assert normalized == pytest.approx(base_command)
    assert corrected_command != pytest.approx(base_command)


def test_web_walking_command_uses_ik_walker_defaults():
    command = web_control.walk_command(2)

    assert command == [
        "ros2",
        "run",
        "biped_bike_robot",
        "ik_walker.py",
        "--ros-args",
        "-p",
        "num_cycles:=2",
    ]


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
    left_hip_yaw_index = sequence.JOINT_NAMES.index("l_hip_yaw_jnt")
    left_hip_pitch_index = sequence.JOINT_NAMES.index("l_hip_pitch_jnt")
    left_knee_pitch_index = sequence.JOINT_NAMES.index("l_knee_pitch_jnt")
    left_ankle_pitch_index = sequence.JOINT_NAMES.index("l_ankle_pitch_jnt")
    right_hip_yaw_index = sequence.JOINT_NAMES.index("r_hip_yaw_jnt")
    right_hip_pitch_index = sequence.JOINT_NAMES.index("r_hip_pitch_jnt")
    right_knee_pitch_index = sequence.JOINT_NAMES.index("r_knee_pitch_jnt")
    right_ankle_pitch_index = sequence.JOINT_NAMES.index("r_ankle_pitch_jnt")
    shoulder_index = sequence.JOINT_NAMES.index("arm_shoulder_pitch_jnt")
    wrist_pitch_index = sequence.JOINT_NAMES.index("arm_wrist_pitch_jnt")
    wrist_roll_index = sequence.JOINT_NAMES.index("arm_wrist_roll_jnt")

    wrist_return_start = sequence.REVERT_SEQUENCE[5]
    wrist_return_end = sequence.REVERT_SEQUENCE[6]
    yaw_return_end = sequence.REVERT_SEQUENCE[7]
    pre_rise = sequence.REVERT_SEQUENCE[8]
    rise_early = sequence.REVERT_SEQUENCE[9]
    rise_mid = sequence.REVERT_SEQUENCE[10]
    rise_late = sequence.REVERT_SEQUENCE[11]
    ready = sequence.REVERT_SEQUENCE[-1]

    assert len(sequence.REVERT_SEQUENCE) == 13
    assert sequence.REVERT_SEQUENCE[-1] == sequence.HARDWARE_READY
    assert sequence.HARDWARE_READY == sequence.READY
    assert sequence.REVERT_SEQUENCE[0][left_hip_yaw_index] == pytest.approx(
        sequence.BIKE_HIP_YAW_INWARD_RAD
    )
    assert sequence.REVERT_SEQUENCE[0][right_hip_yaw_index] == pytest.approx(
        -sequence.BIKE_HIP_YAW_INWARD_RAD
    )
    assert wrist_return_start[yaw_index] == pytest.approx(math.pi, abs=1e-5)
    assert wrist_return_start[shoulder_index] == pytest.approx(
        sequence.SHOULDER_YAWED_SUPPORT_RAD
    )
    assert wrist_return_start[wrist_pitch_index] == pytest.approx(
        sequence.WRIST_PITCH_DOWN_RAD
    )
    assert wrist_return_start[wrist_roll_index] == pytest.approx(
        sequence.WRIST_ROLL_YAWED_RAD
    )
    assert wrist_return_end[yaw_index] == pytest.approx(math.pi, abs=1e-5)
    assert wrist_return_end[shoulder_index] == pytest.approx(
        sequence.SHOULDER_READY_RAD
    )
    assert wrist_return_end[wrist_pitch_index] == pytest.approx(0.0)
    assert wrist_return_end[wrist_roll_index] == pytest.approx(
        sequence.WRIST_ROLL_YAWED_RAD
    )
    assert yaw_return_end[yaw_index] == pytest.approx(0.0)
    assert yaw_return_end[shoulder_index] == pytest.approx(
        sequence.SHOULDER_READY_RAD
    )
    assert yaw_return_end[wrist_pitch_index] == pytest.approx(0.0)
    assert yaw_return_end[wrist_roll_index] == pytest.approx(0.0)
    assert yaw_return_end[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD
    )
    assert yaw_return_end[right_hip_pitch_index] == pytest.approx(
        sequence.DEEP_SQUAT_HIP_PITCH_RAD
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
    assert pre_rise[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD + sequence.REVERT_PRE_RISE_HIP_RETURN_RAD
    )
    assert pre_rise[right_hip_pitch_index] == pytest.approx(
        sequence.DEEP_SQUAT_HIP_PITCH_RAD - sequence.REVERT_PRE_RISE_HIP_RETURN_RAD
    )
    assert pre_rise[left_knee_pitch_index] == pytest.approx(
        yaw_return_end[left_knee_pitch_index]
    )
    assert pre_rise[right_knee_pitch_index] == pytest.approx(
        yaw_return_end[right_knee_pitch_index]
    )
    assert rise_early[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD + sequence.REVERT_RISE_EARLY_HIP_RETURN_RAD
    )
    assert rise_early[right_hip_pitch_index] == pytest.approx(
        sequence.DEEP_SQUAT_HIP_PITCH_RAD - sequence.REVERT_RISE_EARLY_HIP_RETURN_RAD
    )
    assert rise_early[left_knee_pitch_index] == pytest.approx(
        yaw_return_end[left_knee_pitch_index]
        + sequence.REVERT_RISE_EARLY_RATIO
        * (ready[left_knee_pitch_index] - yaw_return_end[left_knee_pitch_index])
    )
    assert rise_early[right_knee_pitch_index] == pytest.approx(
        yaw_return_end[right_knee_pitch_index]
        + sequence.REVERT_RISE_EARLY_RATIO
        * (ready[right_knee_pitch_index] - yaw_return_end[right_knee_pitch_index])
    )
    assert rise_mid[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD + sequence.REVERT_RISE_MID_HIP_RETURN_RAD
    )
    assert rise_mid[left_knee_pitch_index] == pytest.approx(
        yaw_return_end[left_knee_pitch_index]
        + sequence.REVERT_RISE_MID_RATIO
        * (ready[left_knee_pitch_index] - yaw_return_end[left_knee_pitch_index])
    )
    assert rise_late[left_hip_pitch_index] == pytest.approx(
        -sequence.DEEP_SQUAT_HIP_PITCH_RAD + sequence.REVERT_RISE_LATE_HIP_RETURN_RAD
    )
    assert rise_late[left_knee_pitch_index] == pytest.approx(
        yaw_return_end[left_knee_pitch_index]
        + sequence.REVERT_RISE_LATE_RATIO
        * (ready[left_knee_pitch_index] - yaw_return_end[left_knee_pitch_index])
    )
    assert len(sequence.REVERT_POINT_TIME_FACTORS) == len(sequence.REVERT_SEQUENCE)
    assert sequence.REVERT_POINT_TIME_FACTORS[7:12] == (8.0, 8.5, 9.0, 9.6, 10.2)
    assert all(
        len(point) == len(sequence.JOINT_NAMES)
        for point in sequence.REVERT_SEQUENCE + sequence.TRANSFORM_SEQUENCE
    )


def test_transform_yaws_wrist_before_pitching_claw_down():
    elbow_index = sequence.JOINT_NAMES.index("arm_elbow_pitch_jnt")
    left_hip_yaw_index = sequence.JOINT_NAMES.index("l_hip_yaw_jnt")
    right_hip_yaw_index = sequence.JOINT_NAMES.index("r_hip_yaw_jnt")
    wrist_pitch_index = sequence.JOINT_NAMES.index("arm_wrist_pitch_jnt")
    wrist_roll_index = sequence.JOINT_NAMES.index("arm_wrist_roll_jnt")

    before_yaw = sequence.TRANSFORM_SEQUENCE[1]
    after_yaw = sequence.TRANSFORM_SEQUENCE[2]
    after_pitch_down = sequence.TRANSFORM_SEQUENCE[3]
    bike_final = sequence.TRANSFORM_SEQUENCE[-1]

    assert len(sequence.TRANSFORM_SEQUENCE) == 9
    assert before_yaw[wrist_pitch_index] == pytest.approx(0.0)
    assert before_yaw[wrist_roll_index] == pytest.approx(0.0)
    assert after_yaw[wrist_pitch_index] == pytest.approx(0.0)
    assert after_yaw[wrist_roll_index] == pytest.approx(
        sequence.WRIST_ROLL_YAWED_RAD
    )
    assert after_pitch_down[wrist_pitch_index] == pytest.approx(
        sequence.WRIST_PITCH_DOWN_RAD
    )
    assert after_pitch_down[wrist_roll_index] == pytest.approx(
        sequence.WRIST_ROLL_YAWED_RAD
    )
    assert bike_final[wrist_pitch_index] == pytest.approx(
        sequence.WRIST_PITCH_DOWN_RAD
    )
    assert bike_final[wrist_roll_index] == pytest.approx(
        sequence.WRIST_ROLL_YAWED_RAD
    )
    assert bike_final[elbow_index] == pytest.approx(
        sequence.ELBOW_LIFT_RAD
    )
    assert bike_final[elbow_index] == pytest.approx(
        math.radians(20.0),
        abs=1e-6,
    )
    assert bike_final[left_hip_yaw_index] == pytest.approx(
        sequence.BIKE_HIP_YAW_INWARD_RAD
    )
    assert bike_final[right_hip_yaw_index] == pytest.approx(
        -sequence.BIKE_HIP_YAW_INWARD_RAD
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
