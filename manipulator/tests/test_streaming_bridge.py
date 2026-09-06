import importlib.util
from pathlib import Path


def load_adapter():
    path = Path(__file__).parents[1] / "nodes" / "dxl_joint_state_bridge_streaming.py"
    spec = importlib.util.spec_from_file_location("streaming_bridge", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_cached_start_positions_prefers_measured_position_without_serial_read():
    module = load_adapter()

    class Motor:
        id = 20
        center_tick = 2038

        @staticmethod
        def tick_to_position(tick):
            return float(tick) / 1000.0

    bridge = type(
        "Bridge",
        (),
        {
            "joint_to_motor": {"arm_gripper_jnt": Motor()},
            "latest_present_positions": {"arm_gripper_jnt": 0.42},
            "last_goal_ticks": {20: 1500},
        },
    )()
    assert module.cached_start_positions(bridge, ["arm_gripper_jnt"]) == {
        "arm_gripper_jnt": 0.42
    }


def test_cached_start_positions_falls_back_to_last_goal():
    module = load_adapter()

    class Motor:
        id = 20
        center_tick = 2038

        @staticmethod
        def tick_to_position(tick):
            return float(tick) / 1000.0

    bridge = type(
        "Bridge",
        (),
        {
            "joint_to_motor": {"arm_gripper_jnt": Motor()},
            "latest_present_positions": {},
            "last_goal_ticks": {20: 1500},
        },
    )()
    assert module.cached_start_positions(bridge, ["arm_gripper_jnt"]) == {
        "arm_gripper_jnt": 1.5
    }


def test_one_point_stream_is_extracted_for_immediate_write():
    module = load_adapter()
    point = type("Point", (), {"positions": [0.2, 0.7]})()
    msg = type(
        "Message",
        (),
        {"joint_names": ["joint1", "gripper"], "points": [point]},
    )()
    assert module.one_point_positions(msg, {"joint1": object(), "gripper": object()}) == {
        "joint1": 0.2,
        "gripper": 0.7,
    }


def test_multi_point_posture_keeps_original_interpolation_path():
    module = load_adapter()
    msg = type("Message", (), {"points": [object(), object()]})()
    assert module.one_point_positions(msg, {}) is None


def test_position_read_groups_keep_arm_feedback_independent():
    module = load_adapter()
    groups = module.chunked(list(range(1, 21)), module.POSITION_READ_GROUP_SIZE)
    assert groups == [
        [1, 2, 3, 4, 5, 6, 7],
        [8, 9, 10, 11, 12, 13, 14],
        [15, 16, 17, 18, 19, 20],
    ]
