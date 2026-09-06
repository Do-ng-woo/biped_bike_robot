import pytest

from common.hardware_config import merge_gripper_motor


def test_gripper_motor_is_appended_without_mutating_base():
    base = {"joints": [{"id": 19, "joint_name": "arm_wrist_roll_jnt"}]}
    motor = {"id": 20, "joint_name": "arm_gripper_jnt"}
    merged = merge_gripper_motor(base, motor)
    assert [joint["id"] for joint in merged["joints"]] == [19, 20]
    assert len(base["joints"]) == 1


def test_duplicate_gripper_id_is_rejected():
    base = {"joints": [{"id": 20, "joint_name": "some_other_joint"}]}
    with pytest.raises(ValueError):
        merge_gripper_motor(base, {"id": 20, "joint_name": "arm_gripper_jnt"})
