from pathlib import Path

from common.config import load_config
from common.postures import load_reference_postures


def test_fixed_kneeling_pose_contract():
    cfg = load_config()
    lower = cfg["lower_body"]
    posture = load_reference_postures(cfg)
    kneeling = posture.kneeling[:12]
    assert len(lower["joint_names"]) == 12
    assert len(kneeling) == 12
    assert kneeling[2] == -0.7764
    assert kneeling[8] == 0.7764
    assert kneeling[3] == kneeling[9] == -2.0944
    assert lower["active_drift_tolerance_rad"] >= lower["position_tolerance_rad"]


def test_whole_body_and_reset_contract():
    cfg = load_config()
    lower = cfg["lower_body"]
    arm = cfg["arm"]
    posture = load_reference_postures(cfg)
    assert len(lower["joint_names"] + arm["follower_joint_names"]) == 18
    assert not set(lower["joint_names"]) & set(arm["follower_joint_names"])
    assert len(posture.kneeling[12:]) == len(arm["follower_joint_names"]) == 6
    assert arm["leader_joint_names"][-1] == "gripper_joint_1"
    assert arm["follower_joint_names"][-1] == "arm_gripper_jnt"
    assert posture.ready[-1] == posture.kneeling[-1] == 0.0
    assert cfg["leader_reader"]["motor_ids"][-1] == 26
    assert cfg["gripper_motor"]["id"] == 20
    assert cfg["gripper_motor"]["operating_mode"] == 3
    assert cfg["gripper_motor"]["center_tick"] == 2055
    assert cfg["gripper_motor"]["direction"] == -1
    assert cfg["leader_reader"]["direction"][-1] == 1.0
    assert cfg["arm"]["min_position"][-1] == 0.0
    assert cfg["arm"]["scale"][-1] == 0.72
    assert cfg["arm"]["max_position"][-1] == 1.22
    assert cfg["arm"]["publish_rate_hz"] == 30.0
    assert cfg["arm"]["max_velocity_rad_s"] == 2.0
    assert cfg["lower_body"]["active_drift_tolerance_rad"] == 0.18
    assert cfg["lower_body"]["active_drift_fault_sec"] == 0.75
    assert cfg["recording"]["max_sync_skew_sec"] <= 0.05


def test_all_runtime_data_stays_inside_module():
    cfg = load_config()
    root = Path(cfg["_root"])
    assert not Path(cfg["recording"]["data_root"]).is_absolute()
    assert not Path(cfg["training"]["model_root"]).is_absolute()
    assert root == Path(__file__).resolve().parents[1]
    assert (root / cfg["recording"]["data_root"]).resolve().is_relative_to(root)
    assert (root / cfg["training"]["model_root"]).resolve().is_relative_to(root)
