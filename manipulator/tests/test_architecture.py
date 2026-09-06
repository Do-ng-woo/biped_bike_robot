from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_only_arbiter_publishes_hardware_trajectory():
    publishers = []
    for path in (ROOT / "nodes").glob("*.py"):
        if 'topics["hardware_trajectory"]' in path.read_text(encoding="utf-8"):
            publishers.append(path.name)
    assert publishers == ["arm_command_arbiter.py"]


def test_mode_manager_zeros_wheels_on_every_tick():
    source = (ROOT / "nodes" / "mode_manager.py").read_text(encoding="utf-8")
    tick = source.split("    def tick(self):", 1)[1].split("    def publish_state", 1)[0]
    assert "self.stop_wheels()" in tick


def test_ready_on_start_uses_measured_smoothstep_trajectory():
    source = (ROOT / "launch_native.py").read_text(encoding="utf-8")
    assert '"startup_ready_posture_on_start:=false"' in source
    assert 'str(ROOT / "nodes" / "smooth_ready.py")' in source
    assert '"--duration", "3.0"' in source
    assert '"--rate", "50.0"' in source
    assert '"enable_opencr_imu:=false"' in source

    ready_source = (ROOT / "nodes" / "smooth_ready.py").read_text(encoding="utf-8")
    assert "load_reference_postures" in ready_source
    assert "build_smooth_samples" in ready_source


def test_runtime_bypasses_stale_overlay_and_crashing_leader_control_manager():
    source = (ROOT / "launch_native.py").read_text(encoding="utf-8")
    assert 'cfg["hardware"]["lower_bridge_script"]' in source
    assert 'cfg["hardware"]["lower_config"]' in source
    assert 'node("leader_openrb_reader", config_path)' in source
    assert "omx_l_leader_ai.launch.py" not in source
    assert "materialize_hardware_config" in source


def test_gripper_is_in_the_full_teleop_learning_contract():
    config = (ROOT / "config" / "system.yaml").read_text(encoding="utf-8")
    assert "motor_ids: [21, 22, 23, 24, 25, 26]" in config
    assert "id: 20" in config
    assert "arm_gripper_jnt" in config
    assert "gripper_joint_1" in config


def test_kneeling_and_arm_enable_are_separate_modes():
    manager = (ROOT / "nodes" / "mode_manager.py").read_text(encoding="utf-8")
    assert '"STABILIZE"' in manager
    assert '"STABLE"' in manager
    assert "press Kneeling Stable and wait for STABLE first" in manager

    arbiter = (ROOT / "nodes" / "arm_command_arbiter.py").read_text(encoding="utf-8")
    assert "build_smooth_samples" in arbiter
    assert "self.kneeling_positions" in arbiter

    web = (ROOT / "web" / "index.html").read_text(encoding="utf-8")
    assert "1. Kneeling Stable" in web
    assert 'startArmMode(\'TELEOP\')' in web
