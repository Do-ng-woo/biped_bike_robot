from common.joints import JointMapping, ordered_positions


def config():
    return {
        "leader_joint_names": ["j1", "j2"],
        "follower_joint_names": ["a", "b"],
        "direction": [1, -1],
        "offset": [0.1, 0.2],
        "min_position": [-1, -1],
        "max_position": [1, 1],
    }


def test_mapping_reorders_and_clamps():
    mapping = JointMapping.from_config(config())
    leader = mapping.extract(["j2", "j1"], [0.4, 2.0])
    assert leader == [2.0, 0.4]
    assert mapping.map(leader) == [1.0, -0.2]


def test_alignment_is_continuous():
    mapping = JointMapping.from_config(config())
    leader = [0.7, -0.3]
    follower = [0.2, 0.5]
    offsets = mapping.alignment_offsets(leader, follower)
    assert mapping.map(leader, offsets) == follower


def test_scale_compresses_leader_range_after_alignment():
    cfg = config()
    cfg["scale"] = [1.0, 0.5]
    mapping = JointMapping.from_config(cfg)
    leader_start = [0.0, 0.0]
    follower_start = [0.0, 0.0]
    offsets = mapping.alignment_offsets(leader_start, follower_start)
    assert mapping.map([0.5, -1.0], offsets) == [0.5, 0.5]


def test_ordered_positions():
    assert ordered_positions(["b", "a"], [2, 1], ["a", "b"]) == [1.0, 2.0]
