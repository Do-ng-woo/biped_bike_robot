import pytest

from common.motion import build_smooth_samples, smoothstep


def test_smoothstep_clamps_and_eases_endpoints():
    assert smoothstep(-1.0) == 0.0
    assert smoothstep(0.0) == 0.0
    assert smoothstep(0.5) == 0.5
    assert smoothstep(1.0) == 1.0
    assert smoothstep(2.0) == 1.0


def test_three_second_fifty_hz_trajectory_has_measured_start_and_exact_target():
    samples = build_smooth_samples([0.2, -0.4], [1.2, 0.6], 3.0, 50.0)
    assert len(samples) == 151
    assert samples[0] == (0.0, [0.2, -0.4])
    assert samples[-1][0] == pytest.approx(3.0)
    assert samples[-1][1] == pytest.approx([1.2, 0.6])
    assert samples[75][1] == pytest.approx([0.7, 0.1])


def test_smooth_trajectory_rejects_invalid_shapes():
    with pytest.raises(ValueError):
        build_smooth_samples([0.0], [0.0, 1.0], 3.0, 50.0)
