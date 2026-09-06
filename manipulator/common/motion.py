from __future__ import annotations

import math
from typing import Sequence


def smoothstep(progress: float) -> float:
    """Cubic ease-in/ease-out used by the proven MJLab hardware bridge."""
    value = min(max(float(progress), 0.0), 1.0)
    return value * value * (3.0 - 2.0 * value)


def build_smooth_samples(
    start: Sequence[float],
    target: Sequence[float],
    duration_sec: float,
    rate_hz: float,
) -> list[tuple[float, list[float]]]:
    """Return absolute trajectory samples from the measured pose to target."""
    start_values = [float(value) for value in start]
    target_values = [float(value) for value in target]
    if len(start_values) != len(target_values):
        raise ValueError("start and target must have the same joint count")
    if not start_values:
        raise ValueError("trajectory must contain at least one joint")
    if duration_sec <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration and rate must be positive")
    if not all(math.isfinite(value) for value in start_values + target_values):
        raise ValueError("trajectory contains a non-finite value")

    steps = max(1, int(round(duration_sec * rate_hz)))
    samples: list[tuple[float, list[float]]] = []
    for index in range(steps + 1):
        progress = index / steps
        alpha = smoothstep(progress)
        positions = [
            initial + alpha * (goal - initial)
            for initial, goal in zip(start_values, target_values, strict=True)
        ]
        samples.append((duration_sec * progress, positions))
    return samples
