#!/usr/bin/env python3
"""Check the free-walking training/hardware observation and action contract."""

from __future__ import annotations

import numpy as np

try:
  from .contract import (
    ACTION_DIM,
    ACTION_SCALE_NEGATIVE,
    ACTION_SCALE_POSITIVE,
    LOWER_BODY_JOINTS,
    OBSERVATION_DIM,
    OBSERVATION_SLICES,
    READY_TARGET,
  )
  from .controller import FreeWalkingController
  from .observation import ObservationHistory
except ImportError:
  from contract import (
    ACTION_DIM,
    ACTION_SCALE_NEGATIVE,
    ACTION_SCALE_POSITIVE,
    LOWER_BODY_JOINTS,
    OBSERVATION_DIM,
    OBSERVATION_SLICES,
    READY_TARGET,
  )
  from controller import FreeWalkingController
  from observation import ObservationHistory


def main() -> None:
  controller = FreeWalkingController()
  ready = np.asarray(READY_TARGET, np.float32)
  np.testing.assert_allclose(controller.step(np.zeros(ACTION_DIM)), ready)
  assert np.all(np.asarray(ACTION_SCALE_NEGATIVE) > 0.0)
  assert np.all(np.asarray(ACTION_SCALE_POSITIVE) > 0.0)

  negative_target = controller.step(-np.ones(ACTION_DIM, np.float32))
  positive_target = controller.step(np.ones(ACTION_DIM, np.float32))
  assert np.all(negative_target[:ACTION_DIM] < ready[:ACTION_DIM])
  assert np.all(positive_target[:ACTION_DIM] > ready[:ACTION_DIM])

  action = np.linspace(-1.0, 1.0, ACTION_DIM, dtype=np.float32)
  target = controller.step(action)
  expected = ready.copy()
  scale = np.where(
    action < 0.0,
    np.asarray(ACTION_SCALE_NEGATIVE, np.float32),
    np.asarray(ACTION_SCALE_POSITIVE, np.float32),
  )
  expected[:ACTION_DIM] += action * scale
  np.testing.assert_allclose(target, expected)
  np.testing.assert_allclose(target[ACTION_DIM:], ready[ACTION_DIM:])

  positions = {
    name: READY_TARGET[index] for index, name in enumerate(LOWER_BODY_JOINTS)
  }
  velocities = {name: 0.0 for name in LOWER_BODY_JOINTS}
  history = ObservationHistory()
  observation = history.append(
    positions,
    velocities,
    np.asarray((0.0, 0.0, -1.0), np.float32),
    np.asarray((0.1, 0.2, 0.3), np.float32),
    np.asarray((0.08, 0.0, 0.0), np.float32),
    action,
  )
  assert observation.shape == (OBSERVATION_DIM,)
  for start, end in OBSERVATION_SLICES.values():
    assert 0 <= start < end <= OBSERVATION_DIM
  np.testing.assert_allclose(observation[210:225], np.tile((0.08, 0.0, 0.0), 5))

  history.reset()
  lateral_observation = history.append(
    positions,
    velocities,
    np.asarray((0.0, 0.0, -1.0), np.float32),
    np.zeros(3, np.float32),
    np.asarray((0.0, 0.04, 0.0), np.float32),
    action,
  )
  np.testing.assert_allclose(
    lateral_observation[210:225], np.tile((0.08, 0.01, 0.0), 5)
  )

  history.reset()
  yaw_observation = history.append(
    positions,
    velocities,
    np.asarray((0.0, 0.0, -1.0), np.float32),
    np.zeros(3, np.float32),
    np.asarray((0.0, 0.0, -0.25), np.float32),
    action,
  )
  np.testing.assert_allclose(
    yaw_observation[210:225], np.tile((0.08, 0.0, -0.05), 5)
  )
  print("Free-walking contract OK: observation=225D, action=12D")


if __name__ == "__main__":
  main()
