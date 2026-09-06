"""Hardware mirror of the direct 12-joint position action."""

from __future__ import annotations

import numpy as np

try:
  from .contract import (
    ACTION_DIM,
    ACTION_SCALE_NEGATIVE,
    ACTION_SCALE_POSITIVE,
    READY_TARGET,
  )
except ImportError:
  from contract import (
    ACTION_DIM,
    ACTION_SCALE_NEGATIVE,
    ACTION_SCALE_POSITIVE,
    READY_TARGET,
  )


class FreeWalkingController:
  """Convert normalized policy output directly into 17 joint targets."""

  def __init__(self, action_filter_alpha: float = 1.0) -> None:
    if not 0.0 < action_filter_alpha <= 1.0:
      raise ValueError("action_filter_alpha must be in (0, 1]")
    self.raw_action = np.zeros(ACTION_DIM, np.float32)
    self.last_action = np.zeros(ACTION_DIM, np.float32)
    self._action_filter_alpha = np.float32(action_filter_alpha)
    self._negative_scale = np.asarray(ACTION_SCALE_NEGATIVE, np.float32)
    self._positive_scale = np.asarray(ACTION_SCALE_POSITIVE, np.float32)
    self._ready = np.asarray(READY_TARGET, np.float32)

  def reset(self) -> None:
    self.raw_action.fill(0.0)
    self.last_action.fill(0.0)

  def step(self, raw_action: np.ndarray) -> np.ndarray:
    self.raw_action[:] = np.clip(
      np.asarray(raw_action, np.float32).reshape(ACTION_DIM), -1.0, 1.0
    )
    if self._action_filter_alpha == 1.0:
      self.last_action[:] = self.raw_action
    else:
      self.last_action += self._action_filter_alpha * (
        self.raw_action - self.last_action
      )
    scale = np.where(
      self.last_action < 0.0, self._negative_scale, self._positive_scale
    )
    target = self._ready.copy()
    target[:ACTION_DIM] += self.last_action * scale
    return target
