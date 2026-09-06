"""NumPy inference for exported deterministic RSL-RL actors."""

from __future__ import annotations

from pathlib import Path

import numpy as np


class NumpyPolicy:
  def __init__(self, path: str | Path):
    bundle = np.load(path, allow_pickle=False)
    self.mean = bundle["obs_mean"].astype(np.float32)
    self.std = bundle["obs_std"].astype(np.float32)
    self.eps = float(bundle["normalizer_eps"])
    layer_count = int(bundle["layer_count"])
    self.weights = [
      bundle[f"weight_{index}"].astype(np.float32)
      for index in range(layer_count)
    ]
    self.biases = [
      bundle[f"bias_{index}"].astype(np.float32)
      for index in range(layer_count)
    ]
    self.actor_joint_names = tuple(str(v) for v in bundle["actor_joint_names"])
    self.target_joint_names = tuple(str(v) for v in bundle["target_joint_names"])
    self.policy_dt = float(bundle["policy_dt"])

  @staticmethod
  def _elu(value: np.ndarray) -> np.ndarray:
    result = value.copy()
    negative = result <= 0.0
    result[negative] = np.expm1(result[negative])
    return result

  def __call__(self, observation: np.ndarray) -> np.ndarray:
    value = np.asarray(observation, np.float32).reshape(1, -1)
    value = (value - self.mean) / (self.std + self.eps)
    for index, (weight, bias) in enumerate(
      zip(self.weights, self.biases, strict=True)
    ):
      value = value @ weight.T + bias
      if index + 1 < len(self.weights):
        value = self._elu(value)
    return value[0].astype(np.float32)

  def normalized_observation(self, observation: np.ndarray) -> np.ndarray:
    value = np.asarray(observation, np.float32).reshape(1, -1)
    return ((value - self.mean) / (self.std + self.eps))[0]
