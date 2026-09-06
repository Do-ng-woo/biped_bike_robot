"""Build term-major proprioceptive history in the free-walking actor order."""

from __future__ import annotations

from collections import deque

import numpy as np

try:
  from .contract import (
    ACTION_DIM,
    GAIT_ACTIVATION_SPEED,
    GAIT_LATERAL_ENCODING_SCALE,
    GAIT_YAW_ENCODING_SCALE,
    LOWER_BODY_JOINTS,
    OBSERVATION_DIM,
    OBSERVATION_HISTORY_LENGTH,
    READY_TARGET,
  )
except ImportError:
  from contract import (
    ACTION_DIM,
    GAIT_ACTIVATION_SPEED,
    GAIT_LATERAL_ENCODING_SCALE,
    GAIT_YAW_ENCODING_SCALE,
    LOWER_BODY_JOINTS,
    OBSERVATION_DIM,
    OBSERVATION_HISTORY_LENGTH,
    READY_TARGET,
  )

_READY_LOWER = np.asarray(READY_TARGET[:ACTION_DIM], np.float32)


def _encode_velocity_command(velocity_command: np.ndarray) -> np.ndarray:
  command = np.asarray(velocity_command, np.float32).reshape(3).copy()
  if np.linalg.norm(command[:2]) + abs(float(command[2])) > 0.025:
    command[0] = max(abs(float(command[0])), GAIT_ACTIVATION_SPEED)
  else:
    command[0] = 0.0
  command[1] *= GAIT_LATERAL_ENCODING_SCALE
  command[2] *= GAIT_YAW_ENCODING_SCALE
  return command


class ObservationHistory:
  """Mirror MJLab's per-term, oldest-to-newest flattened history."""

  _TERM_NAMES = (
    "base_ang_vel",
    "projected_gravity",
    "joint_pos",
    "joint_vel",
    "actions",
    "command",
  )

  def __init__(self) -> None:
    self._buffers: dict[str, deque[np.ndarray]] = {
      name: deque(maxlen=OBSERVATION_HISTORY_LENGTH) for name in self._TERM_NAMES
    }

  def reset(self) -> None:
    for buffer in self._buffers.values():
      buffer.clear()

  def append(
    self,
    joint_positions: dict[str, float],
    joint_velocities: dict[str, float],
    projected_gravity: np.ndarray,
    angular_velocity: np.ndarray,
    velocity_command: np.ndarray,
    last_action: np.ndarray,
  ) -> np.ndarray:
    missing = [name for name in LOWER_BODY_JOINTS if name not in joint_positions]
    if missing:
      raise ValueError(f"Missing free-walking joints: {', '.join(missing)}")

    joint_pos = np.asarray(
      [joint_positions[name] for name in LOWER_BODY_JOINTS], np.float32
    )
    terms = {
      "base_ang_vel": np.asarray(angular_velocity, np.float32).reshape(3),
      "projected_gravity": np.asarray(projected_gravity, np.float32).reshape(3),
      "joint_pos": joint_pos - _READY_LOWER,
      "joint_vel": np.asarray(
        [joint_velocities.get(name, 0.0) for name in LOWER_BODY_JOINTS], np.float32
      ),
      "actions": np.asarray(last_action, np.float32).reshape(ACTION_DIM),
      "command": _encode_velocity_command(velocity_command),
    }
    for name, value in terms.items():
      buffer = self._buffers[name]
      if not buffer:
        buffer.extend(value.copy() for _ in range(OBSERVATION_HISTORY_LENGTH))
      else:
        buffer.append(value.copy())

    observation = np.concatenate(
      [np.concatenate(tuple(self._buffers[name])) for name in self._TERM_NAMES]
    ).astype(np.float32)
    if observation.shape != (OBSERVATION_DIM,):
      raise RuntimeError(
        f"Expected observation ({OBSERVATION_DIM},), got {observation.shape}"
      )
    if not np.all(np.isfinite(observation)):
      raise ValueError("Observation contains NaN or infinity")
    return observation
