"""OpenCR IMU mounting correction and simulation-frame conversion."""

from __future__ import annotations

import math

import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
  q = np.asarray(q, dtype=np.float64)
  norm = np.linalg.norm(q)
  if norm <= 1.0e-12:
    return np.asarray([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
  return q / norm


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
  ax, ay, az, aw = a
  bx, by, bz, bw = b
  return np.asarray(
    (
      aw * bx + ax * bw + ay * bz - az * by,
      aw * by - ax * bz + ay * bw + az * bx,
      aw * bz + ax * by - ay * bx + az * bw,
      aw * bw - ax * bx - ay * by - az * bz,
    ),
    dtype=np.float64,
  )


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
  cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
  cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
  cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
  return normalize_quaternion(
    np.asarray(
      (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
      )
    )
  )


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
  x, y, z, w = normalize_quaternion(q)
  return np.asarray(
    (
      (1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
      (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
      (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)),
    ),
    dtype=np.float64,
  )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
  m = np.asarray(matrix, dtype=np.float64)
  trace = np.trace(m)
  if trace > 0.0:
    s = math.sqrt(trace + 1.0) * 2.0
    q = (
      (m[2, 1] - m[1, 2]) / s,
      (m[0, 2] - m[2, 0]) / s,
      (m[1, 0] - m[0, 1]) / s,
      0.25 * s,
    )
  elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
    s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
    q = (
      0.25 * s,
      (m[0, 1] + m[1, 0]) / s,
      (m[0, 2] + m[2, 0]) / s,
      (m[2, 1] - m[1, 2]) / s,
    )
  elif m[1, 1] > m[2, 2]:
    s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
    q = (
      (m[0, 1] + m[1, 0]) / s,
      0.25 * s,
      (m[1, 2] + m[2, 1]) / s,
      (m[0, 2] - m[2, 0]) / s,
    )
  else:
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    q = (
      (m[0, 2] + m[2, 0]) / s,
      (m[1, 2] + m[2, 1]) / s,
      0.25 * s,
      (m[1, 0] - m[0, 1]) / s,
    )
  return normalize_quaternion(np.asarray(q))


def remap_matrix(remap: str) -> np.ndarray:
  axes = {"r": 0, "p": 1, "y": 2}
  if len(remap) != 3:
    raise ValueError(f"IMU remap must contain three axes, got {remap!r}")
  matrix = np.zeros((3, 3), dtype=np.float64)
  for row, key in enumerate(remap):
    axis = key.lower()
    if axis not in axes:
      raise ValueError(f"Invalid IMU remap axis: {key}")
    matrix[row, axes[axis]] = -1.0 if key.isupper() else 1.0
  if not np.isclose(abs(np.linalg.det(matrix)), 1.0):
    raise ValueError(f"Invalid IMU remap matrix for {remap!r}")
  return matrix


class ImuTransform:
  """Apply the same mount/remap convention used by imu_base_tf.py."""

  def __init__(
    self,
    mount_roll_deg: float = -90.0,
    mount_pitch_deg: float = 0.0,
    mount_yaw_deg: float = 0.0,
    rpy_remap: str = "YRp",
  ):
    self.mount_quaternion = rpy_to_quaternion(
      math.radians(mount_roll_deg),
      math.radians(mount_pitch_deg),
      math.radians(mount_yaw_deg),
    )
    self.mount_matrix = quaternion_to_matrix(self.mount_quaternion)
    self.remap = remap_matrix(rpy_remap)

  def orientation(self, raw_xyzw: np.ndarray) -> np.ndarray:
    mounted = normalize_quaternion(
      quaternion_multiply(self.mount_quaternion, normalize_quaternion(raw_xyzw))
    )
    mounted_matrix = quaternion_to_matrix(mounted)
    return matrix_to_quaternion(self.remap @ mounted_matrix @ self.remap.T)

  def angular_velocity(self, raw_rad_s: np.ndarray) -> np.ndarray:
    raw = np.asarray(raw_rad_s, dtype=np.float64).reshape(3)
    return (self.remap @ self.mount_matrix @ raw).astype(np.float32)

  def linear_acceleration(self, raw_m_s2: np.ndarray) -> np.ndarray:
    """Rotate OpenCR specific force into the policy body frame."""
    raw = np.asarray(raw_m_s2, dtype=np.float64).reshape(3)
    return (self.remap @ self.mount_matrix @ raw).astype(np.float32)

  @staticmethod
  def stationary_acceleration_bias(
    acceleration_samples: list[np.ndarray] | np.ndarray,
    projected_gravity_samples: list[np.ndarray] | np.ndarray,
  ) -> np.ndarray:
    """Estimate sensor DC bias while preserving the expected gravity signal."""
    acceleration_mean = np.mean(np.asarray(acceleration_samples), axis=0)
    gravity_mean = np.mean(np.asarray(projected_gravity_samples), axis=0)
    gravity_norm = max(float(np.linalg.norm(gravity_mean)), 1.0e-6)
    expected_specific_force = -9.80665 * gravity_mean / gravity_norm
    return (acceleration_mean - expected_specific_force).astype(np.float32)

  def projected_gravity(self, raw_xyzw: np.ndarray) -> np.ndarray:
    body_to_world = quaternion_to_matrix(self.orientation(raw_xyzw))
    gravity_world = np.asarray([0.0, 0.0, -1.0], dtype=np.float64)
    return (body_to_world.T @ gravity_world).astype(np.float32)
