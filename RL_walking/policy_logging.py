"""CSV and overlay-plot logging for real-robot policy runs."""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path

import numpy as np

try:
  from .contract import TARGET_JOINTS
except ImportError:  # Direct script execution from this folder.
  from contract import TARGET_JOINTS


class PolicyRunLogger:
  def __init__(self, log_dir: Path, extra_header: tuple[str, ...] = ()):
    log_dir.mkdir(parents=True, exist_ok=True)
    self.stem = log_dir / datetime.now().strftime("policy_%Y%m%d_%H%M%S")
    self.csv_path = self.stem.with_suffix(".csv")
    self.stream = self.csv_path.open("w", newline="", encoding="utf-8")
    self.writer = csv.writer(self.stream)
    self.extra_header = extra_header
    header = ["time_sec", "phase", "tilt_deg", "obs_zmax", *extra_header]
    for name in TARGET_JOINTS:
      header.extend(
        (
          f"q_target/{name}",
          f"q_actual/{name}",
          f"qd_actual/{name}",
          f"qdd_est/{name}",
          f"torque_est/{name}",
        )
      )
    self.writer.writerow(header)
    self.closed = False
    self.plot_error: str | None = None

  def record(
    self,
    elapsed: float,
    phase: float,
    tilt_deg: float,
    obs_zmax: float,
    target: np.ndarray,
    actual: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    torque: np.ndarray,
    extra_values: np.ndarray | None = None,
  ) -> None:
    row: list[float] = [elapsed, phase, tilt_deg, obs_zmax]
    if self.extra_header:
      if extra_values is None or len(extra_values) != len(self.extra_header):
        raise ValueError("Policy log extra values do not match extra header")
      row.extend(float(value) for value in extra_values)
    for index in range(len(TARGET_JOINTS)):
      row.extend(
        (
          float(target[index]),
          float(actual[index]),
          float(velocity[index]),
          float(acceleration[index]),
          float(torque[index]),
        )
      )
    self.writer.writerow(row)

  def close(self) -> tuple[Path, Path | None]:
    if self.closed:
      return self.csv_path, None
    self.closed = True
    self.stream.flush()
    self.stream.close()
    # Keep the Raspberry Pi runtime lightweight: CSV is always written and no
    # matplotlib subprocess is required. Plot the CSV later on the training PC.
    return self.csv_path, None
