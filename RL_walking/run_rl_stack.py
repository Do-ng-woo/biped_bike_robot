#!/usr/bin/env python3
"""Supervise the package-local RL policy and its exclusive OpenCR bridge."""

from __future__ import annotations

import argparse
import signal
import subprocess
import sys
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parent
DEFAULT_POLICY = ROOT / "models" / "model_90000.npz"
DEFAULT_CONFIG = ROOT / "config.yaml"


def _terminate(process: subprocess.Popen, sig: int, timeout: float) -> None:
  if process.poll() is not None:
    return
  process.send_signal(sig)
  try:
    process.wait(timeout=timeout)
  except subprocess.TimeoutExpired:
    process.kill()
    process.wait(timeout=2.0)


def main() -> int:
  parser = argparse.ArgumentParser()
  parser.add_argument("--policy", type=Path, default=DEFAULT_POLICY)
  parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
  parser.add_argument("--forward-speed", type=float, default=0.08)
  parser.add_argument("--lateral-speed", type=float, default=0.04)
  parser.add_argument("--yaw-rate", type=float, default=0.25)
  parser.add_argument("--action-filter-alpha", type=float, default=1.0)
  parser.add_argument("--control-port", type=int, default=8081)
  args = parser.parse_args()

  if not args.policy.is_file():
    raise SystemExit(f"RL policy not found: {args.policy}")
  if not args.config.is_file():
    raise SystemExit(f"RL config not found: {args.config}")

  stopping = False

  def request_stop(_signum, _frame) -> None:
    nonlocal stopping
    stopping = True

  signal.signal(signal.SIGINT, request_stop)
  signal.signal(signal.SIGTERM, request_stop)

  bridge_command = [
    sys.executable,
    "-u",
    str(ROOT / "run_ros_bridge.py"),
    "--config",
    str(args.config),
  ]
  policy_command = [
    sys.executable,
    "-u",
    str(ROOT / "run_policy_ros.py"),
    "--policy",
    str(args.policy),
    "--config",
    str(args.config),
    "--forward-speed",
    str(args.forward_speed),
    "--lateral-speed",
    str(args.lateral_speed),
    "--yaw-rate",
    str(args.yaw_rate),
    "--action-filter-alpha",
    str(args.action_filter_alpha),
    "--control-ui",
    "--control-port",
    str(args.control_port),
    "--arm",
    "--duration",
    "0",
    "--log-dir",
    str(ROOT / "logs"),
  ]

  print("Starting exclusive OpenCR RL bridge", flush=True)
  bridge = subprocess.Popen(bridge_command)
  policy: subprocess.Popen | None = None
  exit_code = 0
  try:
    # The bridge performs the measured-pose -> READY transition before spinning.
    # The policy safely waits for both joint and IMU messages, so no fixed long
    # startup delay is needed here.
    time.sleep(0.5)
    if bridge.poll() is not None:
      raise RuntimeError(f"RL bridge exited early ({bridge.returncode})")
    print("Starting local NumPy walking policy", flush=True)
    policy = subprocess.Popen(policy_command)

    while not stopping:
      if bridge.poll() is not None:
        exit_code = bridge.returncode or 1
        print(f"RL bridge stopped unexpectedly ({exit_code})", flush=True)
        break
      if policy.poll() is not None:
        exit_code = policy.returncode or 1
        print(f"RL policy stopped ({policy.returncode})", flush=True)
        break
      time.sleep(0.1)
  except Exception as exc:
    exit_code = 1
    print(f"RL stack startup failed: {exc}", file=sys.stderr, flush=True)
  finally:
    # Stop policy first. Its final /biped_rl/enable=false makes the bridge blend
    # all 17 position motors back to HARDWARE_READY_TARGET over three seconds.
    if policy is not None:
      _terminate(policy, signal.SIGINT, 2.0)
    if bridge.poll() is None:
      print("Waiting for READY-pose return before releasing OpenCR", flush=True)
      time.sleep(3.3)
      _terminate(bridge, signal.SIGINT, 5.0)
  return exit_code


if __name__ == "__main__":
  raise SystemExit(main())
