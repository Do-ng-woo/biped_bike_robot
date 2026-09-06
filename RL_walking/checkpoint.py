"""Resolve the package-local NumPy policy bundle."""

from __future__ import annotations

from pathlib import Path


def resolve_policy_path(policy_path: Path) -> Path:
  source = policy_path.expanduser().resolve()
  if not source.is_file():
    raise FileNotFoundError(f"Policy file does not exist: {source}")
  if source.suffix != ".npz":
    raise ValueError(
      "The Raspberry Pi runtime accepts an exported .npz policy. "
      "Export .pt checkpoints on the training PC first."
    )
  return source
