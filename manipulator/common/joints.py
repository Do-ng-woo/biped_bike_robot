from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Mapping, Sequence


@dataclass(frozen=True)
class JointMapping:
    leader_names: tuple[str, ...]
    follower_names: tuple[str, ...]
    direction: tuple[float, ...]
    scale: tuple[float, ...]
    offset: tuple[float, ...]
    minimum: tuple[float, ...]
    maximum: tuple[float, ...]

    @classmethod
    def from_config(cls, arm: Mapping) -> "JointMapping":
        leader_names = tuple(arm["leader_joint_names"])
        result = cls(
            leader_names,
            tuple(arm["follower_joint_names"]),
            tuple(float(v) for v in arm["direction"]),
            tuple(float(v) for v in arm.get("scale", [1.0] * len(leader_names))),
            tuple(float(v) for v in arm["offset"]),
            tuple(float(v) for v in arm["min_position"]),
            tuple(float(v) for v in arm["max_position"]),
        )
        sizes = {len(field) for field in (
            result.leader_names, result.follower_names, result.direction, result.scale,
            result.offset, result.minimum, result.maximum,
        )}
        if sizes != {len(result.leader_names)} or not result.leader_names:
            raise ValueError("All arm mapping arrays must have the same non-zero length")
        return result

    def extract(self, names: Sequence[str], positions: Sequence[float]) -> list[float] | None:
        if len(names) != len(positions):
            return None
        values = dict(zip(names, positions))
        if any(name not in values for name in self.leader_names):
            return None
        return [float(values[name]) for name in self.leader_names]

    def map(self, leader: Sequence[float], runtime_offset: Sequence[float] | None = None) -> list[float]:
        if len(leader) != len(self.leader_names):
            raise ValueError("Unexpected leader vector size")
        offsets = self.offset if runtime_offset is None else runtime_offset
        return [
            min(max(self.direction[i] * self.scale[i] * float(value) + float(offsets[i]), self.minimum[i]), self.maximum[i])
            for i, value in enumerate(leader)
        ]

    def alignment_offsets(self, leader: Sequence[float], follower: Sequence[float]) -> list[float]:
        if len(leader) != len(self.leader_names) or len(follower) != len(self.follower_names):
            raise ValueError("Unexpected vector size for alignment")
        return [
            float(follower[i])
            - self.direction[i] * self.scale[i] * float(leader[i])
            for i in range(len(leader))
        ]


def ordered_positions(names: Sequence[str], positions: Sequence[float], desired: Iterable[str]) -> list[float] | None:
    if len(names) != len(positions):
        return None
    values = dict(zip(names, positions))
    desired_names = list(desired)
    if any(name not in values for name in desired_names):
        return None
    return [float(values[name]) for name in desired_names]
