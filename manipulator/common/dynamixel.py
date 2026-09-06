from __future__ import annotations

import math


def signed_u32(value: int) -> int:
    value = int(value) & 0xFFFFFFFF
    return value - 0x100000000 if value & 0x80000000 else value


def position_tick_to_rad(
    raw_tick: int,
    center_tick: int = 2048,
    ticks_per_revolution: int = 4096,
) -> float:
    if ticks_per_revolution <= 0:
        raise ValueError("ticks_per_revolution must be positive")
    return (signed_u32(raw_tick) - center_tick) * (
        2.0 * math.pi / ticks_per_revolution
    )
