import math

import pytest

from common.dynamixel import position_tick_to_rad, signed_u32


def test_signed_u32_conversion():
    assert signed_u32(0) == 0
    assert signed_u32(0x7FFFFFFF) == 0x7FFFFFFF
    assert signed_u32(0xFFFFFFFF) == -1


def test_xl330_centered_position_conversion():
    assert position_tick_to_rad(2048) == 0.0
    assert position_tick_to_rad(0) == pytest.approx(-math.pi)
    assert position_tick_to_rad(4096) == pytest.approx(math.pi)
