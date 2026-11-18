from __future__ import annotations

import math

import numpy as np
import pytest

from x30_utils.robot import Footprint
from x30_utils.safety import (
    rotation_room,
    stopping_distance,
    stopping_speed,
    translation_clearance,
)

FOOT = Footprint()


def test_stopping_model_is_consistent():
    for v in (0.1, 0.35, 1.0):
        d = stopping_distance(v, 0.6, 0.25)
        assert stopping_speed(d, 0.6, 0.25) == pytest.approx(v)
    assert stopping_speed(0.0, 0.6, 0.25) == 0.0
    assert stopping_speed(-1.0, 0.6, 0.25) == 0.0
    assert math.isinf(stopping_speed(math.inf, 0.6, 0.25))


def test_translation_clearance_ahead_and_beside():
    assert translation_clearance(FOOT, 1.0, 0.0, np.array([[1.4, 0.0]])) == pytest.approx(1.0)
    assert math.isinf(translation_clearance(FOOT, 1.0, 0.0, np.array([[1.4, 1.0]])))
    assert translation_clearance(FOOT, 1.0, 0.0, np.array([[0.2, 0.2]])) == 0.0


def test_point_at_origin_blocks_every_direction():
    origin = np.zeros((72, 2))
    for ux, uy in ((1, 0), (0, 1), (-1, 0)):
        assert translation_clearance(FOOT, ux, uy, origin) == 0.0
    assert rotation_room(FOOT, origin, 1.0) == 0.0


def test_rotation_room_sees_the_swept_ring():
    corner_side = np.array([[0.0, 0.45]])
    assert rotation_room(FOOT, corner_side, 1.0) < math.radians(60)
    assert math.isinf(rotation_room(FOOT, np.array([[2.0, 0.0]]), 1.0))
    assert math.isinf(rotation_room(FOOT, np.empty((0, 2)), -1.0))


def test_sector_points_blocked_and_free():
    from x30_utils.safety import sector_points

    ranges = [4.0] * 72
    ranges[0], ranges[10], ranges[20] = 1.0, 0.0, float('nan')
    sp = sector_points(-math.pi, 2 * math.pi / 72, ranges, 4.0)
    assert sp.full_circle
    assert len(sp.points) == 1
    assert sp.blocked.sum() == 2
    assert len(sp.with_blocked()) == 3
    half = sector_points(-math.pi / 2, math.pi / 36, [4.0] * 36, 4.0)
    assert not half.full_circle
    assert not half.any_blocked
