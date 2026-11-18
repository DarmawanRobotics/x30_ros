"""DUNE's footprint, the record of a checkpoint and the checks before NeuPAN uses it."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_navigation.core.dune_model import (
    DuneRecord,
    check_checkpoint,
    footprint_vertices,
    polygon_distance,
    record_path,
)
from x30_utils.robot import Footprint

X30 = Footprint(front=0.5, rear=0.5, half_width=0.235)


def test_the_vertices_are_the_corners_counter_clockwise():
    vertices = footprint_vertices(X30)
    x, y = vertices
    assert vertices.shape == (2, 4)
    assert 0.5 * np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y) == pytest.approx(0.47)


def test_the_distance_to_the_footprint():
    points = [(0.0, 0.0), (1.5, 0.0), (0.0, -1.235), (0.8, 0.535), (-0.5, 0.235), (0.2, 0.1)]
    expected = [0.0, 1.0, 1.0, math.hypot(0.3, 0.3), 0.0, 0.0]
    assert polygon_distance(footprint_vertices(X30), np.array(points)) == pytest.approx(expected)


def test_a_checkpoint_needs_its_record_for_the_same_footprint(tmp_path):
    vertices = footprint_vertices(X30)
    checkpoint = tmp_path / 'dune.pth'
    assert 'no DUNE checkpoint set' in check_checkpoint('', vertices)[0]
    assert 'not found' in check_checkpoint(str(checkpoint), vertices)[0]
    checkpoint.write_bytes(b'weights')
    assert 'record' in check_checkpoint(str(checkpoint), vertices)[0]
    DuneRecord(vertices=vertices.tolist(), data_range=[-8.0, -8.0, 8.0, 8.0]).save(str(checkpoint))
    assert record_path(str(checkpoint)) == str(tmp_path / 'dune.json')
    assert check_checkpoint(str(checkpoint), vertices) == []
    nudged = vertices + 5e-4
    assert check_checkpoint(str(checkpoint), nudged) == []
    wider = footprint_vertices(Footprint(front=0.5, rear=0.5, half_width=0.3))
    assert 'another footprint' in check_checkpoint(str(checkpoint), wider)[0]


def test_the_record_round_trips_and_gives_the_trained_range(tmp_path):
    checkpoint = str(tmp_path / 'm.pth')
    record = DuneRecord([[1.0, -1.0, -1.0], [0.0, 1.0, -1.0]], [-6.0, -5.0, 6.0, 5.0], 10, 2)
    record.save(checkpoint)
    assert DuneRecord.load(checkpoint) == record
    assert record.range_m == 5.0
    (tmp_path / 'm.json').write_text('{"epochs": 3}')
    with pytest.raises(ValueError, match='no vertices'):
        DuneRecord.load(checkpoint)
