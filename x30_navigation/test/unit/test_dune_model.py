"""DUNE's footprint, the record of a checkpoint and the checks before NeuPAN uses it."""

from __future__ import annotations

import math
from pathlib import Path

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
MODEL = Path(__file__).resolve().parents[2] / 'models' / 'dune_x30.pth'


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


def test_the_x30_model_was_trained_well_for_the_x30():
    record = DuneRecord.load(str(MODEL))
    assert check_checkpoint(str(MODEL), footprint_vertices(X30)) == []
    assert record.range_m >= 6.0
    assert record.mean_error_m < 0.005 and record.max_error_m < 0.1
    assert 0.0 <= record.max_over_m < 0.02  # a point is never seen 2 cm further than it is


def test_the_x30_model_measures_the_distance():
    pytest.importorskip('torch')
    pytest.importorskip('neupan')
    from x30_navigation.core.dune_model import load_model, measure, model_distance

    vertices = footprint_vertices(X30)
    model = load_model(str(MODEL), 4)
    error = measure(model, vertices, 6.0, seed=7)
    assert error['mean_error_m'] < 0.005 and error['max_over_m'] < 0.02
    near = np.array([(0.8, 0.0), (0.0, 0.535), (-0.7, 0.0), (2.5, 1.0)])
    assert model_distance(model, vertices, near) == pytest.approx(
        polygon_distance(vertices, near), abs=0.01
    )
