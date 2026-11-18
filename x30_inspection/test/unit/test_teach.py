"""inspection/add_target requests from the camera's view (no ROS)."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from x30_inspection.core.plan import PlanError
from x30_inspection.utils.teach import target_of


def request(**fields):
    base = {'name': 'ds_clamps', 'point': '', 'thermal': True, 'acoustic': False,
            'strips': 3, 'max_temp_c': 0.0}  # fmt: skip
    return SimpleNamespace(**{**base, **fields})


def test_target_takes_the_cameras_view():
    target = target_of(request(), 234.961, -12.04, 4.02)
    assert (target.pan_deg, target.tilt_deg, target.zoom) == (234.96, -12.04, 4.0)
    assert target.thermal.strips == 3 and not target.acoustic.enabled


def test_view_is_clamped_to_the_plan():
    target = target_of(request(), 0.0, -120.0, 0.5)
    assert (target.tilt_deg, target.zoom) == (-90.0, 1.0)
    assert target_of(request(), 400.0, 95.0, 99.0).pan_deg == 360.0


@pytest.mark.parametrize(
    ('fields', 'error'),
    [
        ({'name': 'DS 1'}, 'name'),
        ({'thermal': False, 'acoustic': False}, 'thermal, acoustic'),
        ({'max_temp_c': -5.0}, 'max_temp_c'),
    ],
)
def test_bad_requests(fields, error):
    with pytest.raises(PlanError, match=error):
        target_of(request(**fields), 0.0, 0.0, 1.0)
