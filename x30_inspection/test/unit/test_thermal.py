"""Overheat criteria: absolute, ambient and similar components."""

from __future__ import annotations

import dataclasses

import pytest

from x30_inspection.core.config import ThermalConfig
from x30_inspection.core.plan import Region, Target, ThermalTarget
from x30_inspection.core.thermal import (
    Reading,
    Severity,
    absolute_limits,
    analyse,
    level,
    median_reading,
    regions_of,
    worst,
)

CONFIG = ThermalConfig()  # ambient 30, ambient 11/21/41, similar 2/4/16, absolute 80/90/105


def target(**thermal) -> Target:
    return Target('ds_clamps', 10.0, -5.0, thermal=ThermalTarget(**thermal))


def by_criterion(findings):
    return {f.criterion: f for f in findings}


def test_level_boundaries():
    assert level(0.9, 1, 2, 3) is Severity.NORMAL
    assert level(1.0, 1, 2, 3) is Severity.ATTENTION
    assert level(2.0, 1, 2, 3) is Severity.WARNING
    assert level(3.0, 1, 2, 3) is Severity.CRITICAL


def test_cool_single_frame_is_normal():
    found = by_criterion(analyse(target(), [Reading('frame', 38.0, 25.0)], CONFIG))
    assert set(found) == {'absolute', 'ambient'}  # one region: nothing to compare
    assert worst(found.values()) is Severity.NORMAL
    assert found['ambient'].delta == pytest.approx(8.0)


def test_rise_over_ambient():
    found = by_criterion(analyse(target(), [Reading('frame', 62.0, 25.0)], CONFIG))
    assert found['ambient'].severity is Severity.WARNING  # +32 degC
    assert found['absolute'].severity is Severity.NORMAL
    hot = by_criterion(analyse(target(), [Reading('frame', 75.0, 25.0)], CONFIG))
    assert hot['ambient'].severity is Severity.CRITICAL  # +45 degC


def test_one_hot_phase_against_the_others():
    readings = [Reading('R', 41.0, 30.0), Reading('S', 58.5, 30.0, 300, 140),
                Reading('T', 40.0, 30.0)]  # fmt: skip
    found = by_criterion(analyse(target(strips=3), readings, CONFIG))
    similar = found['similar']
    assert similar.label == 'S'
    assert similar.reference == pytest.approx(40.5)
    assert similar.delta == pytest.approx(18.0)
    assert similar.severity is Severity.CRITICAL
    assert '(300, 140)' in found['absolute'].message


def test_balanced_phases_are_normal():
    readings = [Reading('R', 35.0, 30.0), Reading('S', 36.0, 30.0), Reading('T', 35.5, 30.0)]
    assert worst(analyse(target(strips=3), readings, CONFIG)) is Severity.NORMAL


def test_target_limit_replaces_critical():
    assert absolute_limits(CONFIG, 0.0) == (80.0, 90.0, 105.0)
    assert absolute_limits(CONFIG, 70.0) == (70.0, 70.0, 70.0)
    found = by_criterion(analyse(target(max_temp_c=70.0), [Reading('frame', 71.0, 20.0)], CONFIG))
    assert found['absolute'].severity is Severity.CRITICAL


def test_ambient_is_live_configuration():
    reading = [Reading('frame', 55.0, 25.0)]
    assert by_criterion(analyse(target(), reading, CONFIG))['ambient'].severity is (
        Severity.WARNING  # +25 degC over 30
    )
    hot_day = dataclasses.replace(CONFIG, ambient_c=38.0)
    found = by_criterion(analyse(target(), reading, hot_day))
    assert found['ambient'].severity is Severity.ATTENTION  # +17 degC over 38
    assert found['ambient'].delta == pytest.approx(17.0)


def test_regions_explicit_strips_or_frame():
    boxes = (Region('a', (0.0, 0.0, 0.5, 0.5)),)
    assert regions_of(target(regions=boxes)) == boxes
    strips = regions_of(target(strips=3))
    assert [r.box for r in strips] == [
        (0.0, 0.0, 1 / 3, 1.0), (1 / 3, 0.0, 2 / 3, 1.0), (2 / 3, 0.0, 1.0, 1.0)
    ]  # fmt: skip
    assert [r.name for r in strips] == ['strip1', 'strip2', 'strip3']
    assert regions_of(target()) == () and regions_of(target(strips=1)) == ()


def test_median_of_samples_drops_an_outlier():
    samples = [Reading('R', 40.0, 20.0), Reading('R', 95.0, 21.0), Reading('R', 41.0, 22.0)]
    assert median_reading(samples).temp_max == 41.0
    assert median_reading(samples).temp_min == 21.0


def test_no_readings_no_findings():
    assert analyse(target(), [], CONFIG) == []
    assert worst([]) is Severity.NORMAL
