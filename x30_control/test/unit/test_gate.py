from __future__ import annotations

import math

import numpy as np
import pytest

from x30_control.core.command import ZERO, Command
from x30_control.core.config import FieldConfig, MuxConfig
from x30_control.core.field import FieldState
from x30_control.core.gate import Gate
from x30_control.core.scan import ScanData
from x30_utils.config import build
from x30_utils.robot import RobotProfile

N = 72
ROBOT = build(
    RobotProfile,
    {'robot.limits.max_vx': 1.0, 'robot.limits.max_vy': 0.5, 'robot.limits.max_wz': 1.5},
    'robot.',
)


def gate(**mux) -> Gate:
    g = Gate(ROBOT, MuxConfig(**mux), FieldConfig())
    return g


def scan(ranges=None) -> ScanData:
    r = np.full(N, 4.0) if ranges is None else np.asarray(ranges, dtype=float)
    return ScanData('base_link', 10.0, -math.pi, 2 * math.pi / N, 4.0, r)


def test_clamped_before_the_field():
    """Kept from protective_field: the envelope scales first, the field sees the clamped one."""
    g = gate()
    g.update_scan(scan(), received=10.0)
    out = g.apply(Command(2.0, 1.0, 0.0), remote=False, received_now=10.0, now=10.0)
    assert out.command == Command(1.0, 0.5, 0.0) and out.clipped
    assert out.field is FieldState.CLEAR and out.speed_scale == pytest.approx(0.5)


def test_no_scan_auto_is_zero():
    """Kept from protective_field: AUTO with no scan does not move."""
    out = gate().apply(Command(0.3, 0.0, 0.0), remote=False, received_now=0.0, now=0.0)
    assert out.command == ZERO and out.field is FieldState.STALE


def test_non_finite_is_zero_even_with_a_clear_scan():
    g = gate()
    g.update_scan(scan(), received=10.0)
    out = g.apply(Command(float('nan'), 0.0, 0.0), remote=True, received_now=10.0, now=10.0)
    assert out.command == ZERO and not out.finite


def test_health_scale_applies_to_every_axis():
    g = gate(use_health_limit=True)
    g.update_scan(scan(), received=10.0)
    g.health.update(0.5, now=10.0)
    out = g.apply(Command(0.4, 0.2, 1.0), remote=False, received_now=10.0, now=10.0)
    assert out.command == Command(0.2, 0.1, 0.5) and out.health_scale == 0.5
    stale = g.apply(Command(0.4, 0.2, 1.0), remote=False, received_now=20.0, now=10.0)
    assert stale.health_scale == 1.0  # stale health: no change


def test_blind_scan_reported_when_idle():
    g = gate()
    g.update_scan(scan(np.zeros(N)), received=10.0)
    assert g.idle_state(False, 10.0, 10.0).state is FieldState.BLIND
    g.update_scan(scan(), received=10.1)
    assert g.idle_state(False, 10.1, 10.1).state is FieldState.CLEAR
