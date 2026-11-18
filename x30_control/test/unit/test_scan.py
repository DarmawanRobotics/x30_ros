from __future__ import annotations

import math

import numpy as np
import pytest

from x30_control.core.config import FieldConfig
from x30_control.core.scan import ScanData, ScanMonitor, ScanState

CFG = FieldConfig()
N = 72
STEP = 2.0 * math.pi / N


def scan(ranges=None, frame='base_link', stamp=100.0, angle_min=-math.pi, step=STEP, rmax=4.0):
    values = np.full(N, 4.0) if ranges is None else np.asarray(ranges, dtype=float)
    return ScanData(frame, stamp, angle_min, step, rmax, values)


def fresh(monitor: ScanMonitor, data: ScanData, t: float = 100.0):
    assert monitor.update(data, received=t) == ''
    return monitor.view(t, t)


def test_nothing_received_is_stale():
    view = ScanMonitor('base_link', CFG).view(0.0, 0.0)
    assert view.state is ScanState.STALE and 'no safety scan' in view.reason


def test_valid_full_circle_scan_is_ok_with_points():
    ranges = np.full(N, 4.0)
    ranges[36] = 1.0  # straight ahead
    view = fresh(ScanMonitor('base_link', CFG), scan(ranges))
    assert view.state is ScanState.OK and view.points.shape == (1, 2)
    assert view.points[0] == pytest.approx([1.0 * math.cos(STEP / 2), 1.0 * math.sin(STEP / 2)])


def test_wrong_frame_is_rejected():
    """Finding #7: a scan in another frame must not be read as base_link points."""
    monitor = ScanMonitor('base_link', CFG)
    problem = monitor.update(scan(frame='livox_frame'), received=100.0)
    assert 'livox_frame' in problem
    assert monitor.view(100.0, 100.0).state is ScanState.STALE


def test_partial_coverage_is_rejected():
    """Finding #7: a front-only scan cannot protect the sides and the rear."""
    monitor = ScanMonitor('base_link', CFG)
    half = ScanData('base_link', 100.0, -math.pi / 2, STEP, 4.0, np.full(N // 2, 4.0))
    assert 'full circle' in monitor.update(half, received=100.0)
    assert monitor.view(100.0, 100.0).state is ScanState.STALE


@pytest.mark.parametrize(
    'bad',
    [
        {'step': float('nan')},
        {'step': 0.0},
        {'rmax': float('nan')},
        {'rmax': 0.0},
        {'angle_min': float('inf')},
        {'stamp': float('nan')},
        {'ranges': []},
    ],
)
def test_malformed_scans_are_rejected(bad):
    monitor = ScanMonitor('base_link', CFG)
    assert monitor.update(scan(**bad), received=100.0) != ''
    assert monitor.view(100.0, 100.0).state is ScanState.STALE


def test_an_invalid_scan_invalidates_the_previous_one_at_once():
    monitor = ScanMonitor('base_link', CFG)
    fresh(monitor, scan())
    monitor.update(scan(frame='odom'), received=100.05)
    assert monitor.view(100.05, 100.05).state is ScanState.STALE
    fresh(monitor, scan(stamp=100.1), t=100.1)  # a valid scan recovers


def test_receive_age_on_monotonic_time():
    monitor = ScanMonitor('base_link', CFG)
    fresh(monitor, scan())
    assert monitor.view(100.0 + CFG.scan_timeout_sec - 0.01, 100.0).state is ScanState.OK
    view = monitor.view(100.0 + CFG.scan_timeout_sec + 0.01, 100.0)
    assert view.state is ScanState.STALE and 'no scan' in view.reason


def test_header_age_old_and_future():
    """Finding #7: freshness by stamp too, with a tolerance for small clock offsets."""
    monitor = ScanMonitor('base_link', CFG)
    monitor.update(scan(stamp=99.0), received=100.0)
    view = monitor.view(100.0, 100.0)
    assert view.state is ScanState.STALE and 'old' in view.reason
    monitor.update(scan(stamp=100.0 + CFG.scan_clock_tolerance_sec / 2), received=100.0)
    assert monitor.view(100.0, 100.0).state is ScanState.OK
    monitor.update(scan(stamp=100.5), received=100.0)
    view = monitor.view(100.0, 100.0)
    assert view.state is ScanState.STALE and 'future' in view.reason


def test_header_age_grows_while_waiting():
    monitor = ScanMonitor('base_link', CFG)
    fresh(monitor, scan())
    assert monitor.view(100.1, 100.0 + CFG.scan_max_age_sec + 0.01).state is ScanState.STALE


@pytest.mark.parametrize('blocked', [0.0, -1.0, float('nan'), float('inf')])
def test_blocked_or_unknown_sectors_are_blind(blocked):
    """Finding #7: NaN / 0 sectors are not free space."""
    ranges = np.full(N, 4.0)
    ranges[5] = blocked
    ranges[40] = 2.0
    view = fresh(ScanMonitor('base_link', CFG), scan(ranges))
    assert view.state is ScanState.BLIND and view.blocked == 1
    assert len(view.points) == 1  # the seen obstacle is kept


def test_all_sectors_zero_is_blind_not_clear():
    """Finding B1: a blind LiDAR reporting 0 everywhere must never read as clear."""
    view = fresh(ScanMonitor('base_link', CFG), scan(np.zeros(N)))
    assert view.state is ScanState.BLIND and view.blocked == N


def test_clockwise_scan_is_accepted():
    view = fresh(ScanMonitor('base_link', CFG), scan(angle_min=math.pi, step=-STEP))
    assert view.state is ScanState.OK
