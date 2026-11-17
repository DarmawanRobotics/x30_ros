"""Validated safety scan snapshots: frame, full-circle coverage, receive and stamp age, blocked."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math

import numpy as np

from x30_control.core.config import FieldConfig
from x30_utils.safety import sector_points

NO_POINTS = np.empty((0, 2))
NO_ANGLES = np.empty(0)


class ScanState(Enum):
    """What the newest safety scan can be used for."""

    OK = 'ok'  # fresh, in the base frame, full circle, every sector seen
    STALE = 'stale'  # none, invalid or too old: nothing is known about the surroundings
    BLIND = 'blind'  # fresh, but some sectors are blocked or unknown


@dataclass(frozen=True)
class ScanData:
    """A SectorScan as plain values (stamp in seconds of the node clock)."""

    frame_id: str
    stamp_sec: float
    angle_min: float
    angle_increment: float
    range_max: float
    ranges: np.ndarray


@dataclass(frozen=True)
class ScanView:
    """The usable part of the newest scan: obstacle points of the seen sectors (robot frame)."""

    state: ScanState
    points: np.ndarray
    blocked: int = 0
    reason: str = ''
    blocked_angles: np.ndarray = field(default_factory=lambda: NO_ANGLES)
    sector_width: float = 0.0


class ScanMonitor:
    """Validate safety scans and report the newest one as a fail-safe view."""

    def __init__(self, base_frame: str, config: FieldConfig):
        self.base_frame = base_frame
        self.config = config
        self._points = NO_POINTS
        self._blocked = 0
        self._blocked_angles = NO_ANGLES
        self._width = 0.0
        self._received: float | None = None
        self._stamp = 0.0
        self._problem = 'no safety scan received yet'

    def update(self, scan: ScanData, received: float) -> str:
        """Store a scan received at monotonic time `received`; return its problem or ''."""
        problem = self._problem_of(scan)
        if problem:
            self._problem = problem
            return problem
        sectors = sector_points(scan.angle_min, scan.angle_increment, scan.ranges, scan.range_max)
        if not sectors.full_circle:
            self._problem = f'covers {_coverage_deg(scan):.0f} deg, a full circle is required'
            return self._problem
        self._points = sectors.points
        self._blocked = int(sectors.blocked.sum())
        index = np.flatnonzero(sectors.blocked)
        self._blocked_angles = scan.angle_min + (index + 0.5) * scan.angle_increment
        self._width = abs(scan.angle_increment)
        self._received = received
        self._stamp = scan.stamp_sec
        self._problem = ''
        return ''

    def view(self, received_now: float, now: float) -> ScanView:
        """Return the scan state at monotonic received_now and node-clock now."""
        if self._problem:
            return ScanView(ScanState.STALE, NO_POINTS, reason=self._problem)
        silent = received_now - self._received
        if silent > self.config.scan_timeout_sec:
            return ScanView(ScanState.STALE, NO_POINTS, reason=f'no scan for {silent:.2f} s')
        age = now - self._stamp
        if age > self.config.scan_max_age_sec:
            return ScanView(ScanState.STALE, NO_POINTS, reason=f'scan stamp {age:.2f} s old')
        if age < -self.config.scan_clock_tolerance_sec:
            reason = f'scan stamp {-age:.2f} s in the future (clock offset?)'
            return ScanView(ScanState.STALE, NO_POINTS, reason=reason)
        if self._blocked:
            reason = f'{self._blocked} blocked or unknown sectors'
            return ScanView(
                ScanState.BLIND,
                self._points,
                self._blocked,
                reason,
                self._blocked_angles,
                self._width,
            )
        return ScanView(ScanState.OK, self._points)

    def _problem_of(self, scan: ScanData) -> str:
        if scan.frame_id != self.base_frame:
            return f'frame {scan.frame_id!r} is not the base frame {self.base_frame!r}'
        values = (scan.stamp_sec, scan.angle_min, scan.angle_increment, scan.range_max)
        if not all(math.isfinite(v) for v in values):
            return 'non-finite stamp, angles or range_max'
        if scan.angle_increment == 0.0 or scan.range_max <= 0.0:
            return 'angle_increment must not be 0 and range_max must be positive'
        if len(scan.ranges) == 0:
            return 'no sectors'
        return ''


def _coverage_deg(scan: ScanData) -> float:
    return math.degrees(abs(scan.angle_increment) * len(scan.ranges))
