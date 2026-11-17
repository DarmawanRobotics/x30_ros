"""TripMeter: distance, moving time and speeds of the estimator since the last reset."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math

from x30_odometry.core.config import TripConfig


@dataclass(frozen=True)
class TripTotals:
    """What is kept across restarts."""

    distance: float = 0.0  # m
    moving_sec: float = 0.0
    max_speed: float = 0.0  # m/s
    since: float = 0.0  # wall clock of the last reset, s

    def to_dict(self) -> dict:
        """Return the totals as JSON-friendly values."""
        return {
            'distance_m': round(self.distance, 3),
            'moving_sec': round(self.moving_sec, 1),
            'max_speed_mps': round(self.max_speed, 3),
            'since': round(self.since, 3),
        }

    @classmethod
    def from_dict(cls, data) -> TripTotals | None:
        """Return the totals of to_dict(), or None for anything else."""
        if not isinstance(data, dict):
            return None
        try:
            values = [float(data[key]) for key in ('distance_m', 'moving_sec', 'max_speed_mps')]
            since = float(data['since'])
        except (KeyError, TypeError, ValueError):
            return None
        if not all(math.isfinite(v) and v >= 0.0 for v in [*values, since]):
            return None
        return cls(values[0], values[1], values[2], since)


class TripMeter:
    """Feed it the estimator poses; read distance, moving_sec, speed, average, max_speed."""

    def __init__(self, config: TripConfig, since: float = 0.0):
        self.config = config
        self.distance = 0.0
        self.moving_sec = 0.0
        self.max_speed = 0.0
        self.speed = 0.0
        self.since = since
        self.changed = False  # totals changed since the last totals() call
        self._anchor: tuple[float, float] | None = None
        self._anchor_t = 0.0
        self._last_t: float | None = None
        self._window: deque[tuple[float, float, float]] = deque()

    def configure(self, config: TripConfig):
        """Apply a new configuration (live parameter change)."""
        self.config = config

    @property
    def average(self) -> float:
        """Mean speed while moving, m/s (0 before the robot moved)."""
        return self.distance / self.moving_sec if self.moving_sec > 0.0 else 0.0

    def add(self, t: float, x: float, y: float):
        """Count the pose (x, y) at odometry time t (s)."""
        if not all(map(math.isfinite, (t, x, y))):
            return
        cfg = self.config
        last_t = self._last_t
        if last_t is not None:
            dt = t - last_t
            px, py = self._window[-1][1:]
            step = math.hypot(x - px, y - py)
            if dt <= 0.0 or dt > cfg.gap_sec or step > cfg.max_mps * max(dt, 0.05):
                last_t = None  # silence, a stamp going back or a jump: a new stretch
        if last_t is None:
            self._start(t, x, y)
            return
        self._last_t = t
        ax, ay = self._anchor
        moved = math.hypot(x - ax, y - ay)
        if moved >= cfg.step_m:
            self.distance += moved
            elapsed = t - self._anchor_t
            if cfg.moving_mps <= 0.0 or elapsed * cfg.moving_mps <= moved:
                self.moving_sec += elapsed  # covered at moving speed: moving all along
            self._anchor, self._anchor_t = (x, y), t
            self.changed = True
        window = self._window
        window.append((t, x, y))
        while len(window) > 2 and t - window[1][0] >= cfg.window_sec:
            window.popleft()
        t0, x0, y0 = window[0]
        span = t - t0
        # a shorter span is mostly pose noise: no speed until the window is nearly full
        speed = math.hypot(x - x0, y - y0) / span if span >= 0.8 * cfg.window_sec else 0.0
        self.speed = speed if speed >= cfg.moving_mps else 0.0
        if self.speed > self.max_speed:
            self.max_speed = self.speed
            self.changed = True

    def gap(self):
        """Count nothing up to the next pose (the estimator jumped, restarted or went silent)."""
        self._last_t = None
        self._anchor = None
        self._window.clear()
        self.speed = 0.0

    def reset(self, since: float):
        """Start again from zero at wall-clock time since; the current stretch goes on."""
        self.distance = self.moving_sec = self.max_speed = 0.0
        self.since = since
        if self._window:
            self._anchor, self._anchor_t = self._window[-1][1:], self._window[-1][0]
        self.changed = True

    def totals(self) -> TripTotals:
        """Return the totals to keep (and clear the changed flag)."""
        self.changed = False
        return TripTotals(self.distance, self.moving_sec, self.max_speed, self.since)

    def restore(self, totals: TripTotals):
        """Continue from kept totals."""
        self.distance = totals.distance
        self.moving_sec = totals.moving_sec
        self.max_speed = totals.max_speed
        self.since = totals.since
        self.changed = False

    def _start(self, t: float, x: float, y: float):
        self._last_t = t
        self._anchor, self._anchor_t = (x, y), t
        self._window.clear()
        self._window.append((t, x, y))
        self.speed = 0.0
