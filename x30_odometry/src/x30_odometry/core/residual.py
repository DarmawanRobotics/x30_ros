"""Speed residual of the estimator against the robot odometry: chi^2 with a one-sided CUSUM."""

from __future__ import annotations

from collections import deque
import math

from x30_utils.math import Cusum


class SpeedResidual:
    """Planar speed of the estimator from its poses, compared with the leg odometry speed."""

    def __init__(self, config):
        self.config = config
        self.poses: deque = deque()
        self.leg: tuple[float, float] | None = None  # (receipt time, speed)
        self.cusum = Cusum(
            config.cusum_drift, config.cusum_threshold, cap=2.0 * config.cusum_threshold
        )
        self.speed = 0.0
        self.chi2 = 0.0
        self.alarm = False

    def configure(self, config):
        """Apply new thresholds, keeping the history."""
        self.config = config
        self.cusum.drift, self.cusum.threshold = config.cusum_drift, config.cusum_threshold
        self.cusum.cap = 2.0 * config.cusum_threshold  # bounded recovery

    def reset(self):
        """Forget everything (the estimator restarted)."""
        self.poses.clear()
        self.cusum.reset()
        self.speed, self.chi2, self.alarm = 0.0, 0.0, False

    def add_leg(self, now: float, vx: float, vy: float):
        """Keep the latest planar speed of the robot odometry (base frame twist)."""
        if math.isfinite(vx) and math.isfinite(vy):
            self.leg = (now, math.hypot(vx, vy))

    def add_pose(self, now: float, stamp: float, x: float, y: float) -> bool:
        """Add an estimator position; return True while the speeds disagree."""
        self.poses.append((stamp, x, y))
        while len(self.poses) > 2 and stamp - self.poses[1][0] >= self.config.window_sec:
            self.poses.popleft()
        t0, x0, y0 = self.poses[0]
        if stamp - t0 <= 0.0:
            return self.alarm
        self.speed = math.hypot(x - x0, y - y0) / (stamp - t0)
        if self.leg is None or now - self.leg[0] > self.config.leg_timeout_sec:
            self.cusum.reset()  # nothing to compare with: no evidence either way
            self.chi2, self.alarm = 0.0, False
            return False
        c = self.config
        residual = self.speed - self.leg[1]
        self.chi2 = residual * residual / (c.sigma_lio_mps**2 + c.sigma_leg_mps**2)
        self.alarm = self.cusum.update(self.chi2)
        return self.alarm

    @property
    def leg_speed(self) -> float:
        """The latest robot odometry speed (m/s), NaN when none."""
        return self.leg[1] if self.leg is not None else math.nan
