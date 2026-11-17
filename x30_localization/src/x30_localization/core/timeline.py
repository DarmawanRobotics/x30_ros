"""Stamps of map->odom on /tf: a correction is the old value at t - epsilon, the new at t."""

from __future__ import annotations

from x30_utils.math import Pose2D

EPSILON_SEC = 1e-3


class MapOdomTimeline:
    """The samples of map->odom to broadcast, strictly increasing in time."""

    def __init__(self, epsilon: float = EPSILON_SEC):
        self.epsilon = epsilon
        self.current: Pose2D | None = None
        self.last_stamp = -1.0

    def tick(self, now: float) -> list[tuple[Pose2D, float]]:
        """Return the periodic sample at now (none while unknown or not newer)."""
        if self.current is None or now <= self.last_stamp:
            return []
        self.last_stamp = now
        return [(self.current, now)]

    def change(self, pose: Pose2D | None, now: float) -> list[tuple[Pose2D, float]]:
        """Switch to pose at now; return the samples to send (old at now - eps, new at now)."""
        previous, self.current = self.current, pose
        if pose is None:
            return []
        now = max(now, self.last_stamp + 2.0 * self.epsilon)
        out = []
        if previous is not None and now - self.epsilon > self.last_stamp:
            out.append((previous, now - self.epsilon))
        out.append((pose, now))
        self.last_stamp = now
        return out
