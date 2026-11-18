"""Stop points along the route: when to approach the next one and when it was missed."""

from __future__ import annotations

from enum import Enum
import math

import numpy as np

from x30_navigation.core.config.follower import ApproachConfig
from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D


class StopCheck(Enum):
    """What the next stop point asks for this tick."""

    NONE = 0
    APPROACH = 1
    SKIP = 2


class StopPoints:
    """The stop points still ahead on one route."""

    def __init__(self, route: Route, config: ApproachConfig):
        self.route = route
        self.config = config
        self.indices = [int(i) for i in route.stops]
        self.position = 0

    @property
    def next(self) -> int | None:
        """Waypoint index of the next stop point, or None."""
        return self.indices[self.position] if self.position < len(self.indices) else None

    def reset(self, s: float):
        """Make the first stop at or after arc length s the next one."""
        stops = np.asarray(self.indices, dtype=int)
        ahead = np.flatnonzero(self.route.s[stops] >= s - 1e-6) if stops.size else stops
        self.position = int(ahead[0]) if ahead.size else len(self.indices)

    def advance(self):
        """Move on to the following stop point."""
        self.position = min(self.position + 1, len(self.indices))

    def ahead(self, s: float) -> float:
        """Return the route distance from arc length s to the next stop (inf: none)."""
        i = self.next
        return math.inf if i is None else float(self.route.s[i]) - s

    def check(self, s: float, pose: Pose2D, lateral: float) -> StopCheck:
        """Return whether to approach the next stop point, skip it, or go on."""
        i = self.next
        if i is None:
            return StopCheck.NONE
        along = float(self.route.s[i]) - s
        if along < -self.config.skip_margin:
            return StopCheck.SKIP
        x, y = self.route.xy[i]
        distance = math.hypot(x - pose.x, y - pose.y) - abs(lateral)
        if along <= self.config.radius and distance <= self.config.radius:
            return StopCheck.APPROACH
        return StopCheck.NONE

    def colors(self) -> list[tuple[int, str]]:
        """Return (index, 'done' | 'next' | 'later') of every stop point, for markers."""
        out = []
        for n, i in enumerate(self.indices):
            out.append(
                (i, 'done' if n < self.position else 'next' if n == self.position else 'later')
            )
        return out


class Inspection:
    """The inspections of the current stop point (Trigger hooks the node calls asynchronously)."""

    def __init__(self):
        self.round = 0
        self.pending = 0
        self.results: list[str] = []

    def begin(self) -> int:
        """Start a new round and return its id."""
        self.round += 1
        self.pending = 0
        self.results = []
        return self.round

    def started(self, round_id: int, count: int, unavailable: list[str]) -> bool:
        """Record the calls made for a round; True when that round is already complete."""
        if round_id != self.round:
            return False
        self.pending = count
        self.results += [f'{name}: not available' for name in unavailable]
        return count == 0

    def answer(self, round_id: int, name: str, ok: bool, message: str) -> bool:
        """Record one answer; True when it completed the round."""
        if round_id != self.round or self.pending <= 0:
            return False
        self.results.append(f'{name}: {"ok" if ok else "fail"} {message}'.strip())
        self.pending -= 1
        return self.pending == 0

    def cancel(self):
        """Drop the current round (late answers are ignored)."""
        self.round += 1
        self.pending = 0

    def summary(self) -> str:
        """Return the results as one line."""
        return ' | '.join(self.results)
