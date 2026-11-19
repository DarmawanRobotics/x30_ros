"""Solve times of an optimising controller against its budget, and a warning while it is late."""

from __future__ import annotations

from collections import deque


class SolveTimes:
    """The durations of the last solves: the latest, the slowest, and whether too many overran."""

    def __init__(
        self, budget_sec: float, window: int = 50, late_share: float = 0.1, least: int = 10
    ):
        self.budget_sec = budget_sec
        self.late_share = late_share
        self.least = least
        self.seconds = deque(maxlen=window)

    def add(self, seconds: float):
        """Record one solve."""
        self.seconds.append(max(float(seconds), 0.0))

    @property
    def last_ms(self) -> float:
        """Return the latest solve (ms), 0 before the first."""
        return self.seconds[-1] * 1000.0 if self.seconds else 0.0

    @property
    def max_ms(self) -> float:
        """Return the slowest solve of the window (ms)."""
        return max(self.seconds, default=0.0) * 1000.0

    @property
    def late(self) -> bool:
        """Return True when more than late_share of at least `least` solves overran the budget."""
        if len(self.seconds) < self.least:
            return False
        over = sum(1 for seconds in self.seconds if seconds > self.budget_sec)
        return over > self.late_share * len(self.seconds)

    def warning(self, name: str, hint: str) -> str:
        """Return what to log while late ('' otherwise)."""
        if not self.late:
            return ''
        took = f'{name} solves take up to {self.max_ms:.0f} ms'
        return f'{took}, longer than {self.budget_sec * 1000.0:.0f} ms: {hint}'
