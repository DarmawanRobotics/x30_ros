"""The 360 deg safety scan: stale or blind (every sector BLOCKED or NaN) stops the robot."""

from __future__ import annotations

import numpy as np

from x30_health.core.checks.base import Check, Issue, Level, fmt

BLOCKED = 0.0  # SectorScan.BLOCKED


def blocked_count(ranges) -> tuple[int, int]:
    """Return (blocked or unknown sectors, sectors): ranges <= BLOCKED or NaN."""
    r = np.asarray(ranges, dtype=float).ravel()
    with np.errstate(invalid='ignore'):  # NaN comparisons warn on numpy < 1.15
        return int(np.count_nonzero(~(r > BLOCKED))), int(r.size)


class SafetyScanCheck(Check):
    """One finding: received, fresh, how many sectors are blocked."""

    def __init__(self, topic: str, timeout_sec: float, grace_until: float):
        self.topic = topic
        self.timeout_sec = timeout_sec
        self.grace_until = grace_until
        self.received: tuple[float, int, int] | None = None

    def configure(self, timeout_sec: float):
        """Apply a live timeout change."""
        self.timeout_sec = timeout_sec

    def on_scan(self, now: float, ranges):
        """Store a scan received at monotonic now."""
        self.received = (now, *blocked_count(ranges))

    def run(self, now: float) -> list[Issue]:
        """Return the finding of the safety scan."""
        name = 'state:safety_scan'
        missing = Level.WARN if now < self.grace_until else Level.ERROR
        if self.received is None:
            return [Issue(missing, name, f'no {self.topic} received')]
        at, blocked, total = self.received
        values = {'blocked': blocked, 'sectors': total, 'age_sec': now - at}
        if now - at > self.timeout_sec:
            return [Issue(Level.ERROR, name, f'{self.topic} stale ({fmt(now - at)} s)', values)]
        if blocked == total:  # also an empty scan
            return [Issue(Level.ERROR, name, 'safety scan blind (every sector blocked)', values)]
        if blocked:
            text = f'safety scan: {blocked} of {total} sectors blocked'
            return [Issue(Level.WARN, name, text, values)]
        return [Issue(Level.OK, name, 'safety scan ok', values)]
