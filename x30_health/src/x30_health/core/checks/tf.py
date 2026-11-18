"""TF edges: the age of each dynamic edge and why it is late (clock, load, stopped)."""

from __future__ import annotations

from collections import deque

from x30_health.core.checks.base import Check, Issue, Level, fmt
from x30_health.core.config import TfConfig
from x30_health.core.specs import parse_edge

MIN_SAMPLES = 3  # samples needed to tell a clock offset from load


class EdgeHistory:
    """(clock time, age) samples of one edge over a window."""

    def __init__(self):
        self.samples: deque[tuple[float, float]] = deque()

    def add(self, t: float, age: float, window_sec: float):
        """Add a sample at clock time t (s) and forget samples older than the window."""
        self.samples.append((t, age))
        while self.samples and t - self.samples[0][0] > window_sec:
            self.samples.popleft()

    def stamp_advanced(self) -> bool:
        """Return True when the newest transform changed between the last two samples."""
        if len(self.samples) < 2:
            return True
        (t0, a0), (t1, a1) = self.samples[-2], self.samples[-1]
        return (t1 - a1) > (t0 - a0) + 1e-6


def diagnose(label: str, history: EdgeHistory, config: TfConfig) -> Issue:
    """Return the finding of one edge from its age history."""
    name = f'tf:{label}'
    age = history.samples[-1][1]
    values = {'age_sec': age}
    if age < -config.max_future_sec:
        text = f'{label} stamps {fmt(-age, 2)} s in the future (clock offset?)'
        return Issue(Level.WARN, name, text, {**values, 'cause': 'clock'})
    if age <= config.max_age_sec:
        return Issue(Level.OK, name, f'{label} {fmt(age, 2)} s', values)
    if not history.stamp_advanced():
        text = f'{label} not updated for {fmt(age)} s (publisher stopped?)'
        return Issue(Level.WARN, name, text, {**values, 'cause': 'stopped'})
    ages = [a for _, a in history.samples]
    if len(ages) < MIN_SAMPLES:
        return Issue(Level.WARN, name, f'{label} {fmt(age)} s old', {**values, 'cause': 'late'})
    low, high = min(ages), max(ages)
    if low > config.max_age_sec and high - low <= config.offset_jitter_sec:
        text = f'{label} {fmt(age)} s old, steady (source clock offset?)'
        return Issue(Level.WARN, name, text, {**values, 'cause': 'clock'})
    text = f'{label} {fmt(age)} s old, varying {fmt(low)}-{fmt(high)} s (system load?)'
    return Issue(Level.WARN, name, text, {**values, 'cause': 'load'})


class TfCheck(Check):
    """One finding per dynamic edge (and per required edge), plus the robot's odom->base chain."""

    def __init__(self, config: TfConfig, chain: tuple[str, str] | None = None):
        self.config = config
        self.chain = chain
        self.history: dict[tuple[str, str], EdgeHistory] = {}
        self.current: set[tuple[str, str]] = set()
        self.chain_ok: bool | None = None

    def configure(self, config: TfConfig):
        """Apply live changes."""
        self.config = config

    def observe(self, t: float, edges: dict, chain_ok: bool | None = None):
        """Add edge ages at clock time t: edges is child -> (parent, age) (TfCache.edge_ages)."""
        self.current = set()
        for child, (parent, age) in edges.items():
            key = (parent.strip('/'), child.strip('/'))
            self.current.add(key)
            self.history.setdefault(key, EdgeHistory()).add(t, age, self.config.window_sec)
        self.chain_ok = chain_ok

    def run(self, now: float) -> list[Issue]:
        """Return one finding per known edge, required edge and the chain."""
        out = []
        required = {parse_edge(e) for e in self.config.edges if e.strip()}
        for key in sorted(set(self.history) | required):
            label = f'{key[0]}->{key[1]}'
            if key not in self.history:
                out.append(Issue(Level.WARN, f'tf:{label}', f'{label}: no transform'))
            elif key not in self.current:
                out.append(Issue(Level.WARN, f'tf:{label}', f'{label} gone (publisher stopped?)'))
            else:
                out.append(diagnose(label, self.history[key], self.config))
        if self.chain is not None and self.chain_ok is not None:
            label = f'{self.chain[0]}->{self.chain[1]}'
            level, text = (Level.OK, label) if self.chain_ok else (Level.WARN, f'no TF {label}')
            out.append(Issue(level, f'tf:chain {label}', text))
        return out
