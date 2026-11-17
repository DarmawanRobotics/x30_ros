"""Sensor topics: message rate AND header stamp age (a 10 Hz topic with 2 s old stamps is late)."""

from __future__ import annotations

from dataclasses import replace

from x30_health.core.checks.base import Check, Issue, Level, fmt
from x30_health.core.config import SensorsConfig
from x30_health.core.specs import TopicSpec, parse_topics
from x30_utils.math import RateMeter


class TopicMonitor:
    """Arrivals of one topic and the stamp age of its newest message."""

    def __init__(self, spec: TopicSpec, window_sec: float):
        self.spec = spec
        self.meter = RateMeter(window_sec)
        self.subscribed = False
        self.note = ''
        self.age_sec: float | None = None

    def on_message(self, now: float, age_sec: float | None):
        """Record an arrival at monotonic now; age_sec: clock now - header stamp (None: none)."""
        self.meter.tick(now)
        if age_sec is not None:
            self.age_sec = age_sec


class SensorCheck(Check):
    """One finding per configured topic: published, fast enough, stamps neither old nor ahead."""

    def __init__(self, config: SensorsConfig, resolve=None):
        """resolve: topic -> fully qualified name (the node's namespace), identity by default."""
        self.config = config
        resolve = resolve or (lambda topic: topic)
        specs = [replace(s, topic=resolve(s.topic)) for s in parse_topics(config.topics)]
        self.topics = {s.topic: TopicMonitor(s, config.window_sec) for s in specs}

    def configure(self, config: SensorsConfig):
        """Apply live changes (thresholds; the topic list is fixed at start)."""
        self.config = config

    def subscribed(self, topic: str):
        """Mark a topic as subscribed (it is published)."""
        self.topics[topic].subscribed = True

    def unavailable(self, topic: str, note: str):
        """Say why a published topic cannot be subscribed (e.g. its type is not installed)."""
        self.topics[topic].note = note

    def on_message(self, topic: str, now: float, age_sec: float | None = None):
        """Record a message of topic at monotonic now."""
        self.topics[topic].on_message(now, age_sec)

    def run(self, now: float) -> list[Issue]:
        """Return one finding per topic."""
        return [self._check(m, now) for m in self.topics.values()]

    def _check(self, monitor: TopicMonitor, now: float) -> Issue:
        spec = monitor.spec
        name = f'sensor:{spec.topic}'
        if not monitor.subscribed:
            reason = f'not subscribed: {monitor.note}' if monitor.note else 'not published'
            text = f'{spec.topic} {reason}'
            return Issue(Level.WARN, name, text, {'min_hz': spec.min_hz})
        hz = monitor.meter.rate(now)
        values = {'hz': hz, 'min_hz': spec.min_hz}
        problems = []
        if hz < spec.min_hz * self.config.rate_tolerance:
            problems.append(f'{fmt(hz)} Hz (min {fmt(spec.min_hz)})')
        if monitor.age_sec is not None:
            values['age_sec'] = monitor.age_sec
            problems += self._age_problems(spec, monitor.age_sec)
        if problems:
            return Issue(Level.WARN, name, f'{spec.topic} {", ".join(problems)}', values)
        return Issue(Level.OK, name, f'{spec.topic} {fmt(hz)} Hz', values)

    def _age_problems(self, spec: TopicSpec, age: float) -> list[str]:
        limit = self.config.max_age_sec if spec.max_age_sec is None else spec.max_age_sec
        if limit <= 0.0:
            return []
        if age < -self.config.max_future_sec:
            return [f'stamps {fmt(-age, 2)} s in the future (clock offset?)']
        if age > limit:
            return [f'stamps {fmt(age, 2)} s old (max {fmt(limit, 2)})']
        return []
