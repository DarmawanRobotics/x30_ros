"""Parse the compact list entries of the configuration: topics, degradation rules, TF edges."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
import math


@dataclass(frozen=True)
class TopicSpec:
    """A watched topic: minimum rate and largest stamp age (0: no check, None: the default)."""

    topic: str
    min_hz: float
    max_age_sec: float | None


@dataclass(frozen=True)
class Rule:
    """Speed scale requested while a warning whose name matches pattern (fnmatch) is active."""

    pattern: str
    scale: float

    def matches(self, name: str) -> bool:
        """Return True when the issue name matches the pattern."""
        return fnmatchcase(name, self.pattern)


def _number(text: str, what: str) -> float:
    try:
        value = float(text)
    except ValueError:
        raise ValueError(f'{what} {text!r} is not a number') from None
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f'{what} {text!r} must be finite and >= 0')
    return value


def parse_topic(spec: str, default_max_age_sec: float | None = None) -> TopicSpec:
    """Parse 'topic:min_hz[:max_age_sec]'; raise ValueError."""
    parts = spec.strip().split(':')
    topic = parts[0].strip()
    if not topic or len(parts) > 3:
        raise ValueError(f'topic {spec!r}: expected "topic:min_hz[:max_age_sec]"')
    min_hz = _number(parts[1], f'topic {topic}: min_hz') if len(parts) > 1 else 0.0
    max_age = (
        _number(parts[2], f'topic {topic}: max_age_sec') if len(parts) > 2 else default_max_age_sec
    )
    return TopicSpec(topic, min_hz, max_age)


def parse_topics(specs, default_max_age_sec: float | None = None) -> list[TopicSpec]:
    """Parse every topic entry (empty entries skipped); raise ValueError on duplicates."""
    out, seen = [], set()
    for spec in specs:
        if not spec.strip():
            continue
        parsed = parse_topic(spec, default_max_age_sec)
        if parsed.topic in seen:
            raise ValueError(f'topic {parsed.topic} listed twice')
        seen.add(parsed.topic)
        out.append(parsed)
    return out


def parse_rule(spec: str) -> Rule:
    """Parse '<issue name pattern>=<speed scale 0..1>'; raise ValueError."""
    pattern, sep, scale = spec.rpartition('=')
    if not sep or not pattern.strip():
        raise ValueError(f'rule {spec!r}: expected "<issue name pattern>=<speed scale>"')
    value = _number(scale, f'rule {pattern.strip()}: scale')
    if value > 1.0:
        raise ValueError(f'rule {pattern.strip()}: scale {value} is above 1')
    return Rule(pattern.strip(), value)


def parse_rules(specs) -> list[Rule]:
    """Parse every degradation rule (empty entries skipped)."""
    return [parse_rule(s) for s in specs if s.strip()]


def parse_edge(spec: str) -> tuple[str, str]:
    """Parse 'parent->child' into (parent, child); raise ValueError."""
    parent, sep, child = spec.partition('->')
    parent, child = parent.strip().strip('/'), child.strip().strip('/')
    if not sep or not parent or not child:
        raise ValueError(f'tf edge {spec!r}: expected "parent->child"')
    return parent, child


def collect_problems(parse, specs) -> list[str]:
    """Return the ValueError messages of parsing every entry with parse."""
    problems = []
    for spec in specs:
        if not spec.strip():
            continue
        try:
            parse(spec)
        except ValueError as error:
            problems.append(str(error))
    return problems
