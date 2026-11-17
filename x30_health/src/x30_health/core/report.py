"""Aggregate the findings into one level, the issue lines and the requested speed scale."""

from __future__ import annotations

from dataclasses import dataclass

from x30_health.core.checks.base import Issue, Level
from x30_health.core.config import DegradationConfig
from x30_health.core.specs import Rule, parse_rules


@dataclass(frozen=True)
class Report:
    """The overall health: worst level, every finding and the speed scale (0..1)."""

    level: Level
    findings: tuple[Issue, ...]
    speed_scale: float

    @property
    def issues(self) -> list[Issue]:
        """The findings that are not OK, errors first."""
        bad = [f for f in self.findings if not f.ok]
        return sorted(bad, key=lambda f: -f.level)

    def lines(self) -> list[str]:
        """One line per problem, errors first."""
        return [f.message for f in self.issues]


def issue_scale(issue: Issue, rules: list[Rule], warn_scale: float) -> float:
    """Return the speed scale one finding asks for: OK 1, ERROR 0, WARN by the lowest rule."""
    if issue.level is Level.OK:
        return 1.0
    if issue.level is Level.ERROR:
        return 0.0
    matching = [r.scale for r in rules if r.matches(issue.name)]
    return min(matching) if matching else warn_scale


def aggregate(findings, config: DegradationConfig) -> Report:
    """Return the report of all findings under the degradation rules."""
    findings = tuple(findings)
    rules = parse_rules(config.rules)
    level = max((f.level for f in findings), default=Level.OK)
    scale = min((issue_scale(f, rules, config.warn_scale) for f in findings), default=1.0)
    return Report(Level(level), findings, float(scale))
