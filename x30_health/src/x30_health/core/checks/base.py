"""What every health check returns: one Issue per checked item, OK included."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import IntEnum
import math


class Level(IntEnum):
    """Severity of a finding (the values of HealthStatus and DiagnosticStatus levels)."""

    OK = 0
    WARN = 1
    ERROR = 2


@dataclass(frozen=True)
class Issue:
    """The state of one checked item: level, a stable name, a readable message and values."""

    level: Level
    name: str
    message: str
    values: dict = field(default_factory=dict)

    @property
    def ok(self) -> bool:
        """Return True when nothing is wrong with the item."""
        return self.level is Level.OK


class Check(ABC):
    """A family of checks; the node feeds it its inputs, run() reports every item."""

    @abstractmethod
    def run(self, now: float) -> list[Issue]:
        """Return the findings at monotonic time now (s)."""


def unreadable(name: str, what: str) -> Issue:
    """Return the warning for a value that could not be read (never silently OK)."""
    return Issue(Level.WARN, name, f'{what} unreadable')


def fmt(value: float, digits: int = 1) -> str:
    """Return a number for messages: fixed digits, 'inf' and 'nan' spelled out."""
    if not math.isfinite(value):
        return str(value)
    return f'{value:.{digits}f}'
