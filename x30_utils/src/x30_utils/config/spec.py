"""Typed configuration: dataclass fields carry their parameter spec (doc, unit, range, live)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

SPEC_KEY = 'x30_spec'
NESTED_KEY = 'x30_nested'


@dataclass(frozen=True)
class Spec:
    """How a configuration value is documented and validated."""

    doc: str = ''
    unit: str = ''
    min: float | None = None
    max: float | None = None
    choices: tuple | None = None
    live: bool = True

    def describe(self) -> str:
        """Return a one-line description for parameter descriptors and docs."""
        text = self.doc
        if self.unit:
            text = f'{text} [{self.unit}]' if text else f'[{self.unit}]'
        if self.choices:
            text = f'{text} ({" | ".join(str(c) for c in self.choices)})'.strip()
        return text


def param(
    default: Any,
    *,
    doc: str = '',
    unit: str = '',
    min: float | None = None,
    max: float | None = None,
    choices: tuple | None = None,
    live: bool = True,
):
    """Declare a configuration field with its default and spec."""
    if isinstance(default, list):
        default = tuple(default)
    spec = Spec(doc, unit, min, max, tuple(choices) if choices else None, live)
    return field(default=default, metadata={SPEC_KEY: spec})


def nested(cls: type):
    """Declare a nested configuration section (parameters prefixed with the field name)."""
    return field(default_factory=cls, metadata={NESTED_KEY: True})
