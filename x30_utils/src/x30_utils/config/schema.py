"""Flatten configuration dataclasses to dotted parameter names, build and validate them."""

from __future__ import annotations

from dataclasses import dataclass, fields, is_dataclass, replace
import math
import typing
from typing import Any

from x30_utils.config.hints import type_hints
from x30_utils.config.spec import NESTED_KEY, SPEC_KEY, Spec

SCALARS = (bool, int, float, str)


class ConfigError(ValueError):
    """One or more configuration values are invalid."""

    def __init__(self, problems: list[str]):
        super().__init__('; '.join(problems))
        self.problems = problems


@dataclass(frozen=True)
class Entry:
    """One leaf of a configuration: its dotted name, type, default and spec."""

    name: str
    path: tuple[str, ...]
    kind: type
    item: type | None
    default: Any
    spec: Spec


def _kind(hint) -> tuple[type, type | None]:
    origin = typing.get_origin(hint)
    if origin in (tuple, list):
        args = [a for a in typing.get_args(hint) if a is not Ellipsis]
        return tuple, (args[0] if args else str)
    if hint in SCALARS:
        return hint, None
    raise TypeError(f'unsupported configuration type {hint!r}')


def entries(cls: type, prefix: str = '') -> list[Entry]:
    """Return the leaves of a configuration dataclass, depth first."""
    hints = type_hints(cls)
    out = []
    for f in fields(cls):
        name = f'{prefix}{f.name}'
        if f.metadata.get(NESTED_KEY) or is_dataclass(hints[f.name]):
            for e in entries(hints[f.name], f'{name}.'):
                out.append(replace(e, path=(f.name, *e.path)))
            continue
        kind, item = _kind(hints[f.name])
        spec = f.metadata.get(SPEC_KEY, Spec())
        out.append(Entry(name, (f.name,), kind, item, f.default, spec))
    return out


def coerce(entry: Entry, value: Any) -> tuple[Any, str | None]:
    """Convert a raw value to the entry type; return (value, problem)."""
    if entry.kind is tuple:
        if isinstance(value, str) or not hasattr(value, '__iter__'):
            return value, f'{entry.name}: expected a list'
        items = []
        for v in value:
            item, problem = _scalar(entry.name, entry.item, v)
            if problem:
                return value, problem
            items.append(item)
        if entry.item is str:
            items = [v for v in items if v != '']
        return tuple(items), None
    return _scalar(entry.name, entry.kind, value)


def _scalar(name: str, kind: type, value: Any) -> tuple[Any, str | None]:
    if kind is float and isinstance(value, int) and not isinstance(value, bool):
        value = float(value)
    if kind is int and isinstance(value, float) and value.is_integer():
        value = int(value)
    if not isinstance(value, kind) or (kind is int and isinstance(value, bool)):
        return value, f'{name}: expected {kind.__name__}, got {type(value).__name__}'
    if kind is float and not math.isfinite(value):
        return value, f'{name}: must be finite'
    return value, None


def check(entry: Entry, value: Any) -> str | None:
    """Return a problem with a coerced value, or None."""
    spec = entry.spec
    values = value if entry.kind is tuple else (value,)
    for v in values:
        if spec.min is not None and isinstance(v, (int, float)) and v < spec.min:
            return f'{entry.name}: {v} is below {spec.min}'
        if spec.max is not None and isinstance(v, (int, float)) and v > spec.max:
            return f'{entry.name}: {v} is above {spec.max}'
        if spec.choices and v not in spec.choices:
            return f'{entry.name}: {v!r} is not one of {list(spec.choices)}'
    return None


def build(cls: type, values: dict[str, Any] | None = None, prefix: str = ''):
    """Build a configuration from dotted values on top of the defaults; raise ConfigError."""
    values = values or {}
    problems, flat = [], {}
    for entry in entries(cls, prefix):
        value = values.get(entry.name, entry.default)
        value, problem = coerce(entry, value)
        problem = problem or check(entry, value)
        if problem:
            problems.append(problem)
        flat[entry.path] = value
    if problems:
        raise ConfigError(problems)
    config = _assemble(cls, flat)
    problems = _validate(config, prefix)
    if problems:
        raise ConfigError(problems)
    return config


def _validate(config, prefix: str) -> list[str]:
    """Run validate() of a section and of every nested section, with dotted prefixes."""
    problems = []
    validator = getattr(config, 'validate', None)
    if callable(validator):
        problems += [f'{prefix}{p}' for p in validator() or []]
    for f in fields(config):
        value = getattr(config, f.name)
        if is_dataclass(value):
            problems += _validate(value, f'{prefix}{f.name}.')
    return problems


def _assemble(cls: type, flat: dict[tuple, Any], path: tuple = ()):
    hints = type_hints(cls)
    kwargs = {}
    for f in fields(cls):
        here = (*path, f.name)
        if f.metadata.get(NESTED_KEY) or is_dataclass(hints[f.name]):
            kwargs[f.name] = _assemble(hints[f.name], flat, here)
        else:
            kwargs[f.name] = flat[here]
    return cls(**kwargs)


def values_of(config, prefix: str = '') -> dict[str, Any]:
    """Return the dotted values of a configuration instance."""
    out = {}
    for entry in entries(type(config), prefix):
        value = config
        for part in entry.path:
            value = getattr(value, part)
        out[entry.name] = value
    return out
