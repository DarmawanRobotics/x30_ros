from __future__ import annotations

from dataclasses import dataclass
import typing

import pytest

from x30_utils.config import build, entries, nested, param, resolve_annotation, type_hints
from x30_utils.config.hints import _resolved_hints


@dataclass(frozen=True)
class Base:
    names: tuple[str, ...] = param(('a',))
    limit: float | None = None


@dataclass(frozen=True)
class Child(Base):
    gains: tuple[float, ...] = param((1.0, 2.0))
    count: int = param(3)


def _shape(hint):
    """Origin and arguments, comparable between typing aliases and PEP 585 generics."""
    args, origin = typing.get_args(hint), typing.get_origin(hint)
    if origin is not None and origin.__module__ == 'types':  # 3.10+: float | None -> UnionType
        origin = typing.Union
    return origin, tuple(_shape(a) if typing.get_args(a) else a for a in args)


@pytest.mark.parametrize(
    ('text', 'expected'),
    [
        ('float | None', typing.Optional[float]),
        ('int | str | None', typing.Union[int, str, None]),
        ('tuple[str, ...]', typing.Tuple[str, ...]),
        ('list[int]', typing.List[int]),
        ('dict[str, list[float]]', typing.Dict[str, typing.List[float]]),
        ('tuple[float, float] | None', typing.Optional[typing.Tuple[float, float]]),
        ('set[int] | frozenset[int]', typing.Union[typing.Set[int], typing.FrozenSet[int]]),
        ('type[Base]', typing.Type[Base]),
    ],
)
def test_resolve_annotation_pep604_pep585(text, expected):
    resolved = resolve_annotation(text, globals())
    assert resolved == expected
    assert _shape(resolved) == _shape(expected)


def test_resolve_annotation_uses_the_module_namespace():
    assert resolve_annotation('Base', globals()) is Base
    assert resolve_annotation('tuple[Child, ...]', globals()) == typing.Tuple[Child, ...]


def test_resolve_annotation_reports_unknown_names():
    with pytest.raises(TypeError, match='Missing'):
        resolve_annotation('Missing | None', {})


@pytest.mark.parametrize('resolver', [type_hints, _resolved_hints])
def test_class_hints_include_bases(resolver):
    hints = resolver(Child)
    assert list(hints) == ['names', 'limit', 'gains', 'count']
    assert _shape(hints['names']) == (tuple, (str, Ellipsis))
    assert _shape(hints['gains']) == (tuple, (float, Ellipsis))
    assert hints['count'] is int
    assert _shape(hints['limit']) == (typing.Union, (float, type(None)))


def test_fallback_matches_get_type_hints_where_available():
    try:
        native = typing.get_type_hints(Child)  # py38: ok
    except TypeError:  # Python 3.8 / 3.9: PEP 604 strings do not evaluate natively
        pytest.skip('typing.get_type_hints cannot evaluate these hints here')
    fallback = _resolved_hints(Child)
    assert {k: _shape(v) for k, v in native.items()} == {k: _shape(v) for k, v in fallback.items()}


@dataclass(frozen=True)
class Leaf:
    names: tuple[str, ...] = param(('a',))
    gains: tuple[float, ...] = param((1.0, 2.0))


@dataclass(frozen=True)
class Sections:
    leaf: Leaf = nested(Leaf)
    rate: float = param(10.0)


def test_config_with_modern_hints_builds():
    assert [e.name for e in entries(Sections)] == ['leaf.names', 'leaf.gains', 'rate']
    cfg = build(Sections, {'leaf.gains': [3, 4], 'leaf.names': ['x', 'y'], 'rate': 5})
    assert cfg.leaf.gains == (3.0, 4.0) and cfg.leaf.names == ('x', 'y') and cfg.rate == 5.0
