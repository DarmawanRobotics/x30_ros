from __future__ import annotations

from dataclasses import dataclass

import pytest

from x30_utils.config import ConfigError, build, entries, nested, param, values_of
from x30_utils.robot import RobotProfile


@dataclass(frozen=True)
class Inner:
    gain: float = param(1.0, min=0.0, max=10.0)


@dataclass(frozen=True)
class Outer:
    rate: float = param(20.0, min=1.0, max=100.0, unit='Hz', live=False)
    mode: str = param('a', choices=('a', 'b'))
    topics: tuple[str, ...] = param(('x', 'y'))
    count: int = param(3, min=0)
    inner: Inner = nested(Inner)

    def validate(self) -> list[str]:
        return ['count must be < 10'] if self.count >= 10 else []


def test_entries_are_dotted():
    names = [e.name for e in entries(Outer, 'p.')]
    assert names == ['p.rate', 'p.mode', 'p.topics', 'p.count', 'p.inner.gain']


def test_build_defaults_and_overrides():
    cfg = build(Outer, {'inner.gain': 2, 'topics': ['a', '', 'b']})
    assert cfg.inner.gain == 2.0 and isinstance(cfg.inner.gain, float)
    assert cfg.topics == ('a', 'b')
    assert values_of(cfg)['inner.gain'] == 2.0


@pytest.mark.parametrize(
    'values',
    [
        {'rate': 0.5},
        {'rate': float('nan')},
        {'mode': 'c'},
        {'count': 2.5},
        {'count': True},
        {'topics': 'abc'},
        {'count': 12},
    ],
)
def test_build_rejects(values):
    with pytest.raises(ConfigError):
        build(Outer, values)


def test_robot_profile_derived_values():
    robot = build(RobotProfile)
    assert robot.footprint.circumscribed_radius == pytest.approx(0.488, abs=1e-3)
    assert robot.footprint.inscribed_radius == pytest.approx(0.28)
    assert robot.can_strafe
    with pytest.raises(ConfigError):
        build(RobotProfile, {'kinematics': 'diff'})
    diff = build(RobotProfile, {'kinematics': 'diff', 'limits.max_vy': 0.0})
    assert not diff.can_strafe


@dataclass(frozen=True)
class Checked:
    low: float = param(1.0)
    high: float = param(2.0)

    def validate(self) -> list[str]:
        return ['low must be below high'] if self.low >= self.high else []


@dataclass(frozen=True)
class Parent:
    band: Checked = nested(Checked)


def test_nested_validate_runs_with_prefix():
    with pytest.raises(ConfigError, match=r'p\.band\.low must be below high'):
        build(Parent, {'p.band.low': 3.0}, 'p.')
