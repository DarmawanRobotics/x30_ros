"""Reproducible scenarios from YAML: seed, obstacles, dropouts, restarts, overrides."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
import math

import numpy as np
import yaml

from x30_sim.core.config import LatencyConfig, OdometryNoiseConfig, SimConfig
from x30_utils.config import ConfigError, build, values_of
from x30_utils.math import default_rngs

SENSORS = ('lidar', 'odometry', 'leg_odometry', 'imu', 'all')  # odometry: FAST-LIO's
NOISE = ('odometry', 'leg_odometry')  # noise sections a scenario may set
KEYS = {'seed', 'latency', *NOISE, 'obstacles', 'dropouts', 'odometry_restarts'}
OBSTACLE_KEYS = {'start', 'end', 'x', 'y', 'radius', 'height', 'vx', 'vy'}
DROPOUT_KEYS = {'start', 'duration', 'sensor'}
LATENCY_KEYS = {f.name for f in fields(LatencyConfig)}
ODOMETRY_KEYS = {f.name for f in fields(OdometryNoiseConfig)}


class ScenarioError(ValueError):
    """The scenario cannot be used."""


@dataclass(frozen=True)
class TimedObstacle:
    """A circle present from start to end (s), moving with (vx, vy) m/s from (x, y) at start."""

    x: float
    y: float
    start: float = 0.0
    end: float = math.inf
    radius: float = 0.25
    height: float = 1.7
    vx: float = 0.0
    vy: float = 0.0

    def circle_at(self, t: float) -> tuple | None:
        """Return (x, y, radius, height) at t, or None while absent."""
        if not self.start <= t < self.end:
            return None
        dt = t - self.start
        return (self.x + self.vx * dt, self.y + self.vy * dt, self.radius, self.height)


@dataclass(frozen=True)
class Dropout:
    """A sensor silent from start for duration seconds."""

    start: float
    duration: float
    sensor: str = 'lidar'

    def covers(self, sensor: str, t: float) -> bool:
        """Return True when sensor is silent at t."""
        return self.sensor in (sensor, 'all') and self.start <= t < self.start + self.duration


@dataclass(frozen=True)
class Scenario:
    """Everything a scenario file sets."""

    seed: int | None = None
    latency: dict = field(default_factory=dict)
    odometry: dict = field(default_factory=dict)
    leg_odometry: dict = field(default_factory=dict)
    obstacles: tuple[TimedObstacle, ...] = ()
    dropouts: tuple[Dropout, ...] = ()
    odometry_restarts: tuple[float, ...] = ()

    def circles_at(self, t: float) -> np.ndarray:
        """Return the (N, 4) obstacle circles present at t."""
        circles = [c for c in (o.circle_at(t) for o in self.obstacles) if c is not None]
        return np.array(circles, dtype=float).reshape(-1, 4)

    def silent(self, sensor: str, t: float) -> bool:
        """Return True while a dropout silences sensor."""
        return any(d.covers(sensor, t) for d in self.dropouts)

    def restart_due(self, t0: float, t1: float) -> bool:
        """Return True when an odometry restart falls in (t0, t1]."""
        return any(t0 < t <= t1 for t in self.odometry_restarts)

    def apply(self, config: SimConfig) -> SimConfig:
        """Return config with the scenario's seed, latency and odometry values; ScenarioError."""
        values = values_of(config)
        if self.seed is not None:
            values['seed'] = self.seed
        values.update({f'latency.{k}': v for k, v in self.latency.items()})
        values.update({f'odometry.{k}': v for k, v in self.odometry.items()})
        values.update({f'leg_odometry.{k}': v for k, v in self.leg_odometry.items()})
        try:
            return build(SimConfig, values)
        except ConfigError as error:
            raise ScenarioError(str(error)) from error


def _checked(item, keys: set, what: str) -> dict:
    if not isinstance(item, dict):
        raise ScenarioError(f'{what}: expected a mapping, got {item!r}')
    unknown = sorted(set(item) - keys)
    if unknown:
        raise ScenarioError(f'{what}: unknown keys {unknown}')
    return item


def _obstacle(item) -> TimedObstacle:
    values = {k: float(v) for k, v in _checked(item, OBSTACLE_KEYS, 'obstacle').items()}
    obstacle = TimedObstacle(**values)
    if obstacle.radius <= 0.0 or obstacle.height <= 0.0 or obstacle.end <= obstacle.start:
        raise ScenarioError(f'obstacle: radius, height and end - start must be positive: {item}')
    return obstacle


def _dropout(item) -> Dropout:
    item = _checked(item, DROPOUT_KEYS, 'dropout')
    dropout = Dropout(
        float(item['start']), float(item['duration']), str(item.get('sensor', 'lidar'))
    )
    if dropout.sensor not in SENSORS or dropout.duration <= 0.0:
        raise ScenarioError(f'dropout: sensor one of {list(SENSORS)}, duration > 0: {item}')
    return dropout


def scenario_from_dict(data: dict) -> Scenario:
    """Build a scenario from its YAML content; raise ScenarioError."""
    data = _checked(data or {}, KEYS, 'scenario')
    try:
        seed = data.get('seed')
        scenario = Scenario(
            seed=None if seed is None else int(seed),
            latency=dict(_checked(data.get('latency') or {}, LATENCY_KEYS, 'latency')),
            odometry=dict(_checked(data.get('odometry') or {}, ODOMETRY_KEYS, 'odometry')),
            leg_odometry=dict(
                _checked(data.get('leg_odometry') or {}, ODOMETRY_KEYS, 'leg_odometry')
            ),
            obstacles=tuple(_obstacle(o) for o in data.get('obstacles') or []),
            dropouts=tuple(_dropout(d) for d in data.get('dropouts') or []),
            odometry_restarts=tuple(sorted(float(t) for t in data.get('odometry_restarts') or [])),
        )
    except ScenarioError:
        raise
    except (KeyError, TypeError, ValueError) as error:
        raise ScenarioError(f'scenario: {error}') from error
    scenario.apply(SimConfig())  # validates the overrides
    return scenario


def load_scenario(path: str) -> Scenario:
    """Load a scenario YAML file; raise ScenarioError."""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as error:
        raise ScenarioError(f'{path}: {error}') from error
    try:
        return scenario_from_dict(data)
    except ScenarioError as error:
        raise ScenarioError(f'{path}: {error}') from error


def make_rngs(seed: int) -> tuple[np.random.Generator, ...]:
    """Return independent generators: LiDAR, odometry, IMU, leg odometry, tag camera."""
    return tuple(default_rngs(seed, 5))
