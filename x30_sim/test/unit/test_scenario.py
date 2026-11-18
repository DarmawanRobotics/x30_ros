"""Scenarios: shipped files load, timed obstacles move, dropouts, restarts, overrides, errors."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from x30_sim.core.config import SimConfig
from x30_sim.core.scenario import (
    Scenario,
    ScenarioError,
    load_scenario,
    make_rngs,
    scenario_from_dict,
)

SCENARIOS = Path(__file__).resolve().parents[2] / 'config' / 'scenarios'


@pytest.mark.parametrize('path', sorted(SCENARIOS.glob('*.yaml')), ids=lambda p: p.stem)
def test_shipped_scenarios_load(path):
    """Every scenario in config/scenarios loads and applies to the default configuration."""
    scenario = load_scenario(str(path))
    assert scenario.seed is not None
    scenario.apply(SimConfig())


def test_timed_obstacles_appear_move_and_leave():
    """An obstacle exists from start to end and moves with its velocity."""
    scenario = scenario_from_dict(
        {'obstacles': [{'start': 2.0, 'end': 4.0, 'x': 1.0, 'y': 0.0, 'vy': 0.5, 'radius': 0.3}]}
    )
    assert scenario.circles_at(1.9).shape == (0, 4)
    assert scenario.circles_at(3.0) == pytest.approx(np.array([[1.0, 0.5, 0.3, 1.7]]))
    assert scenario.circles_at(4.0).shape == (0, 4)


def test_dropouts_and_restarts():
    """Dropouts silence their sensor ('all': every sensor); restarts fire once in (t0, t1]."""
    scenario = load_scenario(str(SCENARIOS / 'sensor_faults.yaml'))
    assert scenario.silent('lidar', 10.5) and not scenario.silent('odometry', 10.5)
    assert not scenario.silent('lidar', 11.6)
    assert scenario.silent('odometry', 20.1) and scenario.silent('imu', 30.5)
    assert scenario.restart_due(39.99, 40.0) and not scenario.restart_due(40.0, 40.01)


def test_overrides_replace_the_parameters():
    """The seed, latency and drift of a scenario replace the node's values."""
    config = load_scenario(str(SCENARIOS / 'sensor_faults.yaml')).apply(SimConfig(seed=99))
    assert config.seed == 3 and config.latency.lidar_sec == pytest.approx(0.15)
    assert config.odometry == SimConfig().odometry
    assert Scenario().apply(SimConfig(seed=99)).seed == 99
    legs = scenario_from_dict({'leg_odometry': {'scale_error': 0.05}}).apply(SimConfig())
    assert legs.leg_odometry.scale_error == 0.05 and legs.odometry == SimConfig().odometry


def test_seeded_generators_are_reproducible_and_independent():
    """make_rngs gives the same streams for a seed, different ones across sensors and seeds."""
    a, b = make_rngs(4), make_rngs(4)
    assert all(np.array_equal(x.random(5), y.random(5)) for x, y in zip(a, b))
    lidar, odom, *_ = make_rngs(4)
    assert len(make_rngs(4)) == 5  # LiDAR, odometry, IMU, leg odometry, camera
    assert not np.array_equal(lidar.random(5), odom.random(5))
    assert not np.array_equal(make_rngs(4)[0].random(5), make_rngs(5)[0].random(5))


@pytest.mark.parametrize(
    ('data', 'match'),
    [
        ({'sed': 1}, 'unknown keys'),
        ({'latency': {'lidar': 0.1}}, 'unknown keys'),
        ({'latency': {'lidar_sec': -1.0}}, 'below'),
        ({'odometry': {'scale_error': 'big'}}, 'expected float'),
        ({'obstacles': [{'x': 1.0}]}, 'scenario'),
        ({'obstacles': [{'x': 1.0, 'y': 0.0, 'radius': 0.0}]}, 'positive'),
        ({'dropouts': [{'start': 1.0, 'duration': 1.0, 'sensor': 'camera'}]}, 'sensor'),
        ({'dropouts': [{'start': 1.0}]}, 'duration'),
        ([1, 2], 'mapping'),
    ],
)
def test_bad_scenarios_raise(data, match):
    """Typos and invalid values raise ScenarioError with the reason."""
    with pytest.raises(ScenarioError, match=match):
        scenario_from_dict(data)


def test_unreadable_scenario_names_the_file(tmp_path):
    """A broken YAML file raises ScenarioError with its path."""
    path = tmp_path / 'broken.yaml'
    path.write_text('seed: [1\n')
    with pytest.raises(ScenarioError, match=r'broken\.yaml'):
        load_scenario(str(path))
