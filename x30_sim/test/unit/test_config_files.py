"""The shipped defaults: config/sim.yaml builds, names only known parameters, keeps the X30's."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from x30_sim.core.config import X30_LIDARS, LidarConfig, SimConfig
from x30_utils.config import ConfigError, build, entries, flatten

CONFIG = Path(__file__).resolve().parents[2] / 'config'


def test_sim_yaml_builds_and_names_only_known_parameters():
    values = flatten(yaml.safe_load((CONFIG / 'sim.yaml').read_text()))
    assert sorted(set(values) - {e.name for e in entries(SimConfig)}) == []
    config = build(SimConfig, values)
    assert config.lidar.mount_list == list(X30_LIDARS)
    assert config.imu == SimConfig().imu and config.lio == SimConfig().lio
    assert config.driver_ns == 'x30' and config.imu.topic == '/imu/data'


def test_bad_mounts_are_refused():
    assert LidarConfig().validate() == []
    assert LidarConfig(mounts=(0.0, 0.0, 0.0, 0.0)).validate()
    with pytest.raises(ConfigError, match='five values'):
        build(SimConfig, {'lidar.mounts': [0.0, 0.0, 0.1]})
