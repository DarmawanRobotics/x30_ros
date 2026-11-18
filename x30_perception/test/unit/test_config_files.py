from __future__ import annotations

from pathlib import Path

import yaml

from x30_perception.core.aruco import ArucoConfig
from x30_perception.core.config import PerceptionConfig
from x30_utils.config import build, entries, flatten

CONFIG = Path(__file__).resolve().parents[2] / 'config'


def check(name: str, cls):
    values = flatten(yaml.safe_load((CONFIG / name).read_text()))
    assert sorted(set(values) - {e.name for e in entries(cls)}) == []
    return build(cls, values)


def test_the_lidar_defaults_file_builds_and_names_only_known_parameters():
    config = check('lidar.yaml', PerceptionConfig)
    assert config.input.livox_topic == '/livox/lidar'


def test_the_aruco_defaults_file_builds():
    assert check('aruco.yaml', ArucoConfig) == ArucoConfig()
