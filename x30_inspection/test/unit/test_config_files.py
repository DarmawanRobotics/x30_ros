"""The shipped defaults: config/inspection.yaml builds and names only known parameters."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from x30_inspection.core.config import AbsoluteLevels, InspectionConfig
from x30_utils.config import ConfigError, build, entries, flatten

CONFIG = Path(__file__).resolve().parents[2] / 'config'


def test_inspection_yaml_builds_and_names_only_known_parameters():
    values = flatten(yaml.safe_load((CONFIG / 'inspection.yaml').read_text()))
    assert sorted(set(values) - {e.name for e in entries(InspectionConfig)}) == []
    config = build(InspectionConfig, values)
    assert config.sites_dir == '~/x30/map'
    assert config.camera.namespace == '/hikmicro' and config.acoustic.namespace == '/hertzino'


def test_levels_must_rise():
    assert AbsoluteLevels().validate() == []
    with pytest.raises(ConfigError, match='attention <= warning <= critical'):
        build(InspectionConfig, {'thermal.absolute.warning': 120.0})
