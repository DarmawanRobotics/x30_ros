from __future__ import annotations

from pathlib import Path

import yaml

from x30_control.core.config import FieldConfig, MuxConfig
from x30_utils.config import build, entries, flatten

CONFIG = Path(__file__).resolve().parents[2] / 'config' / 'control.yaml'


def test_the_defaults_file_builds_and_names_only_known_parameters():
    values = flatten(yaml.safe_load(CONFIG.read_text()))
    known = {e.name for e in entries(MuxConfig)} | {e.name for e in entries(FieldConfig, 'field.')}
    assert sorted(set(values) - known) == []
    assert build(MuxConfig, values) == MuxConfig()  # the file restates the code's defaults
    assert build(FieldConfig, values, 'field.') == FieldConfig()
