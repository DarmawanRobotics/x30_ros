"""mapping.launch: the X30 defaults in each node's namespace and the shared arguments."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'mapping.launch'
NODES = ('grid_mapper_node', 'site_manager_node', 'route_recorder_node')


def params(*args: str) -> dict:
    result = subprocess.run(
        ['roslaunch', '--dump-params', str(LAUNCH), *args],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0 and 'Unable' not in result.stdout, result.stdout + result.stderr
    return yaml.safe_load(result.stdout)


def test_defaults_land_in_each_node():
    p = params()
    assert p['/grid_mapper_node/cloud_topic'] == '/cloud_registered_body'
    assert p['/grid_mapper_node/grid/resolution'] == 0.05
    assert p['/site_manager_node/start_mapping'] is False
    assert p['/route_recorder_node/min_spacing_m'] == 0.2
    assert not any(f'/{node}/sites_dir' in p for node in NODES)  # the dataclass default


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text('grid_mapper_node:\n  rate_hz: 1.0\n')
    p = params(
        'sites_dir:=/data/map', 'site:=gardu', 'start_mapping:=true', f'params_file:={extra}'
    )
    assert all(p[f'/{node}/sites_dir'] == '/data/map' for node in NODES)
    assert p['/site_manager_node/site'] == 'gardu'
    assert p['/site_manager_node/start_mapping'] is True
    assert p['/grid_mapper_node/rate_hz'] == 1.0
    assert p['/grid_mapper_node/cloud_topic'] == '/cloud_registered_body'  # the rest stays
