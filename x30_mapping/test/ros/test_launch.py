"""mapping.launch: the X30 defaults in each node's namespace and the shared arguments."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'mapping.launch'
NODES = ('site_manager_node', 'route_recorder_node')


def roslaunch(option: str, *args: str) -> str:
    result = subprocess.run(
        ['roslaunch', option, str(LAUNCH), *args],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    assert result.returncode == 0 and 'Unable' not in result.stdout, result.stdout + result.stderr
    return result.stdout


def params(*args: str) -> dict:
    return yaml.safe_load(roslaunch('--dump-params', *args))


def test_the_sites_and_their_routes_no_map():
    assert roslaunch('--nodes').split() == [f'/{n}' for n in NODES]


def test_defaults_land_in_each_node():
    p = params()
    assert p['/site_manager_node/create_site'] is True
    assert p['/route_recorder_node/min_spacing_m'] == 0.2
    assert not any(f'/{node}/sites_dir' in p for node in NODES)  # the dataclass default
    assert not any(name.startswith('/grid_mapper_node/') for name in p)


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text('route_recorder_node:\n  min_spacing_m: 0.5\n')
    p = params('sites_dir:=/data/map', 'site:=gardu', f'params_file:={extra}')
    assert all(p[f'/{node}/sites_dir'] == '/data/map' for node in NODES)
    assert p['/site_manager_node/site'] == 'gardu'
    assert p['/route_recorder_node/min_spacing_m'] == 0.5
    assert p['/route_recorder_node/output_path'] == ''  # the rest stays
    assert params('site:=gardu', 'create_site:=false')['/site_manager_node/create_site'] is False
