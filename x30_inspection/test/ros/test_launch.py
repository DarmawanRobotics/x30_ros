"""inspection.launch: the defaults in the node's namespace and the arguments on top."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'inspection.launch'
NODE = '/inspection_node'


def roslaunch(*args: str) -> str:
    result = subprocess.run(
        ['roslaunch', *args], capture_output=True, text=True, timeout=60, check=False
    )
    assert 'Unable' not in result.stdout, result.stdout + result.stderr
    return result.stdout


def params(*args: str) -> dict:
    return yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))


def test_one_node_with_the_defaults():
    assert roslaunch('--nodes', str(LAUNCH)).split() == [NODE]
    p = params()
    assert p[f'{NODE}/sites_dir'] == '~/x30/map'
    assert p[f'{NODE}/reports_dir'] == '~/x30/reports'
    assert p[f'{NODE}/camera/namespace'] == '/hikmicro'
    assert p[f'{NODE}/acoustic/namespace'] == '/hertzino'
    assert p[f'{NODE}/thermal/ambient_c'] == 30.0
    assert not any('password' in key for key in p)  # the drivers read their own


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text('thermal:\n  ambient_c: 34.0\nacoustic:\n  enabled: false\n')
    robot = tmp_path / 'robot.yaml'
    robot.write_text('frames:\n  map: map\n')
    p = params(
        'sites_dir:=/data/map',
        'reports_dir:=/data/reports',
        f'params_file:={extra}',
        f'robot_params:={robot}',
    )
    assert p[f'{NODE}/sites_dir'] == '/data/map'
    assert p[f'{NODE}/reports_dir'] == '/data/reports'
    assert p[f'{NODE}/thermal/ambient_c'] == 34.0
    assert p[f'{NODE}/acoustic/enabled'] is False
    assert p[f'{NODE}/thermal/samples'] == 3  # the rest stays
    assert p['/robot/frames/map'] == 'map'
