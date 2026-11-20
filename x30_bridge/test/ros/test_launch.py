"""bridge.launch: the gateway's defaults, its arguments, no secret on the parameter server."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'bridge.launch'
NODE = '/teleop_udp_node'


def params(*args: str) -> dict:
    result = subprocess.run(
        ['roslaunch', '--dump-params', str(LAUNCH), *args],
        capture_output=True, text=True, timeout=60, check=False,
    )  # fmt: skip
    assert result.returncode == 0 and 'Unable' not in result.stdout, result.stdout + result.stderr
    return yaml.safe_load(result.stdout)


def test_the_defaults_of_the_navigation_host():
    p = params()
    assert p[f'{NODE}/port'] == 9870 and p[f'{NODE}/remote_source'] == 'handheld'
    assert p[f'{NODE}/robot_command_prefix'] == '/x30/'
    assert p[f'{NODE}/battery_topic'] == '/x30/battery'
    assert p[f'{NODE}/token_env'] == 'X30_TELEOP_TOKEN' and f'{NODE}/token' not in p
    assert p[f'{NODE}/cameras'][0] == 'visible=/hikmicro/visible/image_raw/compressed'


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'more.yaml'
    extra.write_text('max_vx: 0.5\n')
    p = params('sites_dir:=/data/sites', 'robot_ns:=dog', 'port:=9900', f'params_file:={extra}')
    assert p[f'{NODE}/sites_dir'] == '/data/sites' and p[f'{NODE}/port'] == 9900
    assert p[f'{NODE}/cmd_topic'] == '/dog/cmd_vel' and p[f'{NODE}/max_vx'] == 0.5
    assert p[f'{NODE}/max_wz'] == 1.0  # the rest stays
