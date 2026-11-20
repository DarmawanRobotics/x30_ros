"""fleet.launch: the uplink's defaults, its arguments, no token on the parameter server."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'fleet.launch'
NODE = '/fleet_uplink_node'


def params(*args: str) -> dict:
    result = subprocess.run(
        ['roslaunch', '--dump-params', str(LAUNCH), *args],
        capture_output=True, text=True, timeout=60, check=False,
    )  # fmt: skip
    assert result.returncode == 0 and 'Unable' not in result.stdout, result.stdout + result.stderr
    return yaml.safe_load(result.stdout)


def test_the_defaults_of_the_navigation_host():
    p = params()
    assert p[f'{NODE}/server_url'] == '' and p[f'{NODE}/robot_id'] == 'x30-01'
    assert p[f'{NODE}/token_env'] == 'X30_NETRA_TOKEN' and f'{NODE}/token' not in p
    assert p[f'{NODE}/battery_topic'] == '/x30/battery'
    assert p[f'{NODE}/driver_diagnostics_topic'] == '/x30/diagnostics'
    assert p[f'{NODE}/sites_dir'] == '~/x30/map'


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'more.yaml'
    extra.write_text('telemetry_hz: 2.0\n')
    p = params('server_url:=https://netra.example.com', 'robot_id:=x30-02', 'robot_ns:=dog',
               'sites_dir:=/data/sites', f'params_file:={extra}')  # fmt: skip
    assert p[f'{NODE}/server_url'] == 'https://netra.example.com'
    assert p[f'{NODE}/robot_id'] == 'x30-02' and p[f'{NODE}/sites_dir'] == '/data/sites'
    assert p[f'{NODE}/battery_topic'] == '/dog/battery' and p[f'{NODE}/telemetry_hz'] == 2.0
    assert p[f'{NODE}/upload_period_sec'] == 5.0  # the rest stays
