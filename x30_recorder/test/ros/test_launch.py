"""recorder.launch: the defaults of the perception host, the arguments, no password anywhere."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'recorder.launch'
NODE = '/recorder_node'


def params(*args: str) -> dict:
    result = subprocess.run(
        ['roslaunch', '--dump-params', str(LAUNCH), *args],
        capture_output=True, text=True, timeout=60, check=False,
    )  # fmt: skip
    assert result.returncode == 0 and 'Unable' not in result.stdout, result.stdout + result.stderr
    return yaml.safe_load(result.stdout)


def test_the_defaults_of_the_perception_host():
    p = params()
    assert p[f'{NODE}/recordings_dir'] == '~/x30/recordings'
    assert p[f'{NODE}/video/streams'] == ['visible:101', 'thermal:201']
    assert p[f'{NODE}/disk/min_free_gb'] == 10.0 and p[f'{NODE}/disk/stop_free_gb'] == 3.0
    assert not [k for k in p if 'password' in k]  # the camera's from HIKMICRO_PASSWORD


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'more.yaml'
    extra.write_text('patrol:\n  stop_after_sec: 90.0\n')
    p = params('recordings_dir:=/data/rec', 'host:=10.0.0.9', f'params_file:={extra}')
    assert p[f'{NODE}/recordings_dir'] == '/data/rec' and p[f'{NODE}/video/host'] == '10.0.0.9'
    assert p[f'{NODE}/patrol/stop_after_sec'] == 90.0
    assert p[f'{NODE}/acoustic/chunk_sec'] == 10.0  # the rest stays
