"""localization.launch: the X30 defaults and the file arguments."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'localization.launch'


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


def test_the_x30_bridge_frames_and_imu():
    p = params()
    assert p['/localization_node/imu_frame'] == 'imu_link'
    assert p['/localization_node/lio_odom_frame'] == 'camera_init'
    assert p['/localization_node/leveling/imu_topic'] == '/imu/data'
    assert p['/localization_node/sites_dir'] == '~/x30/map'
    assert p['/localization_node/tags_file'] == ''


def test_file_arguments_override_when_set():
    p = params('tags_file:=site/tags.json', 'sites_dir:=/data/map')
    assert p['/localization_node/tags_file'] == 'site/tags.json'
    assert p['/localization_node/sites_dir'] == '/data/map'
