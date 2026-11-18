"""odometry.launch: FAST-LIO's parameters in fast_lio/, the gate, the arguments, the methods."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'odometry.launch'


def dump(*args: str) -> tuple[int, str]:
    """Return the exit code and the output of roslaunch --dump-params."""
    result = subprocess.run(
        ['roslaunch', '--dump-params', str(LAUNCH), *args],
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    return result.returncode, result.stdout + result.stderr


def params(*args: str) -> dict:
    code, out = dump(*args)
    assert code == 0, out
    return yaml.safe_load(out)


def test_fast_lio_reads_the_x30_files_in_its_namespace():
    p = params()
    assert p['/fast_lio/common/lid_topic'] == '/livox/lidar'
    assert p['/fast_lio/common/imu_topic'] == '/imu/data'
    assert p['/fast_lio/mapping/extrinsic_T'] == [-0.13045, 0.001, -0.0149]
    assert p['/fast_lio/mapping/extrinsic_est_en'] is False
    assert p['/fast_lio/publish/path_en'] is False
    assert p['/fast_lio/pcd_save/pcd_save_en'] is False
    assert p['/fast_lio/odometry_gate/imu_topic'] == '/imu/data'
    assert p['/odometry_monitor_node/leg_odom_topic'] == '/x30/odom'
    assert not any(k.startswith(('/common/', '/mapping/', '/preprocess/')) for k in p)


def test_save_map_and_leg_odom_arguments():
    p = params('save_map:=true', 'leg_odom_topic:=/leg_odom')
    assert p['/fast_lio/pcd_save/pcd_save_en'] is True
    assert p['/odometry_monitor_node/leg_odom_topic'] == '/leg_odom'


def test_external_runs_the_monitor_only():
    p = params('method:=external')
    assert not any(k.startswith('/fast_lio/') for k in p)
    assert p['/odometry_monitor_node/odom_topic'] == '/Odometry'


def test_an_unknown_method_fails_the_launch():
    _, out = dump('method:=fastlio')  # roslaunch itself exits 1, --dump-params only reports it
    assert "Invalid <arg> tag: 'fastlio'" in out
