"""bringup.launch: the stack layers behind their arguments, one parent for base_link."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch'
LAYERS = ('odometry:=true', 'odometry_method:=external', 'localization:=true', 'mapping:=true')


def roslaunch(*args: str) -> str:
    result = subprocess.run(
        ['roslaunch', *args], capture_output=True, text=True, timeout=60, check=False
    )
    return result.stdout + result.stderr


def test_the_robot_alone_by_default():
    nodes = roslaunch('--nodes', str(LAUNCH)).split()
    assert nodes == ['/x30/x30_driver', '/x30/robot_state_publisher']


def test_every_layer_behind_its_argument():
    nodes = roslaunch('--nodes', str(LAUNCH), *LAYERS).split()
    for node in (
        '/odometry_monitor_node',
        '/localization_node',
        '/grid_mapper_node',
        '/site_manager_node',
        '/route_recorder_node',
    ):
        assert node in nodes


def test_the_layers_follow_the_namespace_and_the_site():
    out = roslaunch('--dump-params', str(LAUNCH), *LAYERS, 'namespace:=dog', 'site:=gardu')
    params = yaml.safe_load(out)
    assert params['/odometry_monitor_node/leg_odom_topic'] == '/dog/odom'
    assert params['/site_manager_node/site'] == 'gardu'


def test_leg_odom_tf_and_localization_together_are_refused():
    out = roslaunch('--dump-params', str(LAUNCH), 'leg_odom_tf:=true', 'localization:=true')
    assert 'both publish odom -> base_link' in out
