"""navigation.launch: the X30 defaults in each node's namespace, the arguments, the remaps."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'navigation.launch'
NODES = (
    'planner_node',
    'path_loader_node',
    'path_follower_node',
    'obstacle_avoidance_node',
    'docking_node',
)


def roslaunch(*args: str) -> str:
    result = subprocess.run(
        ['roslaunch', *args], capture_output=True, text=True, timeout=60, check=False
    )
    assert 'Unable' not in result.stdout, result.stdout + result.stderr
    return result.stdout


def params(*args: str) -> dict:
    return yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))


def test_the_nodes_and_docking_behind_its_argument():
    assert roslaunch('--nodes', str(LAUNCH)).split() == [f'/{n}' for n in NODES]
    assert '/docking_node' not in roslaunch('--nodes', str(LAUNCH), 'docking:=false').split()


def test_defaults_land_in_each_node():
    p = params()
    assert p['/path_follower_node/controller'] == 'holonomic'
    assert p['/path_follower_node/speed/cruise'] == 0.4
    assert p['/path_loader_node/sites_dir'] == '~/x30/map'
    assert p['/obstacle_avoidance_node/policy'] == 'vfh'
    assert p['/docking_node/charge/confirm_timeout_sec'] == 0.0
    assert p['/planner_node/costmap/inflation_radius'] == 0.8


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text('path_follower_node:\n  loop_route: true\n')
    p = params('sites_dir:=/data/map', 'route:=gardu/routes/a.csv', f'params_file:={extra}')
    assert p['/path_loader_node/sites_dir'] == '/data/map'
    assert p['/path_loader_node/route'] == 'gardu/routes/a.csv'
    assert p['/path_follower_node/loop_route'] is True
    assert p['/path_follower_node/controller'] == 'holonomic'  # the rest stays


def test_the_robot_topics_are_remapped():
    for node in ('/path_follower_node', '/docking_node'):
        out = roslaunch('--args', node, str(LAUNCH))
        assert 'battery:=/x30/battery' in out, out
    assert 'odom:=/x30/odom' in roslaunch('--args', '/path_follower_node', str(LAUNCH))
