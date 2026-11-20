"""navigation.launch: the X30 defaults in each node's namespace, the arguments, the remaps."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'navigation.launch'
NODES = (
    'site_manager_node',
    'route_recorder_node',
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
    assert p['/site_manager_node/create_site'] is True
    assert p['/route_recorder_node/min_spacing_m'] == 0.2
    assert not any(name.startswith(('/planner_node/', '/grid_mapper_node/')) for name in p)


def test_arguments_override_when_set(tmp_path):
    extra = tmp_path / 'site.yaml'
    extra.write_text('path_follower_node:\n  loop_route: true\n')
    p = params(
        'sites_dir:=/data/map', 'site:=gardu', 'route:=gardu/routes/a.csv', f'params_file:={extra}'
    )
    for node in ('site_manager_node', 'route_recorder_node', 'path_loader_node'):
        assert p[f'/{node}/sites_dir'] == '/data/map', node
    assert p['/site_manager_node/site'] == 'gardu'
    assert p['/path_loader_node/route'] == 'gardu/routes/a.csv'
    assert params('create_site:=false')['/site_manager_node/create_site'] is False
    assert p['/path_follower_node/loop_route'] is True
    assert p['/path_follower_node/controller'] == 'holonomic'  # the rest stays


def test_the_robot_topics_are_remapped():
    for node in ('/path_follower_node', '/docking_node'):
        out = roslaunch('--args', node, str(LAUNCH))
        assert 'battery:=/x30/battery' in out, out
    assert 'odom:=/x30/odom' in roslaunch('--args', '/path_follower_node', str(LAUNCH))


def test_neupan_gets_the_x30_model_unless_a_params_file_gives_another(tmp_path):
    model = Path(__file__).resolve().parents[2] / 'models' / 'dune_x30.pth'
    assert Path(params()['/path_follower_node/neupan/dune_checkpoint']).resolve() == model
    extra = tmp_path / 'own.yaml'
    extra.write_text('path_follower_node:\n  neupan:\n    dune_checkpoint: /data/own.pth\n')
    p = params(f'params_file:={extra}')
    assert p['/path_follower_node/neupan/dune_checkpoint'] == '/data/own.pth'


def test_the_controller_argument_wins_over_the_files(tmp_path):
    extra = tmp_path / 'own.yaml'
    extra.write_text('path_follower_node:\n  controller: mppi\n')
    assert params(f'params_file:={extra}')['/path_follower_node/controller'] == 'mppi'
    p = params(f'params_file:={extra}', 'controller:=neupan')
    assert p['/path_follower_node/controller'] == 'neupan'


def test_the_follower_runs_with_the_python_of_neupan(monkeypatch):
    monkeypatch.delenv('X30_PYTHON', raising=False)
    assert 'launch-prefix' not in roslaunch('--args', '/path_follower_node', str(LAUNCH))
    out = roslaunch('--args', '/path_follower_node', str(LAUNCH), 'python:=/opt/venv/bin/python')
    assert out.startswith('/opt/venv/bin/python '), out
    monkeypatch.setenv('X30_PYTHON', '/home/ysc/x30_ws/neupan/bin/python')
    out = roslaunch('--args', '/path_follower_node', str(LAUNCH))
    assert out.startswith('/home/ysc/x30_ws/neupan/bin/python '), out
