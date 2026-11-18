"""bringup.launch: the stack layers behind their arguments, one parent for base_link."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch'
LAYERS = (
    'odometry:=true',
    'odometry_method:=external',
    'localization:=true',
    'mapping:=true',
    'navigation:=true',
)


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
        '/planner_node',
        '/path_loader_node',
        '/path_follower_node',
        '/obstacle_avoidance_node',
        '/docking_node',
    ):
        assert node in nodes


def test_the_layers_follow_the_namespace_and_the_site():
    out = roslaunch('--dump-params', str(LAUNCH), *LAYERS, 'namespace:=dog', 'site:=gardu')
    params = yaml.safe_load(out)
    assert params['/odometry_monitor_node/leg_odom_topic'] == '/dog/odom'
    assert params['/site_manager_node/site'] == 'gardu'
    follower = roslaunch('--args', '/path_follower_node', str(LAUNCH), *LAYERS, 'namespace:=dog')
    assert 'battery:=/dog/battery' in follower and 'odom:=/dog/odom' in follower


HEALTH_CASES = (
    (),
    ('control:=true', 'navigation:=true'),
    ('odometry:=true',),  # FAST-LIO
    ('control:=true', 'perception:=true', *LAYERS),
    ('sim:=true', 'control:=true', 'perception:=true', *LAYERS),
)


def test_health_expects_every_node_started_but_itself():
    for layers in HEALTH_CASES:
        nodes = set(roslaunch('--nodes', str(LAUNCH), 'health:=true', *layers).split())
        params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), 'health:=true', *layers))
        expected = params['/health_monitor_node/nodes/expected']
        assert len(expected) == len(set(expected)), expected
        assert set(expected) == nodes - {'/health_monitor_node'}, layers


def test_health_watches_the_states_and_sensors_of_the_layers_started():
    params = yaml.safe_load(
        roslaunch('--dump-params', str(LAUNCH), 'health:=true', 'control:=true', 'odometry:=true')
    )
    state = {
        k.rsplit('/', 1)[1]: v for k, v in params.items() if '/health_monitor_node/state/' in k
    }
    assert state == {'localization_topic': '', 'safety_scan_topic': ''}  # the others: defaults
    sensors = params['/health_monitor_node/sensors/topics']
    assert sensors == ['/livox/lidar:5.0', '/imu/data:100.0', '/x30/odom:20.0', '/Odometry:8.0']
    assert '/health_monitor_node' in roslaunch('--nodes', str(LAUNCH), 'health:=true').split()
    assert '/health_monitor_node' not in roslaunch('--nodes', str(LAUNCH)).split()


def test_leg_odom_tf_and_localization_together_are_refused():
    out = roslaunch('--dump-params', str(LAUNCH), 'leg_odom_tf:=true', 'localization:=true')
    assert 'both publish odom -> base_link' in out


def test_sim_stands_in_for_the_driver_and_fast_lio():
    nodes = roslaunch('--nodes', str(LAUNCH), 'sim:=true', 'odometry:=true').split()
    assert '/sim_robot_node' in nodes and '/odometry_monitor_node' in nodes
    assert '/x30/x30_driver' not in nodes and '/fast_lio/fast_lio' not in nodes
    params = yaml.safe_load(
        roslaunch('--dump-params', str(LAUNCH), 'sim:=true', 'namespace:=dog', 'leg_odom_tf:=true')
    )
    assert params['/sim_robot_node/driver_ns'] == 'dog'
    assert params['/sim_robot_node/publish_tf'] is True
    assert not any(k.startswith('/dog/x30_driver/') for k in params)


def test_sim_with_fast_lio_is_refused():
    out = roslaunch('--nodes', str(LAUNCH), 'sim:=true', 'odometry_method:=fast_lio')
    assert 'sim:=true stands in for FAST-LIO' in out


def test_sites_dir_reaches_every_layer(tmp_path):
    layers = ('localization:=true', 'mapping:=true', 'navigation:=true')
    args = ('sim:=true', 'site:=lobby', f'sites_dir:={tmp_path}', *layers)
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    for node in ('localization_node', 'site_manager_node', 'grid_mapper_node', 'path_loader_node'):
        assert params[f'/{node}/sites_dir'] == str(tmp_path), node
    assert (tmp_path / 'lobby' / 'map.yaml').is_file()  # drawn while the launch file was read
    real = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *layers))
    assert real['/localization_node/sites_dir'] == '~/x30/map'  # the layers' own files
    assert real['/path_loader_node/sites_dir'] == '~/x30/map'
