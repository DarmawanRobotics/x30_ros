"""bringup.launch: the stack layers behind their arguments, one parent for base_link."""

from __future__ import annotations

import os
from pathlib import Path
import re
import subprocess

import yaml

from x30_localization.core.config import LocalizationConfig
from x30_utils.config import build

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'bringup.launch'
LAYERS = (
    'odometry:=true',
    'odometry_method:=external',
    'localization:=true',
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
        '/site_manager_node',
        '/route_recorder_node',
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


def test_inspection_is_the_followers_hook_and_left_out_of_health():
    args = ('inspection:=true', 'navigation:=true', 'health:=true')
    nodes = set(roslaunch('--nodes', str(LAUNCH), *args).split())
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    assert params['/path_follower_node/hooks/inspection'] == ['/inspection/inspect']
    assert params['/path_follower_node/hooks/inspection_timeout_sec'] == 180.0
    expected = set(params['/health_monitor_node/nodes/expected'])
    assert expected == nodes - {'/health_monitor_node', '/inspection_node'}  # not driving
    assert '/inspection_node' in nodes
    assert '/inspection_node' not in roslaunch('--nodes', str(LAUNCH)).split()


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


def test_no_map_the_odometry_start_is_the_origin(tmp_path):
    """No argument for a map: the start is the origin, the site is created, nothing maps."""
    args = ('sim:=true', 'site:=yard', f'sites_dir:={tmp_path}', *LAYERS, 'health:=true')
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    assert '/localization_node/assume_start_origin' not in params  # x30_localization's own
    assert build(LocalizationConfig, {}).assume_start_origin
    assert params['/site_manager_node/create_site'] is True
    assert params['/site_manager_node/site'] == 'yard'
    assert (tmp_path / 'yard' / 'tags.json').is_file()  # the world's, in the start's frame
    nodes = roslaunch('--nodes', str(LAUNCH), *args).split()
    assert '/site_manager_node' in nodes and '/route_recorder_node' in nodes
    assert not [n for n in nodes if 'mapper' in n or 'planner' in n]
    expected = params['/health_monitor_node/nodes/expected']
    assert '/route_recorder_node' in expected and '/path_follower_node' in expected
    # an old odometry_only:= is no argument any more: roslaunch ignores it
    assert roslaunch('--nodes', str(LAUNCH), *args, 'odometry_only:=false').split() == nodes


def test_sites_dir_reaches_every_layer(tmp_path):
    layers = ('localization:=true', 'navigation:=true', 'inspection:=true')
    args = ('sim:=true', 'site:=lobby', f'sites_dir:={tmp_path}', *layers)
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    nodes = ('localization_node', 'site_manager_node', 'path_loader_node')
    for node in (*nodes, 'inspection_node'):
        assert params[f'/{node}/sites_dir'] == str(tmp_path), node
    assert params['/inspection_node/reports_dir'] == '/tmp/x30_sim/reports'
    assert (tmp_path / 'lobby' / 'tags.json').is_file()  # drawn while the launch file was read
    real = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *layers))
    assert real['/localization_node/sites_dir'] == '~/x30/map'  # the layers' own files
    assert real['/path_loader_node/sites_dir'] == '~/x30/map'
    assert real['/inspection_node/sites_dir'] == '~/x30/map'
    assert real['/inspection_node/reports_dir'] == '~/x30/reports'
    chosen = yaml.safe_load(
        roslaunch('--dump-params', str(LAUNCH), 'inspection:=true', 'reports_dir:=/data/r')
    )
    assert chosen['/inspection_node/reports_dir'] == '/data/r'


def test_the_controller_reaches_the_path_follower():
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), 'navigation:=true'))
    assert params['/path_follower_node/controller'] == 'holonomic'
    chosen = yaml.safe_load(
        roslaunch('--dump-params', str(LAUNCH), 'navigation:=true', 'controller:=neupan')
    )
    assert chosen['/path_follower_node/controller'] == 'neupan'
    assert chosen['/path_follower_node/neupan/dune_checkpoint'].endswith('models/dune_x30.pth')


HOSTS = Path(__file__).resolve().parents[1] / 'launch'


def test_the_navigation_host_runs_all_but_the_cameras_and_what_its_watchdog_watches():
    nodes = roslaunch('--nodes', str(HOSTS / 'navigation_host.launch')).split()
    for node in (
        '/x30/x30_driver',
        '/control_mux_node',
        '/lidar_sector_node',
        '/fast_lio/fast_lio',
        '/localization_node',
        '/site_manager_node',
        '/path_follower_node',
        '/health_monitor_node',
        '/inspection_node',
        '/teleop_udp_node',
    ):
        assert node in nodes, node
    assert not [n for n in nodes if n.startswith(('/hikmicro', '/hertzino'))]
    # no map, ever: the site and its routes, no grid mapper, no planner (it plans on a map)
    assert '/site_manager_node' in nodes and '/route_recorder_node' in nodes
    assert '/grid_mapper_node' not in nodes and '/planner_node' not in nodes
    script = (Path(__file__).resolve().parents[2] / 'tools' / 'x30_host.sh').read_text()
    critical = re.search(r'^CRITICAL=\((.*)\)$', script, re.MULTILINE).group(1).split()
    assert critical and set(critical) <= set(nodes)


def test_the_handheld_gateway_follows_the_robots_namespace_and_stays_out_of_health():
    args = ('bridge:=true', 'health:=true', 'namespace:=dog', 'sites_dir:=/data/sites')
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    assert params['/teleop_udp_node/robot_command_prefix'] == '/dog/'
    assert params['/teleop_udp_node/battery_topic'] == '/dog/battery'
    assert params['/teleop_udp_node/cmd_topic'] == '/dog/cmd_vel'
    assert params['/teleop_udp_node/sites_dir'] == '/data/sites'
    assert '/teleop_udp_node/token' not in params  # the secret: X30_TELEOP_TOKEN only
    assert '/teleop_udp_node' not in params['/health_monitor_node/nodes/expected']  # not driving
    assert '/teleop_udp_node' not in roslaunch('--nodes', str(LAUNCH)).split()
    navigation = str(HOSTS / 'navigation_host.launch')
    assert '/teleop_udp_node' not in roslaunch('--nodes', navigation, 'bridge:=false').split()


def test_the_netra_uplink_follows_the_robot_and_runs_with_a_netra_url():
    args = ('fleet:=true', 'namespace:=dog', 'sites_dir:=/data/sites',
            'netra_url:=https://netra.example.com', 'netra_robot_id:=x30-02')  # fmt: skip
    params = yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))
    assert params['/fleet_uplink_node/server_url'] == 'https://netra.example.com'
    assert params['/fleet_uplink_node/robot_id'] == 'x30-02'
    assert params['/fleet_uplink_node/sites_dir'] == '/data/sites'
    assert params['/fleet_uplink_node/battery_topic'] == '/dog/battery'
    assert '/fleet_uplink_node/token' not in params  # the token: X30_NETRA_TOKEN only
    assert '/fleet_uplink_node' not in roslaunch('--nodes', str(LAUNCH)).split()
    navigation = str(HOSTS / 'navigation_host.launch')
    assert '/fleet_uplink_node' not in roslaunch('--nodes', navigation).split()  # no NETRA set
    nodes = roslaunch('--nodes', navigation, 'netra_url:=https://netra.example.com').split()
    assert '/fleet_uplink_node' in nodes


def test_the_navigation_host_keeps_a_black_box_unless_told_not_to():
    launch = str(HOSTS / 'navigation_host.launch')
    command = roslaunch('--args', '/blackbox', launch, 'blackbox_dir:=/data/bb').split()
    assert command[0].endswith('x30_bringup/scripts/blackbox')
    assert command[1:4] == ['/data/bb', '10', '18']  # 10 min splits, 3 h kept
    assert '/blackbox' not in roslaunch('--nodes', launch, 'blackbox:=false').split()


def test_the_black_box_records_the_robot_in_rolling_splits(tmp_path):
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    (bin_dir / 'rosbag').write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    (bin_dir / 'rosbag').chmod(0o755)
    script = Path(__file__).resolve().parents[1] / 'scripts' / 'blackbox'
    env = {**os.environ, 'PATH': f'{bin_dir}:{os.environ["PATH"]}'}
    out = subprocess.run(
        [str(script), str(tmp_path / 'bb'), '10', '18', '__name:=blackbox'],
        capture_output=True, text=True, env=env, timeout=30, check=True,
    ).stdout.split()  # fmt: skip
    assert (tmp_path / 'bb').is_dir()
    assert out[:5] == ['record', '--lz4', '--split', '--duration=10m', '--max-splits=18']
    assert out[5:8] == ['-o', str(tmp_path / 'bb' / 'x30'), '__name:=blackbox']
    for topic in ('/navigation/status', '/cmd_vel', '/x30/odom', '/localization/pose'):
        assert topic in out


def test_the_navigation_host_passes_its_arguments(tmp_path):
    args = ('site:=gardu', 'controller:=neupan', 'inspection:=false')
    out = roslaunch('--dump-params', str(HOSTS / 'navigation_host.launch'), *args)
    params = yaml.safe_load(out)
    assert params['/path_follower_node/controller'] == 'neupan'
    assert params['/site_manager_node/site'] == 'gardu'
    assert params['/site_manager_node/create_site'] is True
    assert '/localization_node/assume_start_origin' not in params  # its default: the start
    assert '/inspection_node/sites_dir' not in params
    # the arguments of the map it had: an old service setting is no map either
    old = ('odometry_only:=false', 'mapping:=false')  # roslaunch ignores them
    nodes = roslaunch('--nodes', str(HOSTS / 'navigation_host.launch'), *old).split()
    assert '/grid_mapper_node' not in nodes and '/planner_node' not in nodes


def fake_driver(root: Path, name: str, node: str):
    """A package with a launch file like the driver's: one node, its host as a parameter."""
    package = root / name
    (package / 'launch').mkdir(parents=True)
    (package / 'package.xml').write_text(
        f'<package format="3"><name>{name}</name><version>0.0.0</version>'
        '<description>fake</description><maintainer email="a@b.c">a</maintainer>'
        '<license>Apache-2.0</license></package>'
    )
    namespace = name.split('_', maxsplit=1)[0]
    (package / 'launch' / f'{namespace}.launch').write_text(
        f'<launch><arg name="host" default=""/><arg name="password" default=""/>'
        f'<node pkg="{name}" type="{node}" name="{node}" ns="{namespace}">'
        '<param name="host" value="$(arg host)"/></node></launch>'
    )


def test_the_perception_host_runs_the_cameras_at_their_addresses_and_records_them(
    tmp_path, monkeypatch
):
    fake_driver(tmp_path, 'hikmicro_driver', 'hikmicro_ptz_node')
    fake_driver(tmp_path, 'hertzino_driver', 'hertzino_node')
    monkeypatch.setenv('ROS_PACKAGE_PATH', f'{tmp_path}:{os.environ["ROS_PACKAGE_PATH"]}')
    launch = str(HOSTS / 'perception_host.launch')
    nodes = roslaunch('--nodes', launch).split()
    assert nodes == ['/hikmicro/hikmicro_ptz_node', '/hertzino/hertzino_node', '/recorder_node']
    params = yaml.safe_load(roslaunch('--dump-params', launch, 'hikmicro_host:=10.0.0.9'))
    assert params['/hikmicro/hikmicro_ptz_node/host'] == '10.0.0.9'
    assert params['/hertzino/hertzino_node/host'] == '192.168.2.204'
    assert params['/recorder_node/video/host'] == '10.0.0.9'  # the camera it records
    assert params['/recorder_node/recordings_dir'] == '~/x30/recordings'
    moved = yaml.safe_load(roslaunch('--dump-params', launch, 'recordings_dir:=/data/rec'))
    assert moved['/recorder_node/recordings_dir'] == '/data/rec'
    assert roslaunch('--nodes', launch, 'hertzino:=false', 'recorder:=false').split() == [
        '/hikmicro/hikmicro_ptz_node'
    ]
