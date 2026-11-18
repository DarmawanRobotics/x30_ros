"""sim.launch: the defaults, the arguments, a site drawn while the launch file is read."""

from __future__ import annotations

from pathlib import Path
import subprocess

import yaml

from x30_utils.io import SiteLayout, read_route, read_tags

LAUNCH = Path(__file__).resolve().parents[2] / 'launch' / 'sim.launch'


def roslaunch(*args: str) -> str:
    result = subprocess.run(
        ['roslaunch', *args], capture_output=True, text=True, timeout=60, check=False
    )
    return result.stdout + result.stderr


def params(*args: str) -> dict:
    return yaml.safe_load(roslaunch('--dump-params', str(LAUNCH), *args))


def test_the_node_reads_the_defaults_and_nothing_is_drawn():
    p = params()
    assert p['/sim_robot_node/driver_ns'] == 'x30'
    assert p['/sim_robot_node/imu/frame'] == 'imu_link'
    assert p['/sim_robot_node/lidar/mounts'][:5] == [0.44367, 0.0, 0.05415, 0.0, 25.0]
    assert p['/sim_robot_node/world_file'] == ''
    assert '/sim/site_dir' not in p and '/robot/base_height' not in p
    assert roslaunch('--nodes', str(LAUNCH)).split() == ['/sim_robot_node']


def test_arguments_override_the_files(tmp_path):
    robot = tmp_path / 'robot.yaml'
    robot.write_text('base_height: 0.5\n')
    extra = tmp_path / 'extra.yaml'
    extra.write_text('seed: 7\nlidar: {rings: 8}\n')
    p = params(
        f'robot_params:={robot}', f'params_file:={extra}', 'world:=mall_loop', 'scenario:=drift'
    )
    assert p['/robot/base_height'] == 0.5
    assert p['/sim_robot_node/seed'] == 7 and p['/sim_robot_node/lidar/rings'] == 8
    assert p['/sim_robot_node/lidar/rate_hz'] == 10.0  # the rest of lidar/ stays
    assert p['/sim_robot_node/world_file'] == 'mall_loop'
    assert p['/sim_robot_node/scenario_file'] == 'drift'


def test_site_is_drawn_while_the_launch_file_is_read(tmp_path):
    p = params('site:=lobby', f'sites_dir:={tmp_path}')
    layout = SiteLayout(str(tmp_path), 'lobby')
    assert p['/sim/site_dir'].strip() == layout.root
    assert Path(layout.map_yaml).is_file() and Path(layout.map_image).is_file()
    tags, problems = read_tags(layout.tags)
    assert set(tags) == {'base_map', 'charging_dock'} and not problems
    route = read_route(str(Path(layout.routes) / 'mall_loop.csv'))
    assert len(route) > 100 and any(point.dwell_sec > 0.0 for point in route)


def test_a_bad_world_fails_the_launch(tmp_path):
    out = roslaunch('--dump-params', str(LAUNCH), 'site:=lobby', 'world:=atlantis',
                    f'sites_dir:={tmp_path}')  # fmt: skip
    assert 'atlantis' in out and not (tmp_path / 'lobby').exists()
