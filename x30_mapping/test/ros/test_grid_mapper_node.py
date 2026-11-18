"""In-process test of grid_mapper_node: scans through TF into a grid, save and load by site."""

from __future__ import annotations

import itertools

from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from x30_mapping.core import map_io, pgm
from x30_mapping.grid_mapper_node import GridMapperNode
from x30_mapping.utils.conversions import msg_grid
from x30_msgs.msg import MappingStatus
from x30_msgs.srv import LoadSite, StartMapping
from x30_utils.math import make_transform
from x30_utils.ros import X30Node
from x30_utils.ros.pointcloud import xyz_cloud
from x30_utils.testing.harness import Collector, delete_params, wait_for

BASE_HEIGHT = 0.48
BODY_IN_BASE = (0.13, 0.0, 0.015)  # FAST-LIO's body: the X30's IMU
WALL_X = 3.025  # the middle of a 5 cm cell: no float rounding moves it to the next one
NAMES = itertools.count()


def scan_points() -> np.ndarray:
    """A wall 3 m ahead (mid-cell), the floor and the robot's legs, in the body frame."""
    ys = np.arange(-1.5, 1.5, 0.02)
    wall = [(WALL_X, y, z) for y in ys for z in (0.1, 0.6)]  # z above the floor
    floor = [(x, 0.0, 0.0) for x in np.arange(1.0, 2.5, 0.05)]
    legs = [(0.2, 0.15, 0.15), (-0.2, -0.15, 0.15)]
    points = np.array(wall + floor + legs) - np.array(BODY_IN_BASE) - (0.0, 0.0, BASE_HEIGHT)
    return points.astype(np.float32)


def cell(grid: map_io.GridMap, x, y) -> int:
    resolution = round(grid.resolution, 6)
    col = int((x - grid.origin_x) // resolution)
    row = int((y - grid.origin_y) // resolution)
    return int(grid.data[row, col])


class Rig:
    """The mapper, a helper with TF and a scan source, in this process."""

    def __init__(self, sites_dir):
        self.ns = f'/grid_mapper_test_{next(NAMES)}'
        cloud_topic = f'{self.ns}/cloud_registered_body'
        self.node = GridMapperNode(
            param_ns=f'{self.ns}/grid_mapper_node',
            overrides={
                'sites_dir': str(sites_dir),
                'cloud_topic': cloud_topic,
                'rate_hz': 10.0,
                'publish_hz': 5.0,
                'robot.base_height': BASE_HEIGHT,
            },
        )
        self.helper = X30Node('helper', param_ns=f'{self.ns}/helper')
        self.helper.tf_out.send_static(
            [
                (make_transform((0.0, 0.0, 0.0)), 'map', 'odom'),
                (make_transform((0.0, 0.0, BASE_HEIGHT)), 'odom', 'base_link'),
                (make_transform(BODY_IN_BASE), 'base_link', 'body'),
            ]
        )
        self.grids = Collector(OccupancyGrid, 'mapping/grid')
        self.status = Collector(MappingStatus, 'mapping/status')
        clouds = self.helper.create_publisher(PointCloud2, cloud_topic, 5)
        points = scan_points()

        def publish():
            clouds.publish(xyz_cloud(Header(frame_id='body', stamp=self.helper.stamp()), points))

        self.helper.every(0.1, publish, steady=True)

    def call(self, srv, name, **request):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, srv)(**request)

    def start(self, site, reset=True):
        return self.call(StartMapping, 'grid_mapper/start', site=site, reset=reset)

    def stop(self):
        return self.call(Trigger, 'grid_mapper/stop')

    def save(self, site):
        return self.call(LoadSite, 'grid_mapper/save', site=site)

    def load(self, site):
        return self.call(LoadSite, 'grid_mapper/load', site=site)

    def close(self):
        self.grids.close()
        self.status.close()
        self.node.destroy_node()
        self.helper.destroy_node()
        delete_params(self.ns)


@pytest.fixture
def rig(ros, tmp_path):
    (tmp_path / 'lobby').mkdir()
    (tmp_path / 'hall').mkdir()
    r = Rig(tmp_path)
    yield r
    r.close()


def test_scans_become_a_grid_saved_into_its_own_site(rig, tmp_path):
    """Walls occupied, floor and legs not; save goes only into the mapped site."""
    assert not rig.save('lobby').success  # nothing mapped yet
    assert rig.start('lobby').success
    assert rig.status.wait(1)
    assert wait_for(lambda: rig.status.last.scans >= 4, 10.0), rig.status.last.message
    wall = lambda: rig.grids.last and cell(msg_grid(rig.grids.last), WALL_X, 0.025) == 100  # noqa: E731
    assert wait_for(wall, 5.0), debug(rig)
    grid = msg_grid(rig.grids.last)
    assert rig.grids.last.header.frame_id == 'map'
    assert cell(grid, 1.52, 0.02) == pgm.FREE  # floor below the band
    assert cell(grid, 0.22, 0.17) != pgm.OCCUPIED  # own legs
    assert rig.status.last.active and rig.status.last.site == 'lobby'

    refused = rig.save('hall')
    assert not refused.success and 'belongs to lobby' in refused.message
    assert not (tmp_path / 'hall' / 'map.yaml').exists()
    for bad in ('../lobby', '/tmp/x', ''):
        assert not rig.save(bad).success
    saved = rig.save('lobby')
    assert saved.success, saved.message
    assert map_io.load_map(str(tmp_path / 'lobby')).width > 0
    assert rig.stop().success
    assert wait_for(lambda: not rig.status.last.active, 3.0)


def test_continue_needs_the_site_loaded(rig, tmp_path):
    """A grid of one site is never continued as another; load takes site names only."""
    data = np.zeros((20, 30), np.int8)
    data[0, :] = 100
    map_io.save_map(str(tmp_path / 'hall'), map_io.GridMap(data, 0.05, -1.0, -1.0))
    assert not rig.start('hall', reset=False).success  # the grid holds no site yet
    assert not rig.load('missing').success
    assert not rig.load('../hall').success
    loaded = rig.load('hall')
    assert loaded.success, loaded.message
    assert rig.start('hall', reset=False).success
    assert not rig.load('lobby').success  # not while mapping
    assert rig.stop().success
    resaved = rig.save('hall')
    assert resaved.success
    assert (tmp_path / 'hall' / 'map.yaml.bak').exists()  # the previous map is kept


def debug(rig) -> str:
    """Describe the last grid around the wall (assertion message)."""
    if rig.grids.last is None:
        return 'no grid published'
    grid = msg_grid(rig.grids.last)
    near = [cell(grid, WALL_X + 0.05 * i, 0.025) for i in range(-2, 3)]
    s = rig.status.last
    return f'wall cells {near}, scans {s.scans}, rejected {s.rejected}: {s.message}'
