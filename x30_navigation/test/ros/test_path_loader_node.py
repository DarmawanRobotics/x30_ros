"""path_loader_node in-process: routes inside the sites directory reach the follower."""

from __future__ import annotations

import math

from ros_rig import call

from x30_msgs.msg import WaypointPath
from x30_msgs.srv import LoadPath
from x30_navigation.path_follower_node import PathFollowerNode
from x30_navigation.path_loader_node import PathLoaderNode
from x30_utils.io import RoutePoint, write_route
from x30_utils.testing.harness import Collector, delete_params, wait_for


def test_load_route_inside_the_sites_dir(ros, tmp_path):
    """Stop points 2 cm apart survive; a path outside sites_dir is refused."""
    sites = tmp_path / 'sites'
    route = [RoutePoint(0.2 * i, 0.0) for i in range(20)]
    route[8] = RoutePoint(1.6, 0.0, math.pi / 2, 3.0)
    route.insert(9, RoutePoint(1.62, 0.0, 0.0, 2.0))
    write_route(str(sites / 'lab' / 'routes' / 'a.csv'), route)
    write_route(str(tmp_path / 'outside.csv'), route)
    loader = PathLoaderNode(
        param_ns='/loader_test/loader',
        overrides={'sites_dir': str(sites), 'remember_file': str(tmp_path / 'last')},
    )
    follower = PathFollowerNode(param_ns='/loader_test/follower', overrides={'control_node': ''})
    paths = Collector(WaypointPath, 'navigation/waypoints')
    try:
        result = call(LoadPath, 'navigation/load_path', path='lab/routes/a.csv')
        assert result.success, result.message
        assert wait_for(lambda: paths.last is not None, 3.0)
        stops = [w for w in paths.last.waypoints if w.dwell_sec > 0.0]
        assert [(w.x, w.dwell_sec) for w in stops] == [(1.6, 3.0), (1.62, 2.0)]
        assert stops[0].yaw == math.pi / 2
        assert wait_for(lambda: follower.follower.tracker is not None, 3.0)
        assert len(follower.follower.tracker.route) == len(paths.last.waypoints)
        assert (tmp_path / 'last').read_text().strip().endswith('a.csv')
        outside = call(LoadPath, 'navigation/load_path', path='../outside.csv')
        assert not outside.success and 'outside' in outside.message
        missing = call(LoadPath, 'navigation/load_path', path='lab/none.csv')
        assert not missing.success
    finally:
        paths.close()
        loader.destroy_node()
        follower.destroy_node()
        delete_params('/loader_test')
