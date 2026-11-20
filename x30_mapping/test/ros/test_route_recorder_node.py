"""In-process test of route_recorder_node: localized only, site routes, stop points, OSError."""

from __future__ import annotations

import itertools
import math

import pytest
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from x30_mapping.route_recorder_node import RouteRecorderNode
from x30_msgs.msg import LocalizationStatus, SiteStatus
from x30_msgs.srv import MarkStopPoint
from x30_utils.io import read_route
from x30_utils.log import Sink
from x30_utils.math import planar_transform
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

NAMES = itertools.count()


class Rig:
    """The recorder, a helper with map -> base_link and the status topics, in this process."""

    def __init__(self, sites_dir):
        self.ns = f'/route_test_{next(NAMES)}'
        self.node = RouteRecorderNode(
            param_ns=f'{self.ns}/route_recorder_node',
            overrides={'sites_dir': str(sites_dir), 'min_spacing_m': 0.1, 'flush_sec': 0.2},
        )
        self.helper = X30Node('helper', param_ns=f'{self.ns}/helper')
        self.pose = [0.0, 0.0, 0.0]
        self.helper.every(
            0.05,
            lambda: self.helper.tf_out.send(planar_transform(*self.pose), 'map', 'base_link'),
            steady=True,
        )
        self.site_pub = self.helper.latched(SiteStatus, 'site/status')
        self.loc_pub = self.helper.latched(LocalizationStatus, 'localization/status')
        self.saved = Collector(String, 'route/saved')
        assert connected(self.site_pub, self.loc_pub, self.saved)

    def set(self, site=None, state=None):
        if site is not None:
            self.site_pub.publish(SiteStatus(site=site))
            assert wait_for(lambda: self.node.site == site, 3.0)
        if state is not None:
            self.loc_pub.publish(LocalizationStatus(state=state, source='tag:base_map'))
            localized = state == LocalizationStatus.LOCALIZED
            assert wait_for(lambda: self.node.localized == localized, 3.0)

    def trigger(self, name):
        rospy.wait_for_service(name, 5.0)
        return rospy.ServiceProxy(name, Trigger)()

    def mark(self, dwell):
        rospy.wait_for_service('route/mark_stop_point', 5.0)
        return rospy.ServiceProxy('route/mark_stop_point', MarkStopPoint)(dwell_sec=dwell)

    def close(self):
        self.saved.close()
        self.node.run_shutdown_hooks()
        self.node.destroy_node()
        self.helper.destroy_node()
        delete_params(self.ns)


class Warnings(Sink):
    """The warnings a node logs."""

    def __init__(self):
        super().__init__('warning')
        self.messages = []

    def write(self, record):
        self.messages.append(record.message)


@pytest.fixture
def rig(ros, tmp_path):
    r = Rig(tmp_path)
    yield r
    r.close()


def test_refuses_while_not_localized_even_with_tf(rig, tmp_path):
    """TF map->base exists, but localization is NONE: no recording, no stop point."""
    rig.set(site='lobby', state=LocalizationStatus.NONE)
    started = rig.trigger('route/start')
    assert not started.success and 'not localized' in started.message
    assert not rig.mark(5.0).success
    assert not (tmp_path / 'lobby' / 'routes').exists()


def test_records_into_the_site_with_stop_points(rig, tmp_path):
    rig.set(site='lobby', state=LocalizationStatus.LOCALIZED)
    assert wait_for(lambda: rig.node._pose() is not None, 3.0)
    started = rig.trigger('route/start')
    assert started.success, started.message
    for k in range(1, 6):
        rig.pose = [0.3 * k, 0.0, 0.1 * k]
        n = k
        assert wait_for(lambda n=n: len(rig.node.recorder.recording.points) >= n, 3.0)
    marked = rig.mark(3.0)
    assert marked.success, marked.message
    rig.set(state=LocalizationStatus.UNCERTAIN)  # samples pause while not LOCALIZED
    count = len(rig.node.recorder.recording.points)
    rig.pose = [5.0, 5.0, 0.0]
    assert not wait_for(lambda: len(rig.node.recorder.recording.points) > count, 0.5)
    stopped = rig.trigger('route/stop')
    assert stopped.success, stopped.message
    assert rig.saved.wait(1)
    path = rig.saved.last.data
    assert path.startswith(str(tmp_path / 'lobby' / 'routes'))
    points = read_route(path)
    assert len(points) == count and points[-1].is_stop and points[-1].dwell_sec == 3.0
    assert points[-1].yaw == pytest.approx(0.5, abs=0.01)
    assert all(math.hypot(p.x - 5.0, p.y - 5.0) > 1.0 for p in points)
    assert not rig.trigger('route/stop').success


def test_the_first_route_starts_without_a_tf_warning(rig):
    """route/start starts the TF listener: no TF yet is no warning, the samples follow."""
    warnings = Warnings()
    rig.node.log.sinks.append(warnings)
    rig.set(site='lobby', state=LocalizationStatus.LOCALIZED)
    started = rig.trigger('route/start')
    assert started.success, started.message
    rig.pose = [0.5, 0.0, 0.0]
    assert wait_for(lambda: len(rig.node.recorder.recording.points) >= 1, 3.0)
    assert not warnings.messages


def test_file_errors_are_answers_not_crashes(rig, tmp_path):
    """An OSError (routes/ is a file here) gives success=false; the node keeps serving."""
    (tmp_path / 'lobby').mkdir()
    (tmp_path / 'lobby' / 'routes').write_text('not a directory')
    rig.set(site='lobby', state=LocalizationStatus.LOCALIZED)
    started = rig.trigger('route/start')
    assert not started.success and 'cannot record' in started.message
    rig.set(site='hall')
    assert rig.trigger('route/start').success
