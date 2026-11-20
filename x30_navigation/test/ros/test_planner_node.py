"""planner_node in-process against a fake follower on a room with a door."""

from __future__ import annotations

import itertools
import math
import threading
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
import pytest
from ros_rig import World, call, set_param
import rospy
from std_srvs.srv import Trigger

from x30_msgs.msg import NavigationStatus, PlannerStatus
from x30_msgs.srv import GoTo, PlanPath, SetPath
from x30_navigation.planner_node import PlannerNode
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, delete_params, wait_for

P = PlannerStatus
INACTIVE = NavigationStatus.INACTIVE
NAMES = itertools.count()


def room_map() -> OccupancyGrid:
    """A 10 x 6 m room: a wall at x = 5 with a 1.2 m door at y = 0.5 .. 1.7."""
    data = np.zeros((120, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    data[:, 100] = 100
    data[10:34, 100] = 0
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = 0.05
    msg.info.width, msg.info.height = 200, 120
    msg.info.origin.orientation.w = 1.0
    msg.data = data.ravel().tolist()
    return msg


class FakeFollower:
    """navigation/set_path, start, stop and a navigation/status the test steers."""

    def __init__(self, ns, delay=0.0):
        self.node = X30Node('fake_follower', param_ns=f'{ns}/fake_follower')
        self.delay = delay
        self.paths, self.calls = [], []
        self.state = NavigationStatus.INACTIVE
        self.status = self.node.latched(NavigationStatus, 'navigation/status')
        self.node.create_service(SetPath, 'navigation/set_path', self.set_path, concurrent=True)
        for name in ('start', 'stop'):
            self.node.create_service(
                Trigger, f'navigation/{name}', self.trigger(name), concurrent=True
            )
        self.node.every(
            0.05, lambda: self.status.publish(NavigationStatus(state=self.state)), steady=True
        )

    def set_path(self, request, response):
        time.sleep(self.delay)
        self.paths.append(list(request.waypoints))
        response.success = True
        return response

    def trigger(self, name):
        def handle(_request, response):
            self.calls.append(name)
            self.state = NavigationStatus.FOLLOWING if name == 'start' else INACTIVE
            response.success = True
            return response

        return handle


class Rig:
    """The planner, the fake world and follower, /map and the status, in this process."""

    def __init__(self, delay=0.0, hidden=False):
        self.ns = f'/planner_test_{next(NAMES)}'
        self.planner = PlannerNode(param_ns=f'{self.ns}/planner')
        self.world = World(pose=(2.0, 4.0, 0.0))
        self.world.hidden = hidden
        self.follower = FakeFollower(self.ns, delay)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1, latch=True)
        self.map_pub.publish(room_map())
        self.status = Collector(PlannerStatus, 'planner/status')
        assert wait_for(lambda: self.planner.grid is not None, 5.0)
        assert hidden or wait_for(lambda: self.planner.robot_pose() is not None, 5.0)

    def set(self, name, value):
        assert set_param(self.planner, name, value)

    def go_to_in_background(self, **goal):
        answer = []
        thread = threading.Thread(
            target=lambda: answer.append(call(GoTo, 'planner/go_to', **goal))
        )
        thread.start()
        return thread, answer

    def last(self):
        return self.status.last.state if self.status.last else None

    def close(self):
        self.status.close()
        self.map_pub.unregister()
        self.planner.destroy_node()
        self.follower.node.destroy_node()
        self.world.close()
        delete_params(self.ns)


@pytest.fixture
def rig(ros):
    made = []

    def make(**kwargs):
        made.append(Rig(**kwargs))
        return made[-1]

    yield make
    for r in made:
        r.close()


def test_go_to_arrives_on_the_plan_goal_and_stops(rig):
    """Arrival against the plan goal stops the follower."""
    r = rig()
    result = call(GoTo, 'planner/go_to', x=8.0, y=4.0, yaw=math.pi / 2)
    assert result.success, result.message
    path = r.follower.paths[-1]
    assert any(4.9 < w.x < 5.1 and 0.5 < w.y < 1.7 for w in path), 'not through the door'
    assert path[-1].yaw == math.pi / 2 and r.follower.calls == ['start']
    r.follower.state = NavigationStatus.GOAL_REACHED
    time.sleep(0.3)
    assert r.last() == P.FOLLOWING
    r.world.pose = (8.0, 4.0, math.pi / 2)
    assert wait_for(lambda: r.last() == P.ARRIVED, 3.0)
    assert wait_for(lambda: r.follower.calls[-1] == 'stop', 2.0)
    preview = call(PlanPath, 'planner/plan', x=2.0, y=4.0, yaw=0.0)
    assert preview.success and len(r.follower.paths) == 1
    assert r.planner.costmap is not None
    r.set('costmap/margin', 0.2)
    assert r.planner.costmap is None  # rebuilt with the new parameters
    off = call(GoTo, 'planner/go_to', x=20.0, y=4.0, yaw=0.0)
    assert not off.success and 'outside the map' in off.message
    assert not call(GoTo, 'planner/go_to', x=math.nan, y=0.0, yaw=0.0).success


def test_blocked_replans_and_blind_scan_fails(rig):
    """A blind safety scan is never read as clear when replanning: the goal fails."""
    r = rig()
    assert call(GoTo, 'planner/go_to', x=8.0, y=2.0, yaw=0.0).success
    r.follower.state = NavigationStatus.BLOCKED
    assert wait_for(lambda: len(r.follower.paths) == 2, 6.0), 'no replan'
    r.follower.state = NavigationStatus.FOLLOWING
    r.world.blind = True
    time.sleep(0.3)
    r.follower.state = NavigationStatus.BLOCKED
    assert wait_for(lambda: r.last() == P.FAILED, 6.0)
    assert 'blind' in r.status.last.message
    assert wait_for(lambda: r.follower.calls[-1] == 'stop', 3.0)


def test_cancel_is_not_blocked_by_a_slow_send(rig):
    """Cancel answers at once while go_to waits for the follower, and nothing starts."""
    r = rig(delay=2.0)
    thread, answer = r.go_to_in_background(x=8.0, y=2.0, yaw=0.0)
    assert wait_for(lambda: r.last() == P.PLANNING, 3.0)
    time.sleep(0.5)
    began = time.monotonic()
    assert call(Trigger, 'planner/cancel').success
    assert time.monotonic() - began < 1.0
    thread.join(5.0)
    assert answer and not answer[0].success and 'start' not in r.follower.calls
    assert r.status.wait_until(lambda m: m.state == P.CANCELLED, 2.0), r.last()


def test_a_site_without_a_map_leaves_nothing_to_plan_on(rig):
    """The empty /map of a site without one (odometry only): go_to refused until a map comes."""
    r = rig()
    r.map_pub.publish(OccupancyGrid())  # what site_manager_node sends for such a site
    assert wait_for(lambda: r.planner.grid is None, 5.0)
    answer = call(GoTo, 'planner/go_to', x=8.0, y=4.0, yaw=0.0)
    assert not answer.success and 'no map' in answer.message
    assert not r.follower.calls
    r.map_pub.publish(room_map())
    assert wait_for(lambda: r.planner.grid is not None, 5.0)


def test_go_to_waits_for_the_robot_pose(rig):
    """go_to right after LOCALIZED: the pose comes late; services and status keep running."""
    r = rig(hidden=True)
    thread, answer = r.go_to_in_background(x=8.0, y=4.0, yaw=0.0)
    assert wait_for(lambda: r.last() == P.PLANNING, 3.0)
    received = len(r.status.messages)
    time.sleep(1.0)
    assert not answer, 'refused without waiting for the pose'
    assert len(r.status.messages) > received, 'status timer blocked while waiting'
    r.world.hidden = False
    thread.join(5.0)
    assert answer and answer[0].success, answer
    assert r.follower.calls == ['start']
    # the status is a topic of its own: it may come after the answer
    assert r.status.wait_until(lambda m: m.state == P.FOLLOWING, 2.0), r.last()


def test_plan_refused_after_the_pose_wait_and_cancel_ends_the_wait(rig):
    """No pose within pose_wait_sec: refused; planner/cancel ends a go_to waiting for it."""
    r = rig(hidden=True)
    r.set('pose_wait_sec', 0.5)
    began = time.monotonic()
    plan = call(PlanPath, 'planner/plan', x=8.0, y=4.0, yaw=0.0)
    assert 0.45 <= time.monotonic() - began < 3.0
    assert not plan.success and 'robot pose unknown' in plan.message
    assert 'within 0.5 s' in plan.message
    r.set('pose_wait_sec', 10.0)
    thread, answer = r.go_to_in_background(x=8.0, y=4.0, yaw=0.0)
    assert wait_for(lambda: r.last() == P.PLANNING, 3.0)
    began = time.monotonic()
    assert call(Trigger, 'planner/cancel').success
    thread.join(2.0)
    assert answer and not answer[0].success and time.monotonic() - began < 2.0
    assert r.status.wait_until(lambda m: m.state == P.CANCELLED, 2.0), r.last()
    assert not r.follower.paths and 'start' not in r.follower.calls
