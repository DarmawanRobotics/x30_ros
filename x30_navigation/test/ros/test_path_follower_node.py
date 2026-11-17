"""path_follower_node in-process: commands on control/nav_cmd_vel, pause reasons, zeros."""

from __future__ import annotations

import itertools
import threading
import time

from geometry_msgs.msg import Twist
import pytest
from ros_rig import World, call, set_param
import rospy
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, NavigationStatus, Waypoint
from x30_msgs.srv import SetPath
from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.path_follower_node import PathFollowerNode
from x30_utils.testing.harness import Collector, delete_params, wait_for

S = NavigationStatus
NAMES = itertools.count()


def line(n=60):
    return [Waypoint(x=0.1 * i, y=0.0, yaw=0.0, dwell_sec=0.0) for i in range(n)]


def moving(cmds) -> bool:
    last = cmds.last
    return last is not None and abs(last.linear.x) > 0.05


def neutral(m) -> bool:
    return m.linear.x == m.linear.y == m.angular.z == 0.0


def zero(cmds) -> bool:
    last = cmds.last
    return last is not None and neutral(last)


class Rig:
    """The follower, the fake world and the collectors, in this process."""

    def __init__(self, hidden=False, **params):
        self.ns = f'/follower_test_{next(NAMES)}'
        overrides = {'require_localization': False, 'control_node': '', **params}
        self.follower = PathFollowerNode(param_ns=f'{self.ns}/node', overrides=overrides)
        self.world = World()
        self.world.hidden = hidden
        self.cmds = Collector(Twist, 'control/nav_cmd_vel')
        self.status = Collector(NavigationStatus, 'navigation/status')

    def close(self):
        self.cmds.close()
        self.status.close()
        self.follower.destroy_node()
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


def test_follow_pause_estop_resume_and_shutdown(rig):
    """nav_cmd_vel commands, zero on every hold, neutral until armed, zeros at shutdown."""
    r = rig()
    cmds, status = r.cmds, r.status
    assert call(SetPath, 'navigation/set_path', waypoints=line()).success
    assert not call(SetPath, 'navigation/set_path', waypoints=line(1)).success
    assert wait_for(lambda: r.follower.pose_input() is not None, 5.0)
    assert call(Trigger, 'navigation/start').success
    assert wait_for(lambda: moving(cmds), 5.0), status.last
    assert status.last.state == S.FOLLOWING
    assert call(Trigger, 'navigation/pause').success
    assert wait_for(lambda: zero(cmds) and status.last.state == S.PAUSED, 2.0)
    assert call(Trigger, 'navigation/resume').success
    assert wait_for(lambda: moving(cmds), 3.0)
    control = rospy.Publisher('control/status', ControlStatus, queue_size=1, latch=True)
    try:
        control.publish(ControlStatus(estop=True, armed=False))
        assert wait_for(lambda: list(status.last.pause_reasons) == ['estop'], 2.0)
        assert zero(cmds)
        assert not call(Trigger, 'navigation/resume').success
        control.publish(ControlStatus(estop=False, armed=False))  # released, not re-armed yet
        assert wait_for(lambda: call(Trigger, 'navigation/resume').success, 2.0)
        assert wait_for(lambda: list(status.last.pause_reasons) == ['disarmed'], 2.0)
        count = len(cmds.messages)
        assert wait_for(lambda: len(cmds.messages) >= count + 3, 2.0)
        assert all(neutral(m) for m in cmds.messages[count:])  # they arm navigation in the mux
        control.publish(ControlStatus(estop=False, armed=True))
        assert wait_for(lambda: moving(cmds), 3.0)
    finally:
        control.unregister()
    r.world.blind = True
    assert wait_for(lambda: zero(cmds) and 'SCAN_BLIND' in status.last.message, 2.0)
    r.world.blind = False
    assert call(Trigger, 'navigation/stop').success
    assert wait_for(lambda: zero(cmds) and status.last.state == S.INACTIVE, 2.0)
    count = len(cmds.messages)
    r.follower.run_shutdown_hooks()
    assert wait_for(lambda: len(cmds.messages) >= count + 1, 2.0) and zero(cmds)


@pytest.mark.parametrize(
    ('name', 'value'),
    [('speed/min_speed', 5.0), ('speed/accel', -1.0), ('controller', 'nope')],
)
def test_invalid_parameters_are_refused(rig, name, value):
    """Out-of-range or inconsistent values are refused, the old ones are kept."""
    r = rig()
    assert set_param(r.follower, name, value, 'rejected')
    assert r.follower.config.speed.min_speed == FollowerConfig().speed.min_speed
    assert r.follower.follower.controller_name == 'holonomic'


def test_read_only_safety_and_a_live_controller(rig):
    r = rig()
    assert set_param(r.follower, 'safety/tf_timeout_sec', 2.0, 'ignored, read-only')
    assert set_param(r.follower, 'controller', 'pid')  # the read-only value is not taken along
    assert r.follower.follower.controller_name == 'pid'
    assert r.follower.config.safety.tf_timeout_sec == FollowerConfig().safety.tf_timeout_sec


def test_start_waits_for_the_robot_pose(rig):
    """start right after LOCALIZED: the pose reaches the follower late; the loop keeps running."""
    r = rig(hidden=True)
    assert call(SetPath, 'navigation/set_path', waypoints=line()).success
    answer = []
    starting = threading.Thread(target=lambda: answer.append(call(Trigger, 'navigation/start')))
    starting.start()
    time.sleep(1.0)
    assert not answer, 'refused without waiting for the pose'
    assert call(Trigger, 'navigation/pause').success, 'state services blocked'
    assert call(Trigger, 'navigation/resume').success
    r.world.hidden = False
    starting.join(5.0)
    assert answer and answer[0].success, answer
    assert wait_for(lambda: moving(r.cmds), 5.0), r.status.last


def test_start_refused_after_the_pose_wait(rig):
    """No pose within pose_wait_sec: refused with the reason."""
    rig(hidden=True, pose_wait_sec=0.5)
    assert call(SetPath, 'navigation/set_path', waypoints=line()).success
    began = time.monotonic()
    reply = call(Trigger, 'navigation/start')
    assert 0.45 <= time.monotonic() - began < 3.0
    assert not reply.success and 'robot pose unknown' in reply.message
