from __future__ import annotations

from dataclasses import dataclass
import itertools
import threading
import time

import numpy as np
import pytest
import rospy
from std_msgs.msg import Int32, String
from std_srvs.srv import Trigger

from x30_utils.config import ConfigError, nested, param
from x30_utils.math import make_transform, quat_from_yaw
from x30_utils.ros import X30Node
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

_ids = itertools.count()


@dataclass(frozen=True)
class Gains:
    kp: float = param(1.0, min=0.0, max=10.0)
    names: tuple[str, ...] = param(('a',), live=False)


@dataclass(frozen=True)
class Cfg:
    rate: float = param(5.0, min=0.1, max=50.0, unit='Hz', live=False)
    label: str = param('x', choices=('x', 'y'))
    gains: Gains = nested(Gains)


@pytest.fixture
def ns(ros):
    """A fresh parameter namespace (and topic prefix) per test."""
    name = f'/node_test_{next(_ids)}'
    yield name
    delete_params(name)


def make(ns: str, **overrides) -> X30Node:
    return X30Node('test_node', param_ns=ns, overrides=overrides)


def noted(node: X30Node, prefix: str) -> bool:
    """Return True once a parameter poll (the node's own or a test's) noted prefix."""
    return any(note.startswith(prefix) for note in node.param_notes)


def test_config_from_the_server_with_overrides(ns):
    rospy.set_param(ns, {'rate': 10, 'gains': {'kp': 2.5, 'names': ['p', 'q']}})
    node = make(ns, label='y')
    try:
        cfg = node.declare_config(Cfg).value
        assert cfg.rate == 10.0 and isinstance(cfg.rate, float)
        assert cfg.label == 'y'
        assert cfg.gains == Gains(2.5, ('p', 'q'))
    finally:
        node.destroy_node()


def test_invalid_values_refuse_to_start(ns):
    rospy.set_param(f'{ns}/gains/kp', 11.0)
    node = make(ns)
    try:
        with pytest.raises(ConfigError, match=r'gains\.kp'):
            node.declare_config(Cfg)
    finally:
        node.destroy_node()


def test_live_fields_follow_the_server_and_bad_ones_are_kept_out(ns):
    node = make(ns)
    try:
        config = node.declare_config(Cfg)
        seen = []
        config.on_change(lambda old, new: seen.append(new.gains.kp))
        rospy.set_param(f'{ns}/gains/kp', 3.0)
        assert wait_for(lambda: config.gains.kp == 3.0, 3.0)
        rospy.set_param(f'{ns}/gains/kp', 99.0)  # out of range: rejected
        rospy.set_param(f'{ns}/rate', 20.0)  # read-only after start: ignored
        time.sleep(2.5)
        assert config.gains.kp == 3.0 and config.rate == 5.0
        assert seen == [3.0]
    finally:
        node.destroy_node()


def test_robot_profile_is_searched_up_from_the_node(ns):
    rospy.set_param('/robot', {'name': 'x30', 'footprint': {'front': 0.55}, 'kinematics': 'omni'})
    try:
        node = make(ns)
        robot = node.declare_robot().value
        assert robot.name == 'x30' and robot.footprint.front == 0.55
        node.destroy_node()
    finally:
        delete_params('/robot')


def test_robot_profile_is_read_only_and_the_nearest_one_wins(ns):
    rospy.set_param('/robot', {'name': 'far'})
    rospy.set_param(f'{ns}/robot', {'name': 'near', 'limits': {'max_vx': 0.6}})
    node = make(ns)
    try:
        robot = node.declare_robot()
        assert robot.name == 'near' and robot.limits.max_vx == 0.6
        rospy.set_param(f'{ns}/robot/limits/max_vx', 2.0)
        notes = node._binder.poll()
        assert notes == ['ignored, read-only until restart: robot.limits.max_vx']
        assert robot.limits.max_vx == 0.6
    finally:
        node.destroy_node()
        delete_params('/robot')


def test_unknown_parameters_are_reported(ns):
    rospy.set_param(ns, {'rate': 5.0, 'raet': 1.0})
    node = make(ns)
    try:
        node.declare_config(Cfg)
        assert node._binder.unknown() == ['raet']
    finally:
        node.destroy_node()


def test_latched_publisher_reaches_a_late_subscriber(ns):
    node = make(ns)
    try:
        pub = node.latched(String, f'{ns}/latched')
        pub.publish(String(data='first'))
        late = Collector(String, f'{ns}/latched')
        assert late.wait_until(lambda m: m.data == 'first')
        late.close()
    finally:
        node.destroy_node()


def test_callbacks_never_overlap(ns):
    node = make(ns)
    running, overlap, calls = [0], [0], [0]
    lock = threading.Lock()

    def busy(*_args):
        with lock:
            running[0] += 1
            overlap[0] = max(overlap[0], running[0])
        time.sleep(0.01)
        with lock:
            running[0] -= 1
            calls[0] += 1

    try:
        node.create_subscription(Int32, f'{ns}/a', busy, 10)
        node.create_subscription(Int32, f'{ns}/b', busy, 10)
        node.every(0.005, busy, steady=True)
        pa = rospy.Publisher(f'{ns}/a', Int32, queue_size=10)
        pb = rospy.Publisher(f'{ns}/b', Int32, queue_size=10)
        assert connected(pa, pb)
        for i in range(20):
            pa.publish(Int32(data=i))
            pb.publish(Int32(data=i))
            time.sleep(0.005)
        assert wait_for(lambda: calls[0] > 40, 5.0)
        assert overlap[0] == 1
    finally:
        node.destroy_node()


def test_timer_errors_are_logged_or_handed_over_and_the_timer_goes_on(ns):
    node = make(ns)
    ticks, errors = [0], []

    def failing():
        ticks[0] += 1
        raise RuntimeError('boom')

    try:
        node.every(0.01, failing, steady=True, on_error=errors.append)
        node.every(0.01, failing)  # ROS time, logged
        assert wait_for(lambda: ticks[0] > 10 and len(errors) > 3, 3.0)
        assert all(isinstance(e, RuntimeError) for e in errors)
    finally:
        node.destroy_node()


def test_a_refused_read_only_value_never_comes_in_with_a_later_change(ns):
    node = make(ns)
    try:
        config = node.declare_config(Cfg)
        rospy.set_param(f'{ns}/rate', 20.0)  # read-only
        assert wait_for(lambda: noted(node, 'ignored, read-only'), 3.0)
        rospy.set_param(f'{ns}/gains/kp', 3.0)  # live, while rate=20 stays on the server
        assert wait_for(lambda: noted(node, 'applied'), 3.0)
        assert config.gains.kp == 3.0 and config.rate == 5.0
    finally:
        node.destroy_node()


def test_the_effective_values_go_back_to_the_server(ns):
    """rosparam get shows what the node runs with: defaults and overrides included."""
    rospy.set_param(f'{ns}/rate', 25)
    node = make(ns, label='y')
    try:
        node.declare_config(Cfg)
        assert rospy.get_param(f'{ns}/rate') == 25
        assert rospy.get_param(f'{ns}/label') == 'y'
        assert rospy.get_param(f'{ns}/gains/names') == list(Cfg().gains.names)
        time.sleep(1.5)  # a poll or two
        assert node.param_notes == []  # nothing changed by writing them back
    finally:
        node.destroy_node()


def test_services_answer_and_errors_fail_the_call(ns):
    node = make(ns)

    def ok(_request, response):
        response.success, response.message = True, 'done'
        return response

    def broken(_request, _response):
        raise ValueError('nope')

    try:
        node.create_service(Trigger, f'{ns}/ok', ok)
        node.create_service(Trigger, f'{ns}/broken', broken)
        rospy.wait_for_service(f'{ns}/ok', 5.0)
        rospy.wait_for_service(f'{ns}/broken', 5.0)
        answer = rospy.ServiceProxy(f'{ns}/ok', Trigger)()
        assert answer.success and answer.message == 'done'
        with pytest.raises(rospy.ServiceException):
            rospy.ServiceProxy(f'{ns}/broken', Trigger)()
    finally:
        node.destroy_node()


def test_any_subscription_passes_bytes_and_the_definition(ns):
    node = make(ns)
    got = []
    pub = rospy.Publisher(f'{ns}/any', String, queue_size=10)
    try:
        node.create_any_subscription(f'{ns}/any', lambda data, c: got.append((data, c)))
        assert connected(pub)
        pub.publish(String('hi'))
        assert wait_for(lambda: got)
        data, connection = got[0]
        assert data == b'\x02\x00\x00\x00hi'  # uint32 length, then the bytes
        assert connection['type'] == 'std_msgs/String'
        assert 'string data' in connection['message_definition']
    finally:
        pub.unregister()
        node.destroy_node()


def test_a_destroyed_subscription_delivers_nothing_more(ns):
    node = make(ns)
    got = []
    pub = rospy.Publisher(f'{ns}/data', Int32, queue_size=10)
    try:
        sub = node.create_subscription(Int32, f'{ns}/data', lambda m: got.append(m.data), 10)
        assert connected(pub)
        pub.publish(Int32(1))
        assert wait_for(lambda: got == [1])
        node.destroy_subscription(sub)
        assert wait_for(lambda: pub.get_num_connections() == 0)
        pub.publish(Int32(2))
        time.sleep(0.3)
        assert got == [1]
    finally:
        pub.unregister()
        node.destroy_node()


def test_a_concurrent_service_can_wait_for_the_timers(ns):
    """A service that waits for the timers (a recording they finish) runs beside them."""
    node = make(ns)
    ticked = threading.Event()

    def wait_for_a_tick(_request, response):
        ticked.clear()
        response.success = ticked.wait(3.0)
        return response

    try:
        node.every(0.05, ticked.set)
        node.create_service(Trigger, f'{ns}/waits', wait_for_a_tick, concurrent=True)
        rospy.wait_for_service(f'{ns}/waits', 5.0)
        assert rospy.ServiceProxy(f'{ns}/waits', Trigger)().success
    finally:
        node.destroy_node()


def test_raw_subscription_passes_bytes_and_drops_other_types(ns):
    node = make(ns)
    got = []
    try:
        node.create_subscription(String, f'{ns}/raw', got.append, raw=True)
        node.create_subscription('std_msgs/String', f'{ns}/wrong', got.append, raw=True)
        pub = rospy.Publisher(f'{ns}/raw', String, queue_size=1)
        wrong = rospy.Publisher(f'{ns}/wrong', Int32, queue_size=1)
        assert connected(pub, wrong)
        wrong.publish(Int32(data=7))
        pub.publish(String(data='hi'))
        assert wait_for(lambda: got, 3.0)
        time.sleep(0.2)
        assert got == [b'\x02\x00\x00\x00hi']
    finally:
        node.destroy_node()


def test_publish_after_destroy_is_dropped(ns):
    node = make(ns)
    pub = node.create_publisher(String, f'{ns}/gone')
    node.destroy_node()
    pub.publish(String(data='late'))  # no exception
    assert node.destroyed


def test_shutdown_hooks_run_once_and_survive_failures(ns):
    node = make(ns)
    calls = []
    node.add_shutdown_hook(lambda: calls.append(1))
    node.add_shutdown_hook(lambda: 1 / 0)
    node.add_shutdown_hook(lambda: calls.append(2))
    node.run_shutdown_hooks()
    node.run_shutdown_hooks()
    node.destroy_node()
    assert calls == [1, 2]


def test_static_transforms_of_nodes_in_one_process_all_stay_latched(ns):
    """rospy latches one /tf_static message per process: it carries every node's set."""
    first, second = make(ns), X30Node('test_node_2', param_ns=f'{ns}_2')
    late = None
    try:
        first.tf_out.send_static([(make_transform((1.0, 0.0, 0.0)), 'base_link', f'{ns[1:]}_a')])
        second.tf_out.send_static([(make_transform((0.0, 2.0, 0.0)), 'base_link', f'{ns[1:]}_b')])
        late = X30Node('test_node_late', param_ns=f'{ns}_late')  # listens after both sent
        assert wait_for(lambda: late.tf.static('base_link', f'{ns[1:]}_a') is not None, 5.0)
        assert wait_for(lambda: late.tf.static('base_link', f'{ns[1:]}_b') is not None, 5.0)
        assert late.tf.static('base_link', f'{ns[1:]}_b')[1, 3] == pytest.approx(2.0)
    finally:
        for node in (first, second, late):
            if node is not None:
                node.destroy_node()


def test_tf_static_and_dynamic_roundtrip(ns):
    node = make(ns)
    try:
        mount = make_transform((0.4, 0.0, 0.1), quat_from_yaw(0.5))
        node.tf_out.send_static([(mount, 'base_link', f'{ns[1:]}_lidar')])
        assert wait_for(lambda: node.tf.static('base_link', f'{ns[1:]}_lidar') is not None, 5.0)
        assert np.allclose(node.tf.static('base_link', f'{ns[1:]}_lidar'), mount)
        odom = make_transform((1.0, 2.0, 0.0), quat_from_yaw(-0.3))

        def heard() -> bool:  # /tf is not latched: a broadcaster sends until the listener hears
            node.tf_out.send(odom, f'{ns[1:]}_odom', 'base_link')
            return node.tf.fresh(f'{ns[1:]}_odom', 'base_link', 1.0)

        assert wait_for(heard, 5.0, period=0.05)
        result = node.tf.lookup(f'{ns[1:]}_odom', 'base_link')
        assert result.pose2d.x == pytest.approx(1.0) and result.pose2d.yaw == pytest.approx(-0.3)
        assert node.tf.lookup('nowhere', 'base_link') is None and node.tf.error
    finally:
        node.destroy_node()
