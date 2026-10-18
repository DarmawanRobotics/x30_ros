#!/usr/bin/env python3
"""
The driver node (driver.py) against the simulated robot over UDP, without a ROS master.

rospy and the message modules are replaced by the stand-ins in rosstub/ for this module only;
the real ROS path is covered by smoke.test. Each test starts its own simulated robot and node.
"""

import importlib
import os
import sys
import threading
import time
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
STUBS = os.path.join(HERE, 'rosstub')
sys.path.insert(0, os.path.join(PACKAGE, 'src'))
sys.path.insert(0, HERE)  # nose -P
from fake_robot import AXES, FakePerception, FakeRobotServer, FakeX30  # noqa: E402
from x30_driver import protocol as p  # noqa: E402

STUB_MODULES = ('rospy', 'rosstub_msg', 'geometry_msgs', 'geometry_msgs.msg', 'nav_msgs',
                'nav_msgs.msg', 'sensor_msgs', 'sensor_msgs.msg', 'std_srvs', 'std_srvs.srv',
                'diagnostic_msgs', 'diagnostic_msgs.msg', 'x30_driver.driver')
_saved = {}
rospy = driver = Twist = None


def setUpModule():  # noqa: N802 (unittest's name)
    """Import the node with the stand-ins in place of rospy and the messages."""
    global rospy, driver, Twist
    for name in STUB_MODULES:
        if name in sys.modules:
            _saved[name] = sys.modules.pop(name)
    sys.path.insert(0, STUBS)
    rospy = importlib.import_module('rospy')
    driver = importlib.import_module('x30_driver.driver')
    Twist = importlib.import_module('geometry_msgs.msg').Twist


def tearDownModule():  # noqa: N802 (unittest's name)
    """Put the real modules back."""
    for name in STUB_MODULES:
        sys.modules.pop(name, None)
    sys.modules.update(_saved)
    sys.path.remove(STUBS)


def wait(predicate, timeout=3.0):
    """Poll until predicate() holds; return its last value."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class Node:
    """A simulated robot and the driver node, wired over loopback."""

    def __init__(self, robot=None, **params):
        rospy.reset()
        self.robot = robot or FakeX30()
        self.perception = FakePerception()
        self.server = FakeRobotServer(self.robot, self.perception, motion_port=0,
                                      report_to=('127.0.0.1', 9), vmap_port=0, charge_port=0,
                                      lio_port=0, rate_hz=100.0)
        with open(os.path.join(PACKAGE, 'config', 'driver_params.yaml')) as f:
            rospy.PARAMS.update(yaml.safe_load(f))
        rospy.PARAMS.update(
            motion_host_ip='127.0.0.1', motion_host_port=self.server.motion_port,
            local_ip='127.0.0.1', local_port=0, perception_host_ip='127.0.0.1',
            elevation_map_port=self.server.vmap_port, charge_port=self.server.charge_port,
            lio_port=self.server.lio_port, transition_timeout_sec=2.0, feedback_timeout_sec=1.0,
            stand_timeout_sec=3.0)
        rospy.PARAMS.update(params)
        try:
            self.node = driver.X30Driver()
        except Exception:
            self.server.stop()
            raise
        self.server.report_to = self.node.link.local
        self.server.start()
        self._drive = None
        assert wait(lambda: self.state().get('connected') == 'True'), 'no reports'

    def call(self, name):
        return rospy.services[name](None)

    def cmd(self, vx=0.0, vy=0.0, wz=0.0):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        rospy.subscribers['cmd_vel'](msg)

    def drive(self, vx=0.0, vy=0.0, wz=0.0):
        """Publish cmd_vel at 20 Hz on a thread until stop_driving()."""
        self.stop_driving()
        stop = threading.Event()

        def run():
            while not stop.is_set():
                self.cmd(vx, vy, wz)
                time.sleep(0.05)
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        self._drive = (stop, thread)

    def stop_driving(self):
        if self._drive is not None:
            self._drive[0].set()
            self._drive[1].join()
            self._drive = None

    def last(self, topic):
        for name, msg in reversed(list(rospy.published)):
            if name == topic:
                return msg
        return None

    def state(self):
        msg = self.last('state')
        return {kv.key: kv.value for kv in msg.values} if msg is not None else {}

    def sent(self, base):
        """Return the codes with this command base the robot received."""
        return self.robot.codes(base)

    def close(self):
        self.stop_driving()
        rospy.signal_shutdown('test done')
        self.server.stop()
        assert not rospy.errors, rospy.errors


class NodeTest(unittest.TestCase):
    def node(self, robot=None, **params):
        node = Node(robot, **params)
        self.addCleanup(node.close)
        return node

    def stepping(self, gait=p.GAIT_STATE_WALK, non_manual=True, **params):
        return self.node(FakeX30(basic_state=p.STATE_STEPPING, gait_state=gait,
                                 non_manual=non_manual), **params)

    def assertMoving(self, node, vx=0.3):
        self.assertTrue(wait(lambda: abs(node.robot.velocity[0] - vx) < 0.005), node.state())

    def assertVelocityGate(self, node, value, timeout=3.0):
        self.assertTrue(wait(lambda: node.state().get('velocity') == value, timeout),
                        node.state().get('velocity'))

    def assertStill(self, node, seconds=1.0):
        """Check that the robot stops within 0.3 s and then stays still for this long."""
        wait(lambda: node.robot.velocity == (0.0, 0.0, 0.0), 0.3)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.assertEqual(node.robot.velocity, (0.0, 0.0, 0.0), node.state())
            time.sleep(0.05)


class TestTakeover(NodeTest):
    def test_manual_mode_service_latches_until_non_manual_mode(self):
        """manual_mode: no switch back after quiet; non_manual_mode, then a zero, drives."""
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        response = node.call('manual_mode')
        self.assertTrue(response.success, response.message)
        before = len(node.sent(p.NON_MANUAL_MODE))
        self.assertStill(node, 1.0)  # cmd_vel keeps coming
        self.assertVelocityGate(node, 'takeover (manual mode)')
        self.assertEqual(node.state()['takeover'], 'manual_mode service')
        node.stop_driving()
        time.sleep(1.5)  # quiet: still latched
        node.drive(0.3)
        self.assertStill(node, 1.0)
        self.assertFalse(node.robot.non_manual)
        self.assertEqual(len(node.sent(p.NON_MANUAL_MODE)), before)
        response = node.call('non_manual_mode')
        self.assertTrue(response.success, response.message)
        self.assertVelocityGate(node, 'waiting for zero or quiet cmd_vel')
        self.assertStill(node, 0.5)  # the stream kept running
        node.stop_driving()
        node.cmd(0.0)
        node.drive(0.3)
        self.assertMoving(node)

    def test_handheld_takeover_is_a_latch(self):
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        before = len(node.sent(p.NON_MANUAL_MODE))
        node.robot.set_mode(False)  # the operator switches to manual on the handheld
        self.assertStill(node, 1.0)
        self.assertIn('took over', node.state().get('takeover', ''))
        node.stop_driving()
        time.sleep(1.5)
        node.drive(0.3)
        self.assertStill(node, 1.0)
        self.assertFalse(node.robot.non_manual)
        self.assertEqual(len(node.sent(p.NON_MANUAL_MODE)), before)
        node.robot.set_mode(True)  # handed back on the handheld
        self.assertVelocityGate(node, 'waiting for zero or quiet cmd_vel')
        node.stop_driving()
        node.cmd(0.0)
        node.drive(0.3)
        self.assertMoving(node)

    def test_takeover_while_cmd_vel_is_zero(self):
        node = self.stepping()
        node.drive(0.0)
        time.sleep(0.3)
        node.robot.set_mode(False)
        self.assertVelocityGate(node, 'takeover (manual mode)')
        node.drive(0.3)
        self.assertStill(node, 1.0)
        self.assertFalse(node.robot.non_manual)
        self.assertTrue(any('not sent' in w[0] for w in rospy.warnings))


class TestServicesHold(NodeTest):
    def test_stop_stepping_holds_until_start_stepping(self):
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        response = node.call('stop_stepping')
        self.assertTrue(response.success, response.message)
        toggles = len(node.sent(p.STEPPING))
        time.sleep(1.0)  # cmd_vel keeps coming
        node.stop_driving()
        time.sleep(1.5)  # quiet: still held
        node.drive(0.3)
        time.sleep(1.0)
        self.assertEqual(node.robot.basic_state, p.STATE_TORQUE_STANDING)
        self.assertEqual(len(node.sent(p.STEPPING)), toggles)
        self.assertVelocityGate(node, 'hold:stop_stepping')
        response = node.call('start_stepping')
        self.assertTrue(response.success, response.message)
        self.assertIn('hold released', response.message)
        self.assertStill(node, 0.5)  # the stream kept running: a zero first
        node.stop_driving()
        node.cmd(0.0)
        node.drive(0.3)
        self.assertMoving(node)

    def test_emergency_stop_three_times_and_confirmed(self):
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        response = node.call('emergency_stop')
        self.assertTrue(response.success, response.message)
        self.assertIn('emergency_stop_or_fall', response.message)
        self.assertEqual(node.robot.estops, 3)
        modes = len(node.sent(p.NON_MANUAL_MODE))
        time.sleep(1.0)
        self.assertEqual(node.robot.basic_state, p.STATE_EMERGENCY)
        self.assertEqual(len(node.sent(p.NON_MANUAL_MODE)), modes)
        self.assertEqual(node.sent(p.STEPPING), [])

    def test_sit_down_holds_until_stand_up(self):
        node = self.stepping()
        response = node.call('sit_down')
        self.assertTrue(response.success, response.message)
        node.drive(0.3)
        time.sleep(1.0)
        self.assertEqual(node.robot.basic_state, p.STATE_SITTING)
        self.assertVelocityGate(node, 'hold:sit_down')
        node.stop_driving()
        response = node.call('stand_up')
        self.assertTrue(response.success, response.message)
        self.assertIn('hold released', response.message)
        self.assertVelocityGate(node, 'idle', 2.0)

    def test_sit_down_while_sitting_holds_too(self):
        """'already sitting' holds: no switches for cmd_vel, none after standing up by hand."""
        node = self.node(FakeX30(basic_state=p.STATE_SITTING, non_manual=False))
        response = node.call('sit_down')
        self.assertEqual((response.success, response.message), (True, 'already sitting'))
        self.assertEqual(node.node.stream.gate(time.monotonic()), 'hold:sit_down')
        node.drive(0.3)
        time.sleep(1.0)
        node.stop_driving()
        self.assertEqual(node.sent(p.NON_MANUAL_MODE), [])
        node.robot.basic_state = p.STATE_TORQUE_STANDING  # stood up on the handheld
        node.robot.set_mode(True)
        time.sleep(1.2)  # quiet: the next stream is a new episode
        node.drive(0.3)
        time.sleep(1.5)
        self.assertEqual(node.sent(p.STEPPING), [])
        self.assertVelocityGate(node, 'hold:sit_down')

    def hold_lands_during(self, service):
        """Run the service with emergency_stop's hold landing inside its first state read."""
        node = self.node(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True))
        real = node.node.basic_state
        fired = []

        def basic_state():
            state = real()
            if not fired:
                fired.append(True)
                node.node.stream.hold(time.monotonic(), 'emergency_stop')  # its own thread
            return state
        node.node.basic_state = basic_state
        response = node.call(service)
        self.assertIn('velocity hold kept: emergency_stop came in', response.message)
        self.assertEqual(node.node.stream.gate(time.monotonic()), 'hold:emergency_stop')

    def test_emergency_stop_during_stand_up_keeps_its_hold(self):
        self.hold_lands_during('stand_up')

    def test_emergency_stop_during_start_stepping_keeps_its_hold(self):
        self.hold_lands_during('start_stepping')

    def test_posture_services_one_at_a_time(self):
        """Two start_stepping at once: one stepping toggle goes out."""
        node = self.node(FakeX30(basic_state=p.STATE_TORQUE_STANDING, non_manual=True))
        results = []
        threads = [threading.Thread(target=lambda: results.append(node.call('start_stepping')))
                   for _ in range(2)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        self.assertEqual(len(node.sent(p.STEPPING)), 1)
        self.assertEqual(node.robot.basic_state, p.STATE_STEPPING)
        messages = sorted(r.message for r in results)
        self.assertTrue(any(r.success for r in results), messages)


class TestHeight(NodeTest):
    def test_old_height_reported_is_a_failure(self):
        node = self.stepping()
        node.robot.ignore_height = True
        response = node.call('height_crawl')
        self.assertFalse(response.success)
        self.assertIn('still reports normal', response.message)

    def test_success_without_any_height_report(self):
        robot = FakeX30(basic_state=p.STATE_STEPPING, non_manual=True)
        robot.send_height = False
        node = self.node(robot)
        response = node.call('height_crawl')
        self.assertTrue(response.success, response.message)
        self.assertIn('no body height report', response.message)

    def test_crawl_needs_a_fresh_motion_state(self):
        robot = FakeX30(basic_state=p.STATE_STEPPING, non_manual=True)
        node = self.node(robot)
        robot.send_motion = False
        self.assertTrue(wait(lambda: node.node.fresh_motion() is None))
        response = node.call('height_crawl')
        self.assertFalse(response.success)
        self.assertIn('no fresh motion state report', response.message)
        self.assertEqual(node.sent(p.BODY_HEIGHT), [])
        self.assertTrue(node.call('height_normal').success)  # normal height stays possible


class TestAxes(NodeTest):
    def test_no_axes_without_a_fresh_motion_state(self):
        robot = FakeX30(basic_state=p.STATE_STEPPING, non_manual=True)
        node = self.node(robot)
        robot.send_motion = False
        self.assertTrue(wait(lambda: node.node.fresh_motion() is None))
        axes = len(node.sent(p.AXIS_FORWARD))
        node.drive(0.3)
        time.sleep(0.8)
        self.assertEqual(len(node.sent(p.AXIS_FORWARD)), axes)
        self.assertIn('no fresh motion state', node.state().get('cmd_vel_ignored', ''))

    def test_gait_without_limits_does_not_move(self):
        node = self.stepping(gait=p.GAIT_STATE_SILENT)
        node.drive(0.3)
        self.assertStill(node, 1.0)
        self.assertIn('silent', node.state().get('cmd_vel_ignored', ''))
        self.assertTrue(node.state()['gait_speeds'].startswith('none'))

    def test_nan_in_any_field_stops(self):
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        node.stop_driving()
        msg = Twist()
        msg.linear.x, msg.angular.x = 0.3, float('nan')
        stop = threading.Event()

        def run():
            while not stop.is_set():
                rospy.subscribers['cmd_vel'](msg)
                time.sleep(0.05)
        thread = threading.Thread(target=run, daemon=True)
        thread.start()
        try:
            self.assertStill(node, 1.0)
        finally:
            stop.set()
            thread.join()
        self.assertTrue(any('NaN' in w[0] for w in rospy.warnings))

    def test_zero_twists_send_only_the_burst(self):
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        node.drive(0.0)
        start = len(node.robot.values)
        time.sleep(4.0)
        node.stop_driving()
        axes = [v for code, v in list(node.robot.values)[start:] if code & 0xFFFFFF in AXES]
        self.assertTrue(axes)
        self.assertEqual(set(axes), {0})
        self.assertLessEqual(len(axes), 3 * 32)  # stop_burst_sec at 50 Hz, three axes

    def test_gait_limits_parameter(self):
        node = self.stepping(gait=p.GAIT_STATE_SILENT,
                             gait_limits={'silent': [1.2, 0.75, 0.15, 0.45]})
        node.drive(0.3)
        self.assertMoving(node)
        self.assertEqual(node.state()['gait_speeds'], 'gait_limits')

    def test_remote_prefix_drives_in_manual_mode(self):
        node = self.stepping(command_prefix=0x21)
        node.drive(0.3)
        self.assertMoving(node)
        self.assertFalse(node.robot.non_manual)
        self.assertEqual(node.sent(p.MANUAL_MODE)[0], 0x21010C02)
        self.assertEqual(node.state()['obeyed_in'], 'manual')

    def test_shutdown_ends_with_zeros(self):
        """Shutdown: no velocity after it, then zero axes three times."""
        node = self.stepping()
        node.drive(0.3)
        self.assertMoving(node)
        node.stop_driving()
        node.drive(0.3)  # still commanding while the node shuts down
        rospy.signal_shutdown('test')
        zeros = [(axis, 0) for _ in range(3) for axis in AXES]

        def axis_log():
            return [(code & 0xFFFFFF, value) for code, value in list(node.robot.values)
                    if code & 0xFFFFFF in AXES]
        self.assertTrue(wait(lambda: axis_log()[-9:] == zeros), axis_log()[-9:])
        count = len(node.robot.values)
        time.sleep(0.3)
        self.assertEqual(len(node.robot.values), count)


class TestStartup(NodeTest):
    def test_prefix_is_checked(self):
        rospy.reset()
        rospy.PARAMS.update(command_prefix=0x41, perception_host_ip='', local_port=0)
        with self.assertRaises(ValueError):
            driver.X30Driver()

    def test_heartbeat_at_least_2_hz(self):
        node = self.node(heartbeat_hz=0.5)
        periods = [t.period for t in rospy.timers if t.callback == node.node.send_heartbeat]
        self.assertEqual(periods, [0.5])


class TestSaveData(NodeTest):
    def test_only_sitting_or_after_an_emergency_stop(self):
        node = self.node(FakeX30(basic_state=p.STATE_TORQUE_STANDING))
        response = node.call('save_data')
        self.assertFalse(response.success)
        self.assertIsNone(node.robot.saved)
        node.robot.basic_state = p.STATE_SITTING
        self.assertTrue(wait(lambda: node.node.basic_state() == p.STATE_SITTING))
        response = node.call('save_data')
        self.assertTrue(response.success, response.message)
        self.assertIn('0x31010C01', response.message)
        self.assertTrue(wait(lambda: node.robot.saved == 0x31010C01))

    def test_old_code_goes_out_raw(self):
        node = self.node(save_data_code=18)
        self.assertTrue(node.call('save_data').success)
        self.assertTrue(wait(lambda: node.robot.saved == 18))


class TestPerception(NodeTest):
    def test_velocity_source_sets_non_manual_mode_first(self):
        node = self.node()
        response = node.call('velocity_source_joystick')
        self.assertTrue(response.success, response.message)
        self.assertTrue(node.robot.non_manual)
        self.assertEqual(node.perception.velocity_source, 1)

    def test_obstacle_height_code_parameter(self):
        node = self.node(obstacle_height_code=0x3101EE04)
        response = node.call('obstacle_height_28cm')
        self.assertTrue(response.success, response.message)
        self.assertTrue(wait(lambda: node.perception.obstacle_codes == [0x3101EE04]))
        self.assertIn('/step_z_max_state', response.message)

    def test_leaving_multi_frame_turns_the_odometry_off(self):
        """1.5.2.5: the new mode first, then the LiDAR-inertial odometry off."""
        node = self.node()
        node.perception.lio_on = True
        node.perception.elevation_mode = p.ELEVATION_MULTI_FRAME
        self.assertTrue(wait(lambda: node.state().get('elevation_mode') == 'multi_frame', 8.0))
        response = node.call('elevation_grid_ground')
        self.assertTrue(response.success, response.message)
        self.assertIn('odometry off', response.message)
        self.assertFalse(node.perception.lio_on)
        self.assertEqual(node.perception.elevation_mode, p.ELEVATION_GRID_GROUND)


if __name__ == '__main__':
    unittest.main()
