#!/usr/bin/env python3
"""rostest: a route through the follower, the mux and the driver to the simulated X30's legs."""

from __future__ import annotations

import threading
import time
import unittest

from nav_msgs.msg import Odometry
import rospy
import rostest
from stack_check import CUSTOM_MSG, livox_scan, ring
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, LocalizationStatus, NavigationStatus, Waypoint
from x30_msgs.srv import SetPath

LENGTH = 1.2  # m, straight ahead


class StackNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.odom = cls.status = cls.nav = None
        cls.livox = rospy.Publisher('/livox/lidar', CUSTOM_MSG, queue_size=1)
        cls.localization = rospy.Publisher(
            '/localization/status', LocalizationStatus, queue_size=1, latch=True
        )
        cls.localization.publish(LocalizationStatus(state=LocalizationStatus.LOCALIZED))
        rospy.Subscriber('/x30/odom', Odometry, cls.keep('odom'), queue_size=1)
        rospy.Subscriber('/control/status', ControlStatus, cls.keep('status'), queue_size=1)
        rospy.Subscriber('/navigation/status', NavigationStatus, cls.keep('nav'), queue_size=1)
        cls.stop = threading.Event()
        cls.thread = threading.Thread(target=cls.scan_loop, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.stop.set()

    @classmethod
    def keep(cls, name: str):
        def callback(msg):
            with cls.lock:
                setattr(cls, name, msg)

        return callback

    @classmethod
    def scan_loop(cls):
        while not cls.stop.wait(0.1) and not rospy.is_shutdown():
            cls.livox.publish(livox_scan(ring()))  # walls 3 m around: always clear

    def latest(self, name: str):
        with self.lock:
            return getattr(self, name)

    def wait(self, name: str, predicate, timeout: float = 30.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.latest(name)
            if msg is not None and predicate(msg):
                return msg
            time.sleep(0.05)
        self.fail(f'{name}: condition not met in {timeout} s, last: {self.latest(name)}')

    def call(self, name: str, srv=Trigger, **request):
        rospy.wait_for_service(name, timeout=30.0)
        return rospy.ServiceProxy(name, srv)(**request)

    def test_the_robot_walks_a_route_to_its_end(self):
        """navigation/set_path + start -> follower -> mux -> driver: the robot walks the route."""
        self.wait('status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR)
        self.assertIn('torque', self.call('/x30/stand_up').message)
        start = self.wait('odom', lambda o: True).pose.pose.position
        steps = int(LENGTH / 0.1)
        route = [
            Waypoint(x=start.x + 0.1 * i, y=start.y, yaw=0.0, dwell_sec=0.0)
            for i in range(steps + 1)
        ]
        loaded = self.call('/navigation/set_path', SetPath, waypoints=route, source='test')
        self.assertTrue(loaded.success, loaded.message)
        started = self.call('/navigation/start')
        self.assertTrue(started.success, started.message)
        self.wait('nav', lambda n: n.state == NavigationStatus.FOLLOWING, 10.0)
        self.wait('nav', lambda n: n.state == NavigationStatus.GOAL_REACHED, 90.0)
        end = self.wait('odom', lambda o: abs(o.twist.twist.linear.x) < 0.02, 10.0)
        self.assertAlmostEqual(end.pose.pose.position.x - start.x, LENGTH, delta=0.15)
        self.assertAlmostEqual(end.pose.pose.position.y - start.y, 0.0, delta=0.1)


if __name__ == '__main__':
    rospy.init_node('navigation_check')
    rostest.rosrun('x30_bringup', 'stack_navigation', StackNavigation)
