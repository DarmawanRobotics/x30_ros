#!/usr/bin/env python3
"""rostest: the driver, the control mux and the sector scans together against fake_robot.py."""

from __future__ import annotations

import math
import threading
import time
import unittest

from genpy.dynamic import generate_dynamic
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rosgraph
import rospy
import rostest
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, SectorScan
from x30_perception.core.livox import DEFINITION

LIVOX = generate_dynamic('livox_ros_driver2/CustomMsg', DEFINITION)
CUSTOM_MSG = LIVOX['livox_ros_driver2/CustomMsg']
CUSTOM_POINT = LIVOX['livox_ros_driver2/CustomPoint']
SPEED = 0.2


def livox_scan(points, frame_id: str = 'livox_frame'):
    """A CustomMsg with points (x, y, z) in livox_frame (base_link: identity mount)."""
    msg = CUSTOM_MSG(point_num=len(points), lidar_id=1, rsvd=[0, 0, 0])
    msg.header.stamp, msg.header.frame_id = rospy.Time.now(), frame_id
    msg.points = [CUSTOM_POINT(offset_time=i, x=x, y=y, z=z) for i, (x, y, z) in enumerate(points)]
    return msg


def ring(radius: float = 3.0, n: int = 720) -> list:
    """Walls all around at base_link height (0.48 m above the floor: in the height band)."""
    return [
        (radius * math.cos(a), radius * math.sin(a), 0.0)
        for a in (2.0 * math.pi * i / n - math.pi for i in range(n))
    ]


def wall_ahead(x: float) -> list:
    """An obstacle face x m ahead."""
    return [(x, -0.3 + 0.05 * i, z) for i in range(13) for z in (-0.2, 0.0, 0.3)]


class StackBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.points = ring()
        cls.odom = cls.status = cls.scan = None
        cls.livox = rospy.Publisher('/livox/lidar', CUSTOM_MSG, queue_size=1)
        cls.nav = rospy.Publisher('/control/nav_cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/x30/odom', Odometry, cls.keep('odom'), queue_size=1)
        rospy.Subscriber('/control/status', ControlStatus, cls.keep('status'), queue_size=1)
        rospy.Subscriber('/perception/safety_scan', SectorScan, cls.keep('scan'), queue_size=1)
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
            with cls.lock:
                points = list(cls.points)
            cls.livox.publish(livox_scan(points))

    @classmethod
    def see(cls, points: list):
        """Change what the LiDAR sees from the next scan on."""
        with cls.lock:
            cls.points = list(points)

    def latest(self, name: str):
        with self.lock:
            return getattr(self, name)

    def wait(self, name: str, predicate, timeout: float = 20.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.latest(name)
            if msg is not None and predicate(msg):
                return msg
            time.sleep(0.05)
        self.fail(f'{name}: condition not met in {timeout} s, last: {self.latest(name)}')

    def drive(self, vx: float, until, timeout: float = 15.0):
        """Publish navigation commands at 20 Hz until until(odom); return the last odom."""
        msg = Twist()
        msg.linear.x = vx
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.nav.publish(msg)
            odom = self.latest('odom')
            if odom is not None and until(odom):
                return odom
            time.sleep(0.05)
        self.fail(f'drive {vx}: odom never matched, last {self.latest("odom")}')

    def call(self, name: str) -> Trigger._response_class:
        rospy.wait_for_service(name, timeout=30.0)
        return rospy.ServiceProxy(name, Trigger)()

    def test_0_scans_and_mux_come_up(self):
        """Livox bytes give an unblocked safety scan; the mux sees it and arms navigation."""
        scan = self.wait('scan', lambda s: min(s.ranges) > 0.0)
        self.assertEqual(scan.header.frame_id, 'base_link')
        self.assertAlmostEqual(min(scan.ranges), 3.0, delta=0.05)
        self.wait('status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR)
        master = rosgraph.Master('/stack_check')
        publishers = dict(master.getSystemState()[0]).get('/x30/cmd_vel', [])
        self.assertEqual(publishers, ['/control_mux_node'])  # the mux is the only one

    def test_1_navigation_walks_the_robot(self):
        """control/nav_cmd_vel -> mux -> /x30/cmd_vel -> driver -> the robot walks."""
        self.assertIn('torque', self.call('/x30/stand_up').message)
        odom = self.drive(SPEED, lambda o: abs(o.twist.twist.linear.x - SPEED) < 0.01)
        self.assertAlmostEqual(odom.twist.twist.linear.x, SPEED, delta=0.01)
        start = odom.pose.pose.position.x
        odom = self.drive(SPEED, lambda o: o.pose.pose.position.x - start > 0.2)
        self.assertGreater(odom.pose.pose.position.x - start, 0.2)

    def test_2_the_protective_field_stops_it(self):
        """An obstacle 0.6 m ahead: the field stops the robot while navigation still pushes."""
        self.see(ring() + wall_ahead(0.6))
        self.wait('scan', lambda s: min(s.ranges) < 1.0)

        def stopped_by_the_field(odom) -> bool:
            status = self.latest('status')
            stopped = abs(odom.twist.twist.linear.x) < 1e-3
            return stopped and status is not None and status.field == ControlStatus.FIELD_STOP

        self.drive(SPEED, stopped_by_the_field)  # the field state is the newest command's
        self.see(ring())
        odom = self.drive(SPEED, lambda o: abs(o.twist.twist.linear.x - SPEED) < 0.01)
        self.assertAlmostEqual(odom.twist.twist.linear.x, SPEED, delta=0.01)

    def test_3_estop_holds_until_release_and_rearm(self):
        """The software e-stop stops the robot; after the release navigation must re-arm."""
        self.assertTrue(self.call('/control/estop/engage').success)
        self.drive(SPEED, lambda o: abs(o.twist.twist.linear.x) < 1e-3)
        self.assertTrue(self.call('/control/estop/release').success)
        self.wait('status', lambda s: not s.estop and not s.armed, timeout=5.0)
        time.sleep(1.0)  # navigation silent for nav_timeout_sec: armed again
        self.wait('status', lambda s: s.armed)
        odom = self.drive(SPEED, lambda o: abs(o.twist.twist.linear.x - SPEED) < 0.01)
        self.assertAlmostEqual(odom.twist.twist.linear.x, SPEED, delta=0.01)


if __name__ == '__main__':
    rospy.init_node('stack_check')
    rostest.rosrun('x30_bringup', 'stack_base', StackBase)
