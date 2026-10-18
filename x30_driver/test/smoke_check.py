#!/usr/bin/env python3
"""rostest: the driver (launch/driver.launch) against fake_robot.py, through its ROS interface."""

import math
import threading
import time
import unittest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy
import rostest
from sensor_msgs.msg import BatteryState, Imu, JointState
from std_srvs.srv import Trigger
from x30_driver.convert import quaternion_from_rpy
from x30_driver.protocol import JOINT_NAMES

NS = '/x30/'


def topic(name, kind, timeout=20.0):
    """Return the next message on a driver topic."""
    return rospy.wait_for_message(NS + name, kind, timeout=timeout)


def call(name):
    """Call a driver Trigger service."""
    rospy.wait_for_service(NS + name, timeout=30.0)
    return rospy.ServiceProxy(NS + name, Trigger)()


def values(status):
    """Return the KeyValues of a DiagnosticStatus as a dict."""
    return {kv.key: kv.value for kv in status.values}


def wait_state(predicate, timeout=20.0):
    """Return the state's values once predicate(values) holds, the last values otherwise."""
    end = time.monotonic() + timeout
    last = {}
    while time.monotonic() < end:
        last = values(topic('state', DiagnosticStatus, timeout=5.0))
        if predicate(last):
            break
    return last


class TestDriver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.odom = None
        rospy.Subscriber(NS + 'odom', Odometry, cls.on_odom, queue_size=1)
        cls.cmd_vel = rospy.Publisher(NS + 'cmd_vel', Twist, queue_size=1)

    @classmethod
    def on_odom(cls, msg):
        with cls.lock:
            cls.odom = msg

    def last_odom(self):
        with self.lock:
            return self.odom

    def drive(self, vx, vy, wz, predicate, timeout=8.0):
        """Publish cmd_vel at 20 Hz until predicate(odom) holds; return the last odom."""
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = vx, vy, wz
        end = time.monotonic() + timeout
        odom = None
        while time.monotonic() < end and not rospy.is_shutdown():
            self.cmd_vel.publish(msg)
            time.sleep(0.05)
            odom = self.last_odom()
            if odom is not None and predicate(odom):
                break
        return odom

    def assertCall(self, name, success=True, contains=''):
        response = call(name)
        self.assertEqual(response.success, success, '{}: {}'.format(name, response.message))
        self.assertIn(contains, response.message)
        return response

    def test_0_reports(self):
        """Connected; odom, imu, joint_states, battery and diagnostics from the reports."""
        state = wait_state(lambda v: v.get('connected') == 'True')
        self.assertEqual(state.get('connected'), 'True', state)
        self.assertEqual(state['basic_state_name'], 'sitting')
        self.assertEqual(state['control_mode'], 'manual')
        self.assertEqual(state['robot_name'], 'X30-SIM')
        self.assertEqual((state['rcs_layout'], state['motion_layout']), ('natural', 'natural'))

        odom = topic('odom', Odometry)
        self.assertEqual((odom.header.frame_id, odom.child_frame_id), ('odom', 'base_link'))

        imu = topic('imu', Imu)
        self.assertEqual(imu.header.frame_id, 'imu_link')
        self.assertAlmostEqual(imu.linear_acceleration.z, 9.81, places=4)
        want = quaternion_from_rpy(math.radians(0.5), math.radians(-1.0), 0.0)
        got = (imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w)
        for g, w in zip(got, want):
            self.assertAlmostEqual(g, w, places=4)

        joints = topic('joint_states', JointState)
        self.assertEqual(list(joints.name), list(JOINT_NAMES))
        self.assertEqual(len(joints.position), 12)
        self.assertEqual(list(joints.effort), [1.5] * 12)

        battery = topic('battery', BatteryState)
        self.assertAlmostEqual(battery.percentage, 0.87, places=5)
        self.assertAlmostEqual(battery.voltage, 54.0, places=4)
        self.assertAlmostEqual(battery.current, -3.5, places=4)
        self.assertAlmostEqual(battery.capacity, 15.0, places=4)

        status = topic('diagnostics', DiagnosticArray).status[0]
        self.assertEqual(status.name, 'x30_driver')
        self.assertIn('rx', values(status))

    def test_1_stand_up_and_step(self):
        """stand_up runs stand/sit then torque control; start_stepping toggles stepping."""
        self.assertCall('gait_walk', success=False, contains='stepping')
        self.assertCall('stand_up', contains='torque')
        self.assertEqual(wait_state(lambda v: v['basic_state_name'] == 'torque_standing')
                         ['basic_state_name'], 'torque_standing')
        self.assertCall('start_stepping', contains='stepping')
        self.assertCall('start_stepping', contains='already')

    def test_2_cmd_vel(self):
        """cmd_vel walks the robot in REP-103: non-manual mode on the way, zeros after."""
        odom = self.drive(0.3, 0.0, 0.0, lambda o: abs(o.twist.twist.linear.x - 0.3) < 0.005)
        self.assertAlmostEqual(odom.twist.twist.linear.x, 0.3, delta=0.005)
        self.assertEqual(wait_state(lambda v: v['control_mode'] == 'non_manual')
                         ['control_mode'], 'non_manual')
        odom = self.drive(0.0, 0.1, 0.0, lambda o: abs(o.twist.twist.linear.y - 0.1) < 0.005)
        self.assertAlmostEqual(odom.twist.twist.linear.y, 0.1, delta=0.005)   # + left
        odom = self.drive(0.0, 0.0, 0.3, lambda o: abs(o.twist.twist.angular.z - 0.3) < 0.005)
        self.assertAlmostEqual(odom.twist.twist.angular.z, 0.3, delta=0.005)  # + ccw
        self.assertGreater(odom.pose.pose.position.x, 0.0)
        end = time.monotonic() + 5.0  # watchdog: zeros, then silence
        while time.monotonic() < end:
            odom = self.last_odom()
            if odom.twist.twist.angular.z == 0.0 and odom.twist.twist.linear.x == 0.0:
                break
            time.sleep(0.05)
        self.assertEqual(odom.twist.twist.angular.z, 0.0)

    def test_3_gaits_and_height(self):
        """Gaits answer with the reported gait; crawl height allows walk and slope only."""
        self.assertCall('gait_slope', contains='slope')
        self.assertCall('height_crawl', contains='crawl height')
        self.assertCall('gait_off_road', success=False, contains='crawl height')
        self.assertCall('height_normal', contains='normal height')
        self.assertCall('gait_walk', contains='walk')

    def test_4_perception_host(self):
        """Elevation map modes and charging through the perception host."""
        self.assertCall('elevation_grid_ground', contains='grid_ground')
        self.assertCall('gait_stair_45', success=False, contains='multi_frame')
        self.assertCall('obstacle_detour', contains='detour')
        self.assertCall('charge_start', contains='startup')
        state = wait_state(lambda v: v.get('charge_state') == 'charging')
        self.assertEqual(state.get('charge_state'), 'charging')
        self.assertEqual(state.get('elevation_mode'), 'grid_ground')
        self.assertCall('charge_clear', contains='idle')

    def test_5_sit_down_and_emergency_stop(self):
        """sit_down leaves stepping first; the soft emergency stop shows in the state."""
        self.assertCall('sit_down', contains='sitting')
        self.assertCall('emergency_stop')
        state = wait_state(lambda v: v['basic_state_name'] == 'emergency_stop_or_fall')
        self.assertEqual(state['basic_state_name'], 'emergency_stop_or_fall')
        self.assertEqual(state['emergency_source'], 'client')
        self.assertEqual(topic('state', DiagnosticStatus).level, DiagnosticStatus.WARN)
        self.assertCall('stand_up', success=False, contains='emergency')
        self.assertCall('save_data', contains='0x10C01')


if __name__ == '__main__':
    rospy.init_node('smoke_check')
    rostest.rosrun('x30_driver', 'smoke', TestDriver)
