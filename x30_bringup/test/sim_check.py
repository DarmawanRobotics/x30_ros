#!/usr/bin/env python3
"""rostest: the whole stack on x30_sim localizes, walks to a goal around a corner, tags correct."""

from __future__ import annotations

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
import rospy
import rostest
from std_msgs.msg import Bool

from x30_msgs.msg import ControlStatus, HealthStatus, LocalizationStatus, PlannerStatus
from x30_msgs.srv import GoTo

GOAL = (4.5, 0.0, math.pi / 2)  # up the east corridor of mall_loop, from the start at (-4.5, -3)
ARRIVED_WITHIN = 0.15  # m, the true pose from the goal (0.25 m without the tag corrections)


class StackSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.latest = {}
        cls.sources = set()  # the localization's corrections seen
        for topic, msg_type in (
            ('/localization/status', LocalizationStatus),
            ('/control/status', ControlStatus),
            ('/planner/status', PlannerStatus),
            ('/health/status', HealthStatus),
            ('/sim/ground_truth', PoseStamped),
            ('/sim/collision', Bool),
        ):
            rospy.Subscriber(topic, msg_type, cls.keep(topic), queue_size=1)

    @classmethod
    def keep(cls, topic: str):
        def callback(msg):
            with cls.lock:
                cls.latest[topic] = msg
                if isinstance(msg, LocalizationStatus):
                    cls.sources.add(msg.source)

        return callback

    def last(self, topic: str):
        with self.lock:
            return self.latest.get(topic)

    def wait(self, topic: str, predicate, timeout: float):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            msg = self.last(topic)
            if msg is not None and predicate(msg):
                return msg
            time.sleep(0.1)
        self.fail(f'{topic}: condition not met in {timeout} s, last: {self.last(topic)}')

    def test_localizes_and_walks_to_the_goal(self):
        """/initialpose from the simulator, go_to around a corner, a tag on the way, arrival."""
        self.wait('/localization/status', lambda s: s.state == LocalizationStatus.LOCALIZED, 60.0)
        self.wait(
            '/control/status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR, 30
        )
        rospy.wait_for_service('/planner/go_to', timeout=30.0)
        answer = rospy.ServiceProxy('/planner/go_to', GoTo)(*GOAL)
        self.assertTrue(answer.success, answer.message)
        done = (PlannerStatus.ARRIVED, PlannerStatus.FAILED, PlannerStatus.CANCELLED)
        status = self.wait('/planner/status', lambda s: s.state in done, 150.0)
        self.assertEqual(status.state, PlannerStatus.ARRIVED, status.message)
        truth = self.last('/sim/ground_truth').pose.position
        error = math.hypot(truth.x - GOAL[0], truth.y - GOAL[1])
        rospy.loginfo('arrived %.3f m from the goal (true pose)', error)
        self.assertLess(error, ARRIVED_WITHIN)
        self.assertFalse(self.last('/sim/collision').data, 'the robot touched something')
        localization = self.last('/localization/status')
        self.assertEqual(localization.state, LocalizationStatus.LOCALIZED, localization.message)
        # FAST-LIO drifts 1 % and 0.1 deg/m in the scenario: the tag on the east wall, seen by
        # the simulated camera's detector on the way, corrected it
        with self.lock:
            self.assertIn('tag:base_map', self.sources)
        self.assertNotEqual(self.last('/health/status').level, HealthStatus.ERROR)


if __name__ == '__main__':
    rospy.init_node('stack_sim_check')
    rostest.rosrun('x30_bringup', 'stack_sim', StackSim)
