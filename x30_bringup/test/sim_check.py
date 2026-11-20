#!/usr/bin/env python3
"""rostest: the whole stack on x30_sim localizes, walks the patrol loop, a tag corrects drift."""

from __future__ import annotations

import math
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped
import rospy
import rostest
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import tf2_ros

from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    NavigationStatus,
    WaypointPath,
)
from x30_msgs.srv import LoadPath

ROUTE = 'mall/routes/mall_loop.csv'  # the patrol loop of mall_loop, drawn into the site
ARRIVED_WITHIN = 0.15  # m, the true pose from the loop's end (0.25 m without the tag corrections)
LOCALIZED_WITHIN = 0.10  # m, map -> base_link from the true pose
STARTS_WITHIN = 0.30  # m, map -> base_link from the true pose before the loop starts


class StackSim(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.latest = {}
        cls.sources = set()  # the localization's corrections seen
        cls.tf = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf)
        for topic, msg_type in (
            ('/localization/status', LocalizationStatus),
            ('/control/status', ControlStatus),
            ('/navigation/status', NavigationStatus),
            ('/navigation/waypoints', WaypointPath),
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

    def off_truth(self) -> float:
        """Return how far map -> base_link is from the true pose (inf: not known yet)."""
        truth = self.last('/sim/ground_truth')
        try:
            seen = self.tf.lookup_transform('map', 'base_link', rospy.Time(0))
        except tf2_ros.TransformException:
            return math.inf
        if truth is None:
            return math.inf
        p, t = seen.transform.translation, truth.pose.position
        return math.hypot(p.x - t.x, p.y - t.y)

    def call(self, name: str, srv_type=Trigger, **request):
        rospy.wait_for_service(name, timeout=30.0)
        answer = rospy.ServiceProxy(name, srv_type)(**request)
        self.assertTrue(answer.success, f'{name}: {answer.message}')
        return answer

    def test_localizes_and_walks_the_loop(self):
        """Localized at the start, the loop walked round its corners, a tag on the way, arrival."""
        self.wait('/localization/status', lambda s: s.state == LocalizationStatus.LOCALIZED, 60.0)
        # map -> base_link shows the pose estimate a little later (FAST-LIO's TF lags)
        end = time.monotonic() + 30.0
        while self.off_truth() > STARTS_WITHIN:
            self.assertLess(
                time.monotonic(), end, f'map -> base_link {self.off_truth():.2f} m off'
            )
            time.sleep(0.1)
        self.wait(
            '/control/status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR, 30
        )
        self.call('/navigation/load_path', LoadPath, path=ROUTE)
        goal = self.wait('/navigation/waypoints', lambda w: len(w.waypoints) > 0, 10.0)
        goal = goal.waypoints[-1]  # the loop's end, in the map frame
        self.call('/navigation/start')
        done = (NavigationStatus.GOAL_REACHED, NavigationStatus.FAILED)
        status = self.wait('/navigation/status', lambda n: n.state in done, 200.0)
        self.assertEqual(status.state, NavigationStatus.GOAL_REACHED, status.message)
        truth = self.last('/sim/ground_truth').pose.position
        error = math.hypot(truth.x - goal.x, truth.y - goal.y)
        seen = self.tf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        estimate = seen.transform.translation
        drift = math.hypot(estimate.x - truth.x, estimate.y - truth.y)
        rospy.loginfo(
            "arrived %.3f m from the loop's end, localized %.3f m off (true pose)", error, drift
        )
        self.assertLess(drift, LOCALIZED_WITHIN)
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
