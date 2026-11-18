#!/usr/bin/env python3
"""rostest: the patrol loop on x30_sim with the NeuPAN controller, around the box on the route."""

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

from x30_msgs.msg import ControlStatus, LocalizationStatus, NavigationStatus
from x30_msgs.srv import LoadPath

ROUTE = 'mall/routes/mall_loop.csv'  # drawn into the site from the world
BOX = (4.5, 0.0, 0.3)  # person_crossing: x, y, radius, on the route up the east corridor
HALF_WIDTH = 0.235  # robot/footprint/half_width
ENDS_WITHIN = 0.3  # m, the true pose from the end of the loop (its start)


class StackNeupan(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.latest, cls.messages = {}, []
        cls.touched = False
        cls.closest = math.inf  # the true robot's nearest approach to the box's centre
        for topic, msg_type in (
            ('/localization/status', LocalizationStatus),
            ('/control/status', ControlStatus),
            ('/navigation/status', NavigationStatus),
            ('/sim/ground_truth', PoseStamped),
            ('/sim/collision', Bool),
        ):
            rospy.Subscriber(topic, msg_type, cls.keep(topic), queue_size=10)

    @classmethod
    def keep(cls, topic: str):
        def callback(msg):
            with cls.lock:
                cls.latest[topic] = msg
                if isinstance(msg, Bool):
                    cls.touched = cls.touched or msg.data
                elif isinstance(msg, NavigationStatus):
                    cls.messages.append(msg.message)
                elif isinstance(msg, PoseStamped):
                    p = msg.pose.position
                    cls.closest = min(cls.closest, math.hypot(p.x - BOX[0], p.y - BOX[1]))

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

    def call(self, name: str, srv_type=Trigger, **request):
        rospy.wait_for_service(name, timeout=30.0)
        answer = rospy.ServiceProxy(name, srv_type)(**request)
        self.assertTrue(answer.success, f'{name}: {answer.message}')
        return answer

    def test_neupan_walks_the_loop_around_the_box(self):
        """Localized, the loop loaded and started, NeuPAN passes the box and ends the loop."""
        self.wait('/localization/status', lambda s: s.state == LocalizationStatus.LOCALIZED, 60.0)
        self.wait(
            '/control/status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR, 30
        )
        self.call('/navigation/load_path', LoadPath, path=ROUTE)
        self.wait('/navigation/status', lambda n: n.count > 0, 10.0)
        self.call('/navigation/start')
        done = (NavigationStatus.GOAL_REACHED, NavigationStatus.FAILED)
        status = self.wait('/navigation/status', lambda n: n.state in done, 330.0)
        self.assertEqual(status.state, NavigationStatus.GOAL_REACHED, status.message)
        truth = self.last('/sim/ground_truth').pose.position
        start = (-4.5, -3.0)  # mall_loop's start and end
        error = math.hypot(truth.x - start[0], truth.y - start[1])
        with self.lock:
            touched, closest = self.touched, self.closest
            following = [m for m in self.messages if '| neupan |' in m]
        rospy.loginfo('loop ended %.3f m from its end, %.3f m from the box', error, closest)
        self.assertTrue(following, 'the follower never reported the neupan controller')
        self.assertFalse(touched, 'the robot touched something')
        self.assertGreater(closest, BOX[2] + HALF_WIDTH, 'not around the box')
        self.assertLess(error, ENDS_WITHIN)


if __name__ == '__main__':
    rospy.init_node('stack_neupan_check')
    rostest.rosrun('x30_bringup', 'stack_neupan', StackNeupan)
