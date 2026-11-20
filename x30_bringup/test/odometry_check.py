#!/usr/bin/env python3
"""rostest: on x30_sim with no map: the start is the origin, a taught route repeated."""

from __future__ import annotations

import math
import os
import threading
import time
import unittest

from geometry_msgs.msg import PoseStamped, Twist, Vector3
import rospy
import rostest
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
import tf2_ros

from x30_msgs.msg import ControlStatus, LocalizationStatus, NavigationStatus, SiteStatus
from x30_msgs.srv import LoadPath, MarkStopPoint, SetMode

SITE = 'yard'  # stack_odometry.test's
LEG = 3.0  # m taught ahead: the south corridor of mall_loop, from the start (-4.5, -3) facing +x
BACK = 2.7  # m walked back before the repeat: the robot joins the route near its start
STICK = 0.4  # m/s, the operator's speed while teaching (0.3 backward)
DWELL = 1.0  # s at the stop point that ends the route
AT_ORIGIN = 0.05  # m, map -> base_link from the origin at the start
FRAME_WITHIN = 0.10  # m, the taught stop in the map frame from the true walk from the start
REPEATED_WITHIN = 0.15  # m, the true pose after the repeat from the true pose at the stop


class StackOdometry(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.Lock()
        cls.latest = {}
        cls.watching = False
        cls.localization_states = set()  # every state seen once watching
        cls.tf = tf2_ros.Buffer()
        cls.tf_listener = tf2_ros.TransformListener(cls.tf)
        for topic, msg_type in (
            ('/localization/status', LocalizationStatus),
            ('/control/status', ControlStatus),
            ('/navigation/status', NavigationStatus),
            ('/site/status', SiteStatus),
            ('/route/saved', String),
            ('/sim/ground_truth', PoseStamped),
            ('/sim/collision', Bool),
        ):
            rospy.Subscriber(topic, msg_type, cls.keep(topic), queue_size=1)
        cls.stick = rospy.Publisher('/control/remote/handheld/cmd_vel', Twist, queue_size=1)

    @classmethod
    def keep(cls, topic: str):
        def callback(msg):
            with cls.lock:
                cls.latest[topic] = msg
                if isinstance(msg, LocalizationStatus) and cls.watching:
                    cls.localization_states.add(msg.state)

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

    def call(self, name: str, srv=Trigger, **request):
        rospy.wait_for_service(name, timeout=30.0)
        answer = rospy.ServiceProxy(name, srv)(**request)
        self.assertTrue(answer.success, f'{name}: {answer.message}')
        return answer

    def truth(self) -> tuple[float, float]:
        p = self.wait('/sim/ground_truth', lambda m: True, 10.0).pose.position
        return p.x, p.y

    def map_pose(self) -> tuple[float, float]:
        seen = self.tf.lookup_transform('map', 'base_link', rospy.Time(0), rospy.Duration(2.0))
        return seen.transform.translation.x, seen.transform.translation.y

    def walk(self, distance: float, speed: float):
        """Hold the stick until the true pose moved distance, then neutral until it stands."""
        x0, y0 = self.truth()
        end = time.monotonic() + distance / abs(speed) + 20.0
        while True:
            x, y = self.truth()
            if math.hypot(x - x0, y - y0) >= distance:
                break
            self.assertLess(time.monotonic(), end, f'at {(x, y)}, from {(x0, y0)}')
            self.stick.publish(Twist(linear=Vector3(x=speed)))
            time.sleep(0.05)
        self.neutral(1.5)

    def neutral(self, seconds: float):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.stick.publish(Twist())
            time.sleep(0.05)

    def test_teaches_and_repeats_a_route_without_a_map(self):
        """The site of the world's tags and route, LOCALIZED at the origin; one more taught."""
        site = self.wait('/site/status', lambda s: s.site == SITE, 60.0)
        drawn = site.routes  # the world's patrol loop
        self.wait('/localization/status', lambda s: s.state == LocalizationStatus.LOCALIZED, 60)
        self.assertEqual(self.last('/localization/status').source, 'origin')
        with self.lock:
            type(self).watching = True  # from here on: never NONE nor UNCERTAIN
        x, y = self.map_pose()
        self.assertLess(math.hypot(x, y), AT_ORIGIN, 'the odometry start is the map origin')
        start = self.truth()
        self.wait(
            '/control/status', lambda s: s.armed and s.field == ControlStatus.FIELD_CLEAR, 30
        )

        # teach: the operator walks the route on the stick, a stop point at its end
        self.call('/control/set_mode', SetMode, mode='remote', source='handheld')
        self.neutral(1.0)  # re-arms the stick
        self.wait('/control/status', lambda s: s.mode == ControlStatus.MODE_REMOTE and s.armed, 5)
        self.call('/route/start')
        self.walk(LEG, STICK)
        self.call('/route/mark_stop_point', MarkStopPoint, dwell_sec=DWELL)
        taught_truth, taught_map = self.truth(), self.map_pose()
        self.call('/route/stop')
        path = self.wait('/route/saved', lambda m: m.data, 5.0).data
        self.assertTrue(os.path.isfile(path) and f'/{SITE}/routes/' in path, path)
        walked = (taught_truth[0] - start[0], taught_truth[1] - start[1])  # start yaw: 0
        self.assertLess(
            math.hypot(taught_map[0] - walked[0], taught_map[1] - walked[1]),
            FRAME_WITHIN,
            f'taught at {taught_map} in the map frame, walked {walked} from the start',
        )

        # back near the start on the stick, then the follower repeats the route
        self.walk(BACK, -0.75 * STICK)
        self.call('/control/set_mode', SetMode, mode='auto', source='')
        self.call('/navigation/load_path', LoadPath, path=path)
        self.wait('/navigation/status', lambda n: n.count > 0, 10.0)  # the follower has it
        self.call('/navigation/start')
        done = (NavigationStatus.GOAL_REACHED, NavigationStatus.FAILED)
        status = self.wait('/navigation/status', lambda n: n.state in done, 120.0)
        self.assertEqual(status.state, NavigationStatus.GOAL_REACHED, status.message)
        end = self.truth()
        error = math.hypot(end[0] - taught_truth[0], end[1] - taught_truth[1])
        rospy.loginfo('repeated to %.3f m of the taught stop (true poses)', error)
        self.assertLess(error, REPEATED_WITHIN)
        self.assertFalse(self.last('/sim/collision').data, 'the robot touched something')
        with self.lock:
            self.assertLessEqual(self.localization_states, {LocalizationStatus.LOCALIZED})
        localization = self.last('/localization/status')
        self.assertEqual(localization.state, LocalizationStatus.LOCALIZED, localization.message)
        self.wait('/site/status', lambda s: s.routes == drawn + 1, 10.0)


if __name__ == '__main__':
    rospy.init_node('stack_odometry_check')
    rostest.rosrun('x30_bringup', 'stack_odometry', StackOdometry)
