"""Helpers of the in-process node tests: a fake world and service calls."""

from __future__ import annotations

import itertools
import math

import rospy

from x30_msgs.msg import AvoidanceCommand, SectorScan
from x30_utils.math import make_transform, quat_from_yaw
from x30_utils.ros import COMMAND, X30Node
from x30_utils.testing.harness import wait_for

NAMES = itertools.count()


class World:
    """A helper node: a robot pose on TF (unless hidden), a clear safety scan, avoidance, 20 Hz."""

    def __init__(self, pose=(0.0, 0.0, 0.0)):
        self.node = X30Node('world', param_ns=f'/world_{next(NAMES)}')
        self.pose = pose
        self.blind = False
        self.hidden = False
        self.scan_pub = self.node.create_publisher(SectorScan, 'perception/safety_scan', 10)
        self.avoid_pub = self.node.create_publisher(
            AvoidanceCommand, 'navigation/avoidance', COMMAND
        )
        self.node.every(0.05, self.tick, steady=True)

    def tick(self):
        """Publish the pose, the scan and avoidance."""
        x, y, yaw = self.pose
        stamp = self.node.stamp()
        if not self.hidden:
            pose = make_transform((float(x), float(y), 0.0), quat_from_yaw(yaw))
            self.node.tf_out.send(pose, 'map', 'base_link', stamp)
        n = 36
        scan = SectorScan(angle_min=-math.pi, angle_increment=2 * math.pi / n, range_max=8.0)
        scan.header.frame_id = 'base_link'
        scan.header.stamp = stamp
        scan.ranges = [0.0 if self.blind else 8.0] * n
        self.scan_pub.publish(scan)
        self.avoid_pub.publish(AvoidanceCommand(velocity_scale=1.0))

    def close(self):
        self.node.destroy_node()


def set_param(node, name: str, value, note: str = 'applied', timeout: float = 3.0) -> bool:
    """Set a parameter of node on the server; True once a poll noted it (note: its start)."""
    since = len(node.param_notes)
    rospy.set_param(f'{node.param_namespace}/{name}', value)
    return wait_for(lambda: any(n.startswith(note) for n in node.param_notes[since:]), timeout)


def call(srv_type, name: str, timeout: float = 10.0, **fields):
    """Call a service and return the response."""
    rospy.wait_for_service(name, timeout)
    return rospy.ServiceProxy(name, srv_type)(**fields)
