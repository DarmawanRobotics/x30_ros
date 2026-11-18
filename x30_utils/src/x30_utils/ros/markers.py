"""Small builders for RViz markers."""

from __future__ import annotations

from geometry_msgs.msg import Point, Vector3
import rospy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from x30_utils.ros.convert import pose2d_to_pose

COLORS = {
    'red': (0.9, 0.2, 0.2),
    'green': (0.2, 0.8, 0.3),
    'blue': (0.2, 0.5, 0.95),
    'yellow': (0.95, 0.8, 0.2),
    'orange': (1.0, 0.55, 0.1),
    'gray': (0.6, 0.6, 0.6),
    'white': (1.0, 1.0, 1.0),
}


def color(name: str = 'white', alpha: float = 1.0) -> ColorRGBA:
    """Return a named color."""
    r, g, b = COLORS[name]
    return ColorRGBA(r=r, g=g, b=b, a=alpha)


def marker(header, ns: str, marker_id: int, kind: int, *, lifetime: float = 0.0) -> Marker:
    """Return an ADD marker with identity pose."""
    m = Marker()
    m.header = header
    m.ns, m.id, m.type, m.action = ns, marker_id, kind, Marker.ADD
    m.pose.orientation.w = 1.0
    m.lifetime = rospy.Duration.from_sec(lifetime)
    return m


def line_strip(header, ns: str, marker_id: int, xy, width: float, rgba: ColorRGBA, **kw) -> Marker:
    """Return a line strip through (x, y) points."""
    m = marker(header, ns, marker_id, Marker.LINE_STRIP, **kw)
    m.scale.x = width
    m.color = rgba
    m.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in xy]
    return m


def arrow(header, ns: str, marker_id: int, pose2d, length: float, rgba: ColorRGBA, **kw) -> Marker:
    """Return an arrow at a planar pose."""
    m = marker(header, ns, marker_id, Marker.ARROW, **kw)
    m.pose = pose2d_to_pose(pose2d, z=0.05)
    m.scale = Vector3(x=length, y=0.05, z=0.05)
    m.color = rgba
    return m


def text(header, ns: str, marker_id: int, xyz, label: str, rgba: ColorRGBA, **kw) -> Marker:
    """Return a text label."""
    m = marker(header, ns, marker_id, Marker.TEXT_VIEW_FACING, **kw)
    m.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))
    m.scale.z = 0.15
    m.color = rgba
    m.text = label
    return m


def delete_all(header, ns: str = '') -> Marker:
    """Return a marker clearing every marker (of ns when given)."""
    m = Marker()
    m.header, m.ns, m.action = header, ns, Marker.DELETEALL
    return m
