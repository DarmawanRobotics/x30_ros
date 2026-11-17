"""RViz markers of the navigation nodes: route, carrot, corridor, shift and status text."""

from __future__ import annotations

import math

from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray

from x30_navigation.core.follower.state import FollowerState
from x30_navigation.core.path.route import Route
from x30_utils.math import Pose2D
from x30_utils.ros.markers import arrow, color, delete_all, line_strip, marker, text

STATE_COLORS = {
    FollowerState.FOLLOWING: 'green',
    FollowerState.JOINING: 'yellow',
    FollowerState.APPROACHING: 'yellow',
    FollowerState.DWELLING: 'blue',
    FollowerState.GOAL_REACHED: 'blue',
    FollowerState.PAUSED: 'gray',
    FollowerState.INACTIVE: 'gray',
    FollowerState.BLOCKED: 'orange',
}
STOP_COLORS = {'done': 'gray', 'next': 'yellow', 'later': 'blue'}
LIFETIME_SEC = 0.5


def route_markers(header: Header, route: Route, stops: list[tuple[int, str]]) -> MarkerArray:
    """Return the latched route drawing: the path, its ends and the stop points."""
    out = MarkerArray(markers=[delete_all(header, 'route')])
    out.markers.append(line_strip(header, 'route', 0, route.xy, 0.05, color('green', 0.8)))
    for n, (i, kind) in enumerate(stops):
        pose = route.pose(i)
        rgba = color(STOP_COLORS[kind], 0.9)
        out.markers.append(arrow(header, 'route', 10 + 2 * n, pose, 0.45, rgba))
        label = f'S{n + 1} {route.dwell[i]:.0f}s'
        out.markers.append(text(header, 'route', 11 + 2 * n, (pose.x, pose.y, 0.4), label, rgba))
    return out


def follower_markers(header: Header, state: FollowerState, message: str, debug: dict):
    """Return the per-tick drawing: robot state, carrot, nearest point, target, rollout."""
    out = MarkerArray()
    rgba = color(STATE_COLORS.get(state, 'red'))
    pose = debug.get('pose')
    kw = {'lifetime': LIFETIME_SEC}
    if isinstance(pose, Pose2D):
        out.markers.append(
            text(header, 'follower', 0, (pose.x, pose.y, 0.8), state.name, rgba, **kw)
        )
        out.markers.append(_disc(header, 1, pose, 0.35, rgba))
        carrot = debug.get('carrot')
        if carrot is not None:
            points = [(pose.x, pose.y), carrot]
            out.markers.append(
                line_strip(header, 'follower', 2, points, 0.02, color('blue'), **kw)
            )
    nearest = debug.get('nearest')
    if nearest is not None:
        out.markers.append(_sphere(header, 3, nearest, color('orange')))
    target = debug.get('target')
    if isinstance(target, Pose2D):
        out.markers.append(arrow(header, 'follower', 4, target, 0.4, color('yellow'), **kw))
    rollout = debug.get('rollout')
    if rollout is not None and len(rollout):
        out.markers.append(
            line_strip(header, 'follower', 5, rollout, 0.02, color('red', 0.8), **kw)
        )
    if message and isinstance(pose, Pose2D):
        xyz = (pose.x, pose.y, 1.0)
        out.markers.append(text(header, 'follower', 6, xyz, message[:80], color('white'), **kw))
    return out


def avoidance_markers(header: Header, out, emergency_distance: float, cone_deg: float):
    """Return the robot-frame drawing of the emergency cone and the avoidance turn."""
    kw = {'lifetime': 0.3}
    half = math.radians(cone_deg) / 2.0
    arc = [(0.0, 0.0)] + [
        (emergency_distance * math.cos(a), emergency_distance * math.sin(a))
        for a in (-half + 2 * half * k / 12 for k in range(13))
    ]
    arc.append((0.0, 0.0))
    alpha = 0.9 if out.emergency else 0.3
    markers = MarkerArray(
        markers=[line_strip(header, 'avoidance', 0, arc, 0.02, color('red', alpha), **kw)]
    )
    if abs(out.heading_offset) > 0.02 or out.velocity_scale < 0.99:
        pose = Pose2D(0.0, 0.0, out.heading_offset)
        length = 0.3 + 0.5 * out.velocity_scale
        markers.markers.append(arrow(header, 'avoidance', 1, pose, length, color('orange'), **kw))
    return markers


def _disc(header, marker_id: int, pose: Pose2D, radius: float, rgba) -> Marker:
    m = marker(header, 'follower', marker_id, Marker.CYLINDER, lifetime=LIFETIME_SEC)
    m.pose.position.x, m.pose.position.y = pose.x, pose.y
    m.scale.x = m.scale.y = 2 * radius
    m.scale.z = 0.01
    m.color = rgba
    m.color.a = 0.3
    return m


def _sphere(header, marker_id: int, xy, rgba) -> Marker:
    m = marker(header, 'follower', marker_id, Marker.SPHERE, lifetime=LIFETIME_SEC)
    m.pose.position.x, m.pose.position.y = float(xy[0]), float(xy[1])
    m.scale.x = m.scale.y = m.scale.z = 0.1
    m.color = rgba
    return m
