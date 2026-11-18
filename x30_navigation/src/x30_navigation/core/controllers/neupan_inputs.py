"""What NeuPAN plans on: the route ahead as its path, the scan's points in the map frame."""

from __future__ import annotations

import math

import numpy as np

from x30_navigation.core.controllers.base import TrackRequest


def route_ahead(request: TrackRequest, window: float, spacing: float, lookahead: float) -> list:
    """Return the route ahead as NeuPAN path points [x, y, yaw, gear=1] (4, 1)."""
    route = request.route
    end = route.length
    start = min(request.s, max(0.0, end - spacing))
    stop = min(end, start + max(window, spacing))
    path = []
    for s in np.append(np.arange(start, stop - 1e-6, spacing), stop):
        x, y = route.shifted_point(float(s), request.offset)
        if lookahead > 0.0 and s < end:
            yaw = route.chord_heading(float(s), min(end, float(s) + lookahead))
        else:
            yaw = float(route.segment_yaw[route.segment_at(float(s))])
        path.append(np.array([[x], [y], [yaw], [1.0]]))
    return path


def map_points(request: TrackRequest, obstacle_range: float):
    """Return the obstacle points within range of base_link in the map frame (2, N), or None."""
    points = np.asarray(request.obstacles, dtype=float).reshape(-1, 2)
    points = points[np.hypot(points[:, 0], points[:, 1]) <= obstacle_range]
    if points.size == 0:
        return None
    pose = request.pose
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return (points @ np.array([[c, s], [-s, c]]) + (pose.x, pose.y)).T
