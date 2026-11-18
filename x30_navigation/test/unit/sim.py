"""Shared helpers of the unit tests: routes and a kinematic robot."""

from __future__ import annotations

import math
import random

import numpy as np

from x30_navigation.core.controllers import TrackRequest
from x30_navigation.core.path.processing import process_route
from x30_navigation.core.path.progress import RouteProgress
from x30_navigation.core.path.route import Route
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D

LOOP = [(0, 0), (10, 0), (10, 4), (0, 4), (0, 0)]
OUT_AND_BACK = [(0, 0), (10, 0), (10, 0.8), (0, 0.8)]


def straight_route(x0: float, x1: float, y: float = 0.0, spacing: float = 0.1):
    """Return (x, y) points of a straight route along x."""
    n = round(abs(x1 - x0) / spacing)
    return [(x0 + (x1 - x0) * i / n, y) for i in range(n + 1)]


def noisy_route(corners, spacing=0.2, noise=0.03, seed=0) -> list[RoutePoint]:
    """Return recorded-route-like points along a corner polyline."""
    rng = random.Random(seed)
    out = []
    for (ax, ay), (bx, by) in zip(corners, corners[1:]):
        n = int(math.hypot(bx - ax, by - ay) / spacing)
        for i in range(n):
            t = i / n
            out.append(
                RoutePoint(
                    ax + t * (bx - ax) + rng.gauss(0, noise),
                    ay + t * (by - ay) + rng.gauss(0, noise),
                )
            )
    out.append(RoutePoint(float(corners[-1][0]), float(corners[-1][1])))
    return out


def route_of(points, stops=None, spacing=None) -> Route:
    """Build a Route from (x, y) tuples or RoutePoints; stops: {index: dwell_sec}."""
    pts = [p if isinstance(p, RoutePoint) else RoutePoint(*map(float, p[:2])) for p in points]
    if spacing:
        pts, _ = process_route(pts, spacing, 0.05, 0.4)
    pts = [
        RoutePoint(p.x, p.y, p.yaw, (stops or {}).get(i, p.dwell_sec)) for i, p in enumerate(pts)
    ]
    return Route.from_points(pts)


def step(pose: Pose2D, vx: float, vy: float, wz: float, dt: float) -> Pose2D:
    """Move a pose by a body-frame command for dt."""
    yaw = pose.yaw + wz * dt
    c, s = math.cos(yaw), math.sin(yaw)
    return Pose2D(pose.x + (vx * c - vy * s) * dt, pose.y + (vx * s + vy * c) * dt, yaw)


def drive(controller, route: Route, speed=0.4, dt=0.05, max_time=200.0, start=None, offset=0.0):
    """Follow a route with a controller on a kinematic robot; return (finished, errors)."""
    pose = start or Pose2D(0.0, 0.1, 0.0)
    progress = RouteProgress(route)
    progress.locate(pose)
    controller.reset()
    errors, t = [], 0.0
    end = route.xy[-1]
    while t < max_time:
        progress.update(pose, offset, 2.0 * speed * dt + 0.05)
        if progress.remaining < 0.3 and math.hypot(end[0] - pose.x, end[1] - pose.y) < 0.3:
            return True, errors
        command = controller.update(TrackRequest(route, progress.s, pose, speed, dt, offset))
        pose = step(pose, command.vx, command.vy, command.wz, dt)
        i = progress.index
        near = route.xy[max(0, i - 30) : i + 30]
        errors.append(float(np.min(np.hypot(near[:, 0] - pose.x, near[:, 1] - pose.y))))
        t += dt
    return False, errors


def line(x0, x1, y, spacing=0.05):
    """Points of a wall along x at y."""
    return np.array([(x, y) for x in np.arange(x0, x1 + 1e-9, spacing)])


def block(x0, x1, y0, y1, spacing=0.05):
    """The outline of a solid block (a shelf, a counter) from x0 to x1, y0 to y1."""
    xs, ys = np.arange(x0, x1 + 1e-9, spacing), np.arange(y0, y1 + 1e-9, spacing)
    edges = (
        [(x, y0) for x in xs],
        [(x, y1) for x in xs],
        [(x0, y) for y in ys],
        [(x1, y) for y in ys],
    )
    return np.vstack(edges)
