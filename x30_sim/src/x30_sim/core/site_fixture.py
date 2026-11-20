"""A ready site drawn from a world: its tags and routes, in the frame of the robot's start."""

# The map frame is where the odometry started (x30_localization's assume_start_origin), as on
# the robot: the world's poses go into the site in the frame of the world's start.

from __future__ import annotations

import os

from x30_sim.core.world import World
from x30_utils.io import PathError, RoutePoint, SiteLayout, valid_name, write_route, write_tags
from x30_utils.math import Pose2D, quat_from_yaw, quat_multiply


def in_start_frame(world: World, pose: Pose2D) -> Pose2D:
    """Return a world pose in the frame of the robot's start (the odometry's, the map's)."""
    return world.start.inverse() @ pose


def site_tags(world: World) -> dict:
    """Return the world's tags (x, y, z, qx, qy, qz, qw) in the frame of the robot's start."""
    turn = quat_from_yaw(-world.start.yaw)
    tags = {}
    for frame, (x, y, z, *q) in world.tags.items():
        at = in_start_frame(world, Pose2D(x, y, 0.0))
        tags[frame] = (at.x, at.y, z, *quat_multiply(turn, q))
    return tags


def site_route(world: World, points: list[RoutePoint]) -> list[RoutePoint]:
    """Return a route of the world in the frame of the robot's start."""
    route = []
    for point in points:
        at = in_start_frame(world, Pose2D(point.x, point.y, point.yaw))
        route.append(RoutePoint(at.x, at.y, at.yaw, point.dwell_sec))
    return route


def write_site(
    world: World,
    layout: SiteLayout,
    routes: dict[str, list[RoutePoint]] | None = None,
    overwrite: bool = False,
) -> bool:
    """Write the tags and routes of a world into a site; False when it has its tags already."""
    bad = [name for name in routes or {} if not valid_name(name)]
    if bad:
        raise PathError(f'invalid route names {bad}')
    if not overwrite and os.path.isfile(layout.tags):
        return False
    os.makedirs(layout.routes, exist_ok=True)
    write_tags(layout.tags, site_tags(world))
    for name, points in (routes or {}).items():
        write_route(os.path.join(layout.routes, f'{name}.csv'), site_route(world, points))
    return True
