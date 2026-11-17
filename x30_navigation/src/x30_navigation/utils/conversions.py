"""Messages <-> core types."""

from __future__ import annotations

import math

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np

from x30_msgs.msg import (
    AvoidanceCommand,
    NavigationStatus,
    SectorScan,
    StopPointEvent,
    Waypoint,
    WaypointPath,
)
from x30_navigation.core.avoidance.base import AvoidanceOut
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.follower.state import Report, StopEvent
from x30_navigation.core.planning.costmap import Costmap, MapGrid
from x30_navigation.core.scan import SectorScanData
from x30_utils.grid import GridSpec
from x30_utils.io import RoutePoint
from x30_utils.math import Pose2D
from x30_utils.ros.convert import msg_yaw, pose2d_to_pose


def scan_data(msg: SectorScan) -> SectorScanData:
    """Return the core view of a SectorScan."""
    return SectorScanData.of(msg.angle_min, msg.angle_increment, msg.range_max, msg.ranges)


def route_points(waypoints) -> list[RoutePoint]:
    """Return RoutePoints of Waypoint messages; raise ValueError on non-finite values."""
    points = []
    for w in waypoints:
        values = (float(w.x), float(w.y), float(w.yaw), float(w.dwell_sec))
        if not all(math.isfinite(v) for v in values):
            raise ValueError('a waypoint is not finite')
        points.append(RoutePoint(values[0], values[1], values[2], max(0.0, values[3])))
    return points


def waypoints(points: list[RoutePoint]) -> list[Waypoint]:
    """Return Waypoint messages of RoutePoints."""
    return [
        Waypoint(x=float(p.x), y=float(p.y), yaw=float(p.yaw), dwell_sec=float(p.dwell_sec))
        for p in points
    ]


def waypoint_path(points: list[RoutePoint], frame: str, stamp) -> WaypointPath:
    """Return a WaypointPath."""
    msg = WaypointPath(waypoints=waypoints(points))
    msg.header.frame_id, msg.header.stamp = frame, stamp
    return msg


def nav_path(points: list[RoutePoint], frame: str, stamp) -> Path:
    """Return a nav_msgs/Path for RViz."""
    msg = Path()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    for p in points:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = pose2d_to_pose(Pose2D(float(p.x), float(p.y), float(p.yaw)))
        msg.poses.append(pose)
    return msg


def twist(command: Command) -> Twist:
    """Return a Twist of a body command."""
    msg = Twist()
    msg.linear.x, msg.linear.y, msg.angular.z = (
        float(command.vx),
        float(command.vy),
        float(command.wz),
    )
    return msg


def avoidance_out(msg: AvoidanceCommand) -> AvoidanceOut:
    """Return the core view of an AvoidanceCommand."""
    return AvoidanceOut(
        float(msg.heading_offset),
        float(msg.yaw_rate_bias),
        float(msg.lateral_bias),
        float(msg.velocity_scale),
        bool(msg.emergency),
    )


def avoidance_msg(out: AvoidanceOut, header) -> AvoidanceCommand:
    """Return an AvoidanceCommand."""
    return AvoidanceCommand(
        header=header,
        heading_offset=float(out.heading_offset),
        yaw_rate_bias=float(out.yaw_rate_bias),
        lateral_bias=float(out.lateral_bias),
        velocity_scale=float(out.velocity_scale),
        emergency=bool(out.emergency),
    )


def status_msg(report: Report, frame: str, stamp) -> NavigationStatus:
    """Return a NavigationStatus."""
    msg = NavigationStatus(
        state=report.state.value,
        pause_reasons=list(report.pause_reasons),
        index=int(report.index),
        count=int(report.count),
        remaining=float(report.remaining),
        cross_track=float(report.cross_track),
        message=report.message,
    )
    msg.header.frame_id, msg.header.stamp = frame, stamp
    return msg


def stop_event_msg(event: StopEvent, frame: str, stamp) -> StopPointEvent:
    """Return a StopPointEvent."""
    msg = StopPointEvent(
        event=event.kind.value,
        index=int(event.index),
        x=float(event.pose.x),
        y=float(event.pose.y),
        yaw=float(event.pose.yaw),
        dwell_sec=float(event.dwell_sec),
        message=event.message,
    )
    msg.header.frame_id, msg.header.stamp = frame, stamp
    return msg


def map_grid(msg: OccupancyGrid) -> MapGrid:
    """Return the core grid of an OccupancyGrid; raise ValueError when it cannot be used."""
    width, height = int(msg.info.width), int(msg.info.height)
    if width * height == 0 or len(msg.data) != width * height:
        raise ValueError('the map is empty or its size does not match its data')
    if abs(msg_yaw(msg.info.origin.orientation)) > 1e-6:
        raise ValueError('maps with a rotated origin are not supported')
    data = np.asarray(msg.data, dtype=np.int8).reshape(height, width)
    resolution = round(float(msg.info.resolution), 6)
    origin = msg.info.origin.position
    return MapGrid(data, GridSpec(resolution, float(origin.x), float(origin.y), width, height))


def costmap_msg(costmap: Costmap, frame: str, stamp) -> OccupancyGrid:
    """Return a costmap as an OccupancyGrid: lethal 100, cost scaled to 0..99."""
    cost = costmap.cost
    data = np.where(np.isfinite(cost), np.clip((cost - 1.0) * 15.0, 0, 99), 100)
    msg = OccupancyGrid()
    msg.header.frame_id, msg.header.stamp = frame, stamp
    spec = costmap.spec
    msg.info.resolution = float(spec.resolution)
    msg.info.width, msg.info.height = spec.width, spec.height
    msg.info.origin.position.x, msg.info.origin.position.y = spec.origin_x, spec.origin_y
    msg.info.origin.orientation.w = 1.0
    msg.data = data.astype(np.int8).ravel().tolist()
    return msg
