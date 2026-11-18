"""Message <-> core conversions: OccupancyGrid <-> grids, LocalizationStatus, MappingStatus."""

from __future__ import annotations

import array
import math

from nav_msgs.msg import OccupancyGrid
import numpy as np

from x30_mapping.core.grid import LogOddsGrid
from x30_mapping.core.map_io import GridMap
from x30_mapping.core.session import Localization, LocState
from x30_msgs.msg import LocalizationStatus, MappingStatus


def grid_msg(grid: GridMap | None, frame: str, stamp) -> OccupancyGrid:
    """Return a GridMap as an OccupancyGrid (empty when grid is None: no map yet)."""
    msg = OccupancyGrid()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.info.map_load_time = stamp
    if grid is None:
        return msg
    msg.info.resolution = float(grid.resolution)
    msg.info.width, msg.info.height = grid.width, grid.height
    msg.info.origin.position.x = float(grid.origin_x)
    msg.info.origin.position.y = float(grid.origin_y)
    msg.info.origin.orientation.z = math.sin(grid.origin_yaw / 2.0)
    msg.info.origin.orientation.w = math.cos(grid.origin_yaw / 2.0)
    msg.data = array.array('b', np.ascontiguousarray(grid.data, dtype=np.int8).tobytes())
    return msg


def log_odds_msg(grid: LogOddsGrid, frame: str, stamp) -> OccupancyGrid:
    """Return the live grid as an OccupancyGrid."""
    return grid_msg(grid.to_grid_map(), frame, stamp)


def msg_grid(msg: OccupancyGrid) -> GridMap:
    """Return an OccupancyGrid as a GridMap."""
    q = msg.info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    origin = msg.info.origin.position
    return GridMap(data, float(msg.info.resolution), origin.x, origin.y, yaw)


def localization(msg: LocalizationStatus) -> Localization:
    """Return the core view of a LocalizationStatus (unknown states count as NONE)."""
    try:
        state = LocState(int(msg.state))
    except ValueError:
        state = LocState.NONE
    return Localization(state, str(msg.source))


def mapping_status(
    stamp, active: bool, site: str, grid: LogOddsGrid, rejected: int, message: str
) -> MappingStatus:
    """Return a MappingStatus."""
    msg = MappingStatus()
    msg.header.stamp = stamp
    msg.active = active
    msg.site = site
    msg.scans = int(grid.scans)
    msg.rejected = int(rejected)
    msg.resolution = float(grid.resolution)
    msg.width, msg.height = grid.width, grid.height
    msg.known_area_m2 = float(grid.known_area())
    msg.message = message
    return msg
