"""Message <-> core conversions of lidar_sector_node."""

from __future__ import annotations

import numpy as np

from x30_msgs.msg import SectorScan
from x30_perception.core.sectors import SectorLayout


def sector_scan(header, layout: SectorLayout, ranges: np.ndarray) -> SectorScan:
    """Return a SectorScan of ranges over layout (BLOCKED or NaN sectors: obstacle at robot)."""
    msg = SectorScan()
    msg.header = header
    msg.angle_min = float(layout.angle_min)
    msg.angle_increment = float(layout.increment)
    msg.range_max = float(layout.range_max)
    msg.ranges = np.asarray(ranges, dtype=np.float32).tolist()
    return msg
