"""RViz markers of the sector scans: hits per sector, depth camera, remembered points, blocked."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray

from x30_perception.core.pipeline import ScanResult
from x30_perception.core.sectors import SectorLayout
from x30_utils.ros.markers import color, marker, text

LIFETIME_SEC = 0.5


def hits(layout: SectorLayout, ranges: np.ndarray) -> np.ndarray:
    """Return the (N, 2) points of the sectors with an obstacle inside range_max."""
    ranges = np.asarray(ranges, dtype=float)
    hit = (ranges > 0.0) & (ranges < layout.range_max - 1e-3)
    theta = layout.angle_min + (np.flatnonzero(hit) + 0.5) * layout.increment
    return np.column_stack((ranges[hit] * np.cos(theta), ranges[hit] * np.sin(theta)))


def _spheres(header, ns: str, marker_id: int, xy: np.ndarray, rgba, size: float) -> Marker:
    m = marker(header, ns, marker_id, Marker.SPHERE_LIST, lifetime=LIFETIME_SEC)
    m.scale.x = m.scale.y = m.scale.z = size
    m.color = rgba
    m.points = [Point(x=float(x), y=float(y), z=0.05) for x, y in xy]
    return m


def scan_markers(
    header, front: SectorLayout, safety: SectorLayout, result: ScanResult
) -> MarkerArray:
    """Return the markers of one processed cloud (header in the base frame)."""
    out = MarkerArray()
    out.markers.append(
        _spheres(header, 'front', 0, hits(front, result.front), color('yellow', 0.9), 0.08)
    )
    out.markers.append(
        _spheres(header, 'safety', 1, hits(safety, result.safety), color('red', 0.9), 0.06)
    )
    out.markers.append(
        _spheres(header, 'memory', 2, result.remembered, color('orange', 0.6), 0.05)
    )
    camera = hits(safety, result.depth) if len(result.depth) else np.empty((0, 2))
    out.markers.append(_spheres(header, 'depth', 5, camera, color('blue', 0.9), 0.07))
    if result.blocked_reason:
        label = f'BLOCKED: {result.blocked_reason}'
        out.markers.append(
            text(header, 'status', 3, (0.0, 0.0, 1.0), label, color('red'), lifetime=LIFETIME_SEC)
        )
        ring = marker(header, 'blocked', 4, Marker.LINE_STRIP, lifetime=LIFETIME_SEC)
        ring.scale.x = 0.03
        ring.color = color('red')
        angles = np.linspace(-math.pi, math.pi, 37)
        ring.points = [Point(x=0.6 * math.cos(a), y=0.6 * math.sin(a), z=0.05) for a in angles]
        out.markers.append(ring)
    return out
