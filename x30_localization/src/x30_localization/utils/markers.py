"""RViz markers of the known tags: a plate, its axes, a label and a drop line to the floor."""

from __future__ import annotations

import math

from geometry_msgs.msg import Point, Vector3
import numpy as np
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray

from x30_utils.math import compose, make_transform, quat_from_rpy
from x30_utils.ros.convert import matrix_to_pose
from x30_utils.ros.markers import delete_all, marker, text

NS = 'tags'
AXES = (
    (np.eye(4), ColorRGBA(r=1.0, g=0.1, b=0.1, a=1.0)),
    (make_transform(quat=quat_from_rpy(0.0, 0.0, math.pi / 2)), ColorRGBA(g=1.0, a=1.0)),
    (make_transform(quat=quat_from_rpy(0.0, -math.pi / 2, 0.0)), ColorRGBA(b=1.0, a=1.0)),
)
LABEL = ColorRGBA(r=1.0, g=0.3, b=0.9, a=1.0)


def tag_markers(tags: dict[str, np.ndarray], frame: str, size: float, stamp) -> MarkerArray:
    """Return the markers of every tag (map->tag 4x4), clearing the previous ones."""
    header = Header(frame_id=frame, stamp=stamp)
    out = [delete_all(header, NS)]
    for i, (name, m) in enumerate(sorted(tags.items())):
        base = 10 * i
        plate = marker(header, NS, base, Marker.CUBE)
        plate.pose = matrix_to_pose(m)
        plate.scale = Vector3(x=size, y=size, z=0.005)
        plate.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        out.append(plate)
        for k, (rotation, color) in enumerate(AXES):
            axis = marker(header, NS, base + 1 + k, Marker.ARROW)
            axis.pose = matrix_to_pose(compose(m, rotation))
            axis.scale = Vector3(x=0.25, y=0.02, z=0.02)
            axis.color = color
            out.append(axis)
        x, y, z = (float(v) for v in m[:3, 3])
        out.append(text(header, NS, base + 4, (x, y, z + 0.25), name, LABEL))
        drop = marker(header, NS, base + 5, Marker.LINE_LIST)
        drop.scale.x = 0.01
        drop.color = ColorRGBA(r=1.0, g=0.3, b=0.9, a=0.6)
        drop.points = [Point(x=x, y=y, z=z), Point(x=x, y=y, z=0.0)]
        out.append(drop)
    return MarkerArray(markers=out)
