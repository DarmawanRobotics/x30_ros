"""Oriented footprint collision checks on the planning grid."""

from __future__ import annotations

import math

import numpy as np

from x30_navigation.core.planning.costmap import Costmap
from x30_utils.math import Pose2D
from x30_utils.robot import Footprint


class FootprintChecker:
    """Does the robot fit at a pose: the footprint at that heading against the obstacle cells."""

    def __init__(self, costmap: Costmap, footprint: Footprint, margin: float):
        self.costmap = costmap
        self.footprint = footprint
        self.margin = margin
        self.reach = math.ceil(costmap.radii.circumscribed / costmap.resolution) + 1

    def collides(self, pose: Pose2D) -> bool:
        """Return True when the footprint (grown by the margin) covers an obstacle cell."""
        cm = self.costmap
        row, col = cm.cell(pose.x, pose.y)
        if not cm.inside(row, col):
            return True
        distance = cm.distance[row, col]
        if distance <= cm.radii.lethal:
            return True
        if distance > cm.radii.circumscribed + cm.resolution:
            return False
        r0, c0 = max(row - self.reach, 0), max(col - self.reach, 0)
        window = cm.obstacles[r0 : row + self.reach + 1, c0 : col + self.reach + 1]
        cells = np.argwhere(window)
        if cells.size == 0:
            return False
        centers = cm.spec.centers(cells[:, ::-1] + (c0, r0))
        local = pose.inverse().apply(centers)
        return bool(np.any(self.footprint.contains(local, self.margin)))

    def first_collision(self, poses: list[Pose2D], skip: int = 0) -> int | None:
        """Return the index of the first colliding pose after the first skip, or None."""
        for i in range(skip, len(poses)):
            if self.collides(poses[i]):
                return i
        return None
