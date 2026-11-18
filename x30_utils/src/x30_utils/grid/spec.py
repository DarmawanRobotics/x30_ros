"""Geometry of a 2D grid map: cells of resolution metres, cell (0, 0) at origin (REP-105)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class GridSpec:
    """Size and placement of a grid; row index = y, column index = x."""

    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def cells(self, xy: np.ndarray) -> np.ndarray:
        """Return integer (col, row) of (N, 2) world points (floor, also for negatives)."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        cols = np.floor((xy[:, 0] - self.origin_x) / self.resolution).astype(np.int64)
        rows = np.floor((xy[:, 1] - self.origin_y) / self.resolution).astype(np.int64)
        return np.column_stack((cols, rows))

    def centers(self, cells: np.ndarray) -> np.ndarray:
        """Return the world (x, y) of (N, 2) cell centres."""
        cells = np.asarray(cells, dtype=float).reshape(-1, 2)
        return np.column_stack(
            (
                self.origin_x + (cells[:, 0] + 0.5) * self.resolution,
                self.origin_y + (cells[:, 1] + 0.5) * self.resolution,
            )
        )

    def inside(self, cells: np.ndarray) -> np.ndarray:
        """Return a mask of cells inside the grid."""
        cells = np.asarray(cells).reshape(-1, 2)
        return (
            (cells[:, 0] >= 0)
            & (cells[:, 0] < self.width)
            & (cells[:, 1] >= 0)
            & (cells[:, 1] < self.height)
        )

    @property
    def size_m(self) -> tuple[float, float]:
        """Return (width, height) in metres."""
        return self.width * self.resolution, self.height * self.resolution
