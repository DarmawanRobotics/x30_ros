"""Log-odds occupancy grid in the map frame that grows in whole cells up to max_size_m."""

from __future__ import annotations

import math

import numpy as np

from x30_mapping.core.config import GridConfig
from x30_mapping.core.map_io import GridMap
from x30_mapping.core.pgm import FREE, OCCUPIED, UNKNOWN
from x30_mapping.core.raycast import nearest_per_bin, ray_cells
from x30_utils.grid import GridSpec


def logit(p: float) -> float:
    """Return the log-odds of a probability."""
    p = min(max(p, 1e-6), 1.0 - 1e-6)
    return math.log(p / (1.0 - p))


def _grow(lo: int, hi: int, size: int, grow: int, limit: int) -> tuple[int, int]:
    """Return (cells added below, above) to cover [lo, hi] on an axis, within limit."""
    below = -lo + grow if lo < 0 else 0
    above = hi - size + 1 + grow if hi >= size else 0
    room = max(limit - size, 0)
    if below + above <= room:
        return below, above
    if below and above:
        share = room * below // (below + above)
        return share, room - share
    return (room, 0) if below else (0, room)


class LogOddsGrid:
    """A growing, size-capped log-odds grid with integrate / occupancy / prior helpers."""

    def __init__(self, config: GridConfig, center: tuple[float, float] = (0.0, 0.0)):
        self.config = config
        self.resolution = float(config.resolution)
        self.l_occupied = logit(config.occupied_threshold)
        self.l_free = logit(config.free_threshold)
        self.grow_cells = max(1, math.ceil(config.grow_m / self.resolution))
        self.max_cells = max(1, int(config.max_size_m / self.resolution))
        cells = min(max(1, math.ceil(config.initial_size_m / self.resolution)), self.max_cells)
        self.origin_x = self.snap(center[0]) - cells // 2 * self.resolution
        self.origin_y = self.snap(center[1]) - cells // 2 * self.resolution
        self.logodds = np.zeros((cells, cells), dtype=np.float32)
        self.scans = 0
        self.capped = False

    def snap(self, value: float) -> float:
        """Round a coordinate down to the cell grid."""
        return math.floor(value / self.resolution) * self.resolution

    @property
    def width(self) -> int:
        """Return the number of columns."""
        return int(self.logodds.shape[1])

    @property
    def height(self) -> int:
        """Return the number of rows."""
        return int(self.logodds.shape[0])

    @property
    def spec(self) -> GridSpec:
        """Return the geometry of the grid."""
        return GridSpec(self.resolution, self.origin_x, self.origin_y, self.width, self.height)

    def ensure(self, xmin: float, ymin: float, xmax: float, ymax: float):
        """Grow the grid (whole cells, cells keep their position, capped) towards the box."""
        res = self.resolution
        col0 = math.floor((xmin - self.origin_x) / res)
        row0 = math.floor((ymin - self.origin_y) / res)
        col1 = math.floor((xmax - self.origin_x) / res)
        row1 = math.floor((ymax - self.origin_y) / res)
        limit_w, limit_h = max(self.max_cells, self.width), max(self.max_cells, self.height)
        left, right = _grow(col0, col1, self.width, self.grow_cells, limit_w)
        bottom, top = _grow(row0, row1, self.height, self.grow_cells, limit_h)
        wanted = col0 < -left or row0 < -bottom
        wanted |= col1 >= self.width + right or row1 >= self.height + top
        self.capped = self.capped or wanted
        if not (left or bottom or right or top):
            return
        grown = np.zeros((self.height + bottom + top, self.width + left + right), np.float32)
        grown[bottom : bottom + self.height, left : left + self.width] = self.logodds
        self.logodds = grown
        self.origin_x -= left * res
        self.origin_y -= bottom * res

    def integrate(
        self,
        sensor_xy: tuple[float, float],
        points_xy: np.ndarray,
        angular_bins: int = 720,
        max_range: float = 15.0,
        min_range: float = 0.0,
    ) -> int:
        """Add one scan: hits (N x 2, map frame) seen from sensor_xy; return the hits used."""
        points_xy = np.asarray(points_xy, dtype=np.float64).reshape(-1, 2)
        sx, sy = float(sensor_xy[0]), float(sensor_xy[1])
        dx, dy = points_xy[:, 0] - sx, points_xy[:, 1] - sy
        dist = np.hypot(dx, dy)
        with np.errstate(invalid='ignore'):  # NaN comparisons warn on numpy < 1.15
            keep = np.isfinite(dist) & (dist <= max_range) & (dist > max(min_range, 1e-6))
        if not keep.any():
            return 0
        points_xy, dx, dy = points_xy[keep], dx[keep], dy[keep]
        margin = self.resolution
        self.ensure(
            min(points_xy[:, 0].min(), sx) - margin,
            min(points_xy[:, 1].min(), sy) - margin,
            max(points_xy[:, 0].max(), sx) + margin,
            max(points_xy[:, 1].max(), sy) + margin,
        )
        spec = self.spec
        cells = spec.cells(points_xy)
        cells = cells[spec.inside(cells)]
        if cells.size == 0:
            return 0
        hit_cells = np.unique(cells[:, 1] * spec.width + cells[:, 0])
        angles, nearest = nearest_per_bin(dx, dy, angular_bins)
        free_cells = ray_cells(spec, sx, sy, angles, nearest - self.resolution)
        if free_cells.size:
            free_cells = np.setdiff1d(free_cells, hit_cells, assume_unique=True)
        cfg = self.config
        flat = self.logodds.reshape(-1)
        flat[free_cells] = np.maximum(flat[free_cells] + cfg.log_odds_miss, cfg.log_odds_min)
        flat[hit_cells] = np.minimum(flat[hit_cells] + cfg.log_odds_hit, cfg.log_odds_max)
        self.scans += 1
        return len(cells)

    def occupancy(self) -> np.ndarray:
        """Return OccupancyGrid values (int8, row 0 at origin_y): trinary."""
        data = np.full(self.logodds.shape, UNKNOWN, dtype=np.int8)
        data[self.logodds > self.l_occupied] = OCCUPIED
        data[self.logodds < self.l_free] = FREE
        return data

    def occupied_mask(self) -> np.ndarray:
        """Return a boolean mask of the occupied cells."""
        return self.logodds > self.l_occupied

    def known_area(self) -> float:
        """Return the free + occupied area in m^2."""
        known = (self.logodds > self.l_occupied) | (self.logodds < self.l_free)
        return float(np.count_nonzero(known)) * self.resolution**2

    def to_grid_map(self) -> GridMap:
        """Return the grid as a GridMap (trinary)."""
        return GridMap(self.occupancy(), self.resolution, self.origin_x, self.origin_y, 0.0)

    def load_prior(self, grid: GridMap, occupied: float = 1.5, free: float = -1.5):
        """Replace the grid with a saved map (same resolution), known cells as the prior."""
        if grid.origin_yaw != 0.0:
            raise ValueError('maps with a rotated origin cannot be continued')
        if not math.isclose(grid.resolution, self.resolution, rel_tol=1e-6):
            raise ValueError(
                f'map resolution {grid.resolution} differs from the mapper {self.resolution}'
            )
        self.origin_x, self.origin_y = grid.origin_x, grid.origin_y
        data = np.asarray(grid.data)
        self.logodds = np.zeros(data.shape, dtype=np.float32)
        self.logodds[data >= 50] = occupied
        self.logodds[(data >= 0) & (data < 50)] = free
        self.scans = 0
        self.capped = False
