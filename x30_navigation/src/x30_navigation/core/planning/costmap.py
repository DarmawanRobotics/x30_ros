"""Planning costmap from an occupancy grid, inflated with radii from the robot footprint."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.config.planner import CostmapConfig
from x30_utils.grid import GridSpec, distance_field
from x30_utils.robot import Footprint

INFLATION_WEIGHT = 5.0


@dataclass(frozen=True)
class MapGrid:
    """An occupancy grid: values -1 (unknown) or 0..100, row index = y."""

    data: np.ndarray
    spec: GridSpec


@dataclass(frozen=True)
class Radii:
    """Clearance radii around base_link (m), from the footprint and the margin."""

    lethal: float
    circumscribed: float
    inflation: float
    margin: float

    @staticmethod
    def of(footprint: Footprint, config: CostmapConfig) -> Radii:
        """Return the radii for a footprint."""
        lethal = footprint.inscribed_radius + config.margin
        circumscribed = footprint.circumscribed_radius + config.margin
        inflation = max(config.inflation_radius, circumscribed)
        return Radii(lethal, circumscribed, inflation, config.margin)


@dataclass(frozen=True, eq=False)
class Costmap:
    """Planning grid: cost per cell length (inf: lethal), obstacles and their distance."""

    spec: GridSpec
    cost: np.ndarray
    obstacles: np.ndarray
    distance: np.ndarray
    radii: Radii

    @property
    def shape(self) -> tuple[int, int]:
        """Return (rows, cols)."""
        return self.cost.shape

    @property
    def resolution(self) -> float:
        """Cell size (m)."""
        return self.spec.resolution

    def cell(self, x: float, y: float) -> tuple[int, int]:
        """Return the (row, col) of a map point (may be outside the grid)."""
        col, row = self.spec.cells(np.array([[x, y]]))[0]
        return int(row), int(col)

    def center(self, row: int, col: int) -> tuple[float, float]:
        """Return the map point at the centre of a cell."""
        x, y = self.spec.centers(np.array([[col, row]]))[0]
        return float(x), float(y)

    def inside(self, row: int, col: int) -> bool:
        """Return True when the cell is in the grid."""
        return 0 <= row < self.shape[0] and 0 <= col < self.shape[1]

    def lethal(self, row: int, col: int) -> bool:
        """Return True outside the grid or inside an inflated obstacle."""
        return not self.inside(row, col) or not math.isfinite(self.cost[row, col])

    def cost_at(self, x: float, y: float) -> float:
        """Return the cost at a map point (inf outside the grid)."""
        row, col = self.cell(x, y)
        return float(self.cost[row, col]) if self.inside(row, col) else math.inf

    def with_lethal(self, cells) -> Costmap:
        """Return a copy with the given (row, col) cells made lethal."""
        cost = self.cost.copy()
        for row, col in cells:
            if self.inside(row, col):
                cost[row, col] = math.inf
        return Costmap(self.spec, cost, self.obstacles, self.distance, self.radii)


def obstacle_mask(data: np.ndarray, unknown_is_obstacle: bool, threshold: int) -> np.ndarray:
    """Return True where occupancy values block the robot."""
    values = np.asarray(data, dtype=np.int16)
    mask = values >= threshold
    if unknown_is_obstacle:
        mask |= values < 0
    return mask


def max_pool(mask: np.ndarray, factor: int) -> np.ndarray:
    """Return a coarser mask: a coarse cell is set when any of its fine cells is."""
    if factor <= 1:
        return mask.copy()
    rows, cols = -(-mask.shape[0] // factor), -(-mask.shape[1] // factor)
    padded = np.zeros((rows * factor, cols * factor), dtype=bool)
    padded[: mask.shape[0], : mask.shape[1]] = mask
    return padded.reshape(rows, factor, cols, factor).any(axis=(1, 3))


def stamp_points(mask: np.ndarray, spec: GridSpec, points: np.ndarray) -> np.ndarray:
    """Return a copy of the mask with the cells of map points set."""
    out = mask.copy()
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    if points.size == 0:
        return out
    cells = spec.cells(points)
    ok = spec.inside(cells)
    out[cells[ok, 1], cells[ok, 0]] = True
    return out


def build_costmap(
    grid: MapGrid, footprint: Footprint, config: CostmapConfig, extra: np.ndarray | None = None
) -> Costmap:
    """Return the inflated planning costmap of an occupancy grid."""
    factor = max(1, round(config.resolution / grid.spec.resolution))
    res = grid.spec.resolution * factor
    mask = obstacle_mask(grid.data, config.unknown_is_obstacle, config.occupied_threshold)
    mask = max_pool(mask, factor)
    spec = GridSpec(res, grid.spec.origin_x, grid.spec.origin_y, mask.shape[1], mask.shape[0])
    if extra is not None:
        mask = stamp_points(mask, spec, extra)
    radii = Radii.of(footprint, config)
    distance = distance_field(mask, res, radii.inflation + res).astype(float)
    return Costmap(
        spec, inflation_cost(distance, radii, config.cost_scaling), mask, distance, radii
    )


def inflation_cost(distance: np.ndarray, radii: Radii, scaling: float) -> np.ndarray:
    """Return 1 + a cost decaying from the lethal radius to the inflation radius (inf inside)."""
    cost = np.ones(distance.shape)
    lethal = distance <= radii.lethal + 1e-6
    near = ~lethal & (distance < radii.inflation)
    span = max(radii.inflation - radii.lethal, 1e-6)
    edge = math.exp(-scaling * span)
    decay = (np.exp(-scaling * (distance[near] - radii.lethal)) - edge) / max(1.0 - edge, 1e-9)
    cost[near] = 1.0 + INFLATION_WEIGHT * np.clip(decay, 0.0, 1.0)
    cost[lethal] = math.inf
    return cost
