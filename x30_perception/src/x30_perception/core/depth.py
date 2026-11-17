"""A depth camera's cloud as obstacles: range, fitted floor, cell density, confirmation."""

from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from x30_perception.core.config import PerceptionConfig
from x30_perception.core.filtering import Obstacles, level, self_mask
from x30_utils.math import transform_points
from x30_utils.robot import RobotProfile

MIN_RANGE = 0.05  # m: closer points are invalid (an ordered cloud marks holes with zeros)
FLOOR_WINDOWS = (0.06, 0.04, 0.02)  # m: floor candidates around the nominal floor, then the fit
FLOOR_MIN_POINTS = 100
FLOOR_MIN_SPREAD = 0.1  # m: floor points spread less than this across: a line, not a floor
CONFIRM_MAX_GAP_SEC = 0.5  # an older previous cloud confirms nothing (the motion is unknown)
GRID_MARGIN = 1.0  # m beyond range_max around base_link: the camera's offset from it


@dataclass(frozen=True)
class Floor:
    """The floor seen by the camera: height above the nominal floor h = a x + b y + c (level)."""

    a: float = 0.0
    b: float = 0.0
    c: float = 0.0
    fitted: bool = False
    note: str = ''  # why the nominal floor is kept

    def height(self, xy: np.ndarray) -> np.ndarray:
        """Return the floor height above the nominal floor at (N, 2) level base points."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        return self.a * xy[:, 0] + self.b * xy[:, 1] + self.c

    @property
    def tilt(self) -> float:
        """Angle between the floor and the nominal floor (rad)."""
        return math.atan(math.hypot(self.a, self.b))

    @property
    def mount_error_deg(self) -> tuple[float, float]:
        """Return how far the mount looks off its TF: (pitch, roll) in deg to add."""
        return math.degrees(math.atan(self.a)), -math.degrees(math.atan(self.b))


def fit_floor(xy: np.ndarray, height: np.ndarray, max_tilt: float, max_offset: float) -> Floor:
    """Fit the floor to the points near it; the nominal floor when no clear fit exists."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    x, y = np.ascontiguousarray(xy[:, 0]), np.ascontiguousarray(xy[:, 1])
    height = np.asarray(height, dtype=float).reshape(-1)
    floor = Floor()
    for window in FLOOR_WINDOWS:
        near = np.abs(height - (floor.a * x + floor.b * y + floor.c)) <= window
        count = int(np.count_nonzero(near))
        if count < FLOOR_MIN_POINTS:
            return Floor(note=f'{count} floor points (min {FLOOR_MIN_POINTS})')
        xs, ys, zs = x[near], y[near], height[near]
        mx, my, mz = float(xs.mean()), float(ys.mean()), float(zs.mean())
        xs, ys, zs = xs - mx, ys - my, zs - mz
        cov = np.array([[xs @ xs, xs @ ys], [xs @ ys, ys @ ys]]) / count
        spread = math.sqrt(max(0.0, float(np.linalg.eigvalsh(cov)[0])))
        if spread < FLOOR_MIN_SPREAD:
            return Floor(note=f'floor points on a line ({spread:.2f} m across)')
        a, b = np.linalg.solve(cov, np.array([xs @ zs, ys @ zs]) / count)
        floor = Floor(float(a), float(b), float(mz - a * mx - b * my), True)
    if floor.tilt > max_tilt:
        return Floor(note=f'fit tilted {math.degrees(floor.tilt):.1f} deg')
    if abs(floor.c) > max_offset:
        return Floor(note=f'fit {floor.c:+.2f} m off the floor under the robot')
    return floor


def dense(xy: np.ndarray, cell: float, min_points: int) -> np.ndarray:
    """Return True for the points in a cell (edge cell, m) holding at least min_points points."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    if min_points <= 1 or not len(xy):
        return np.ones(len(xy), dtype=bool)
    keys = np.floor(xy / cell).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    return counts[inverse.reshape(-1)] >= min_points


@dataclass(frozen=True)
class View:
    """Where one cloud saw: azimuths and distances from the camera (level base frame)."""

    x: float
    y: float
    heading: float  # rad, mean azimuth of the points
    low: float  # rad, azimuth span around heading
    high: float
    near: float  # m, horizontal distance span from the camera
    far: float

    @staticmethod
    def of(xy: np.ndarray, origin) -> View | None:
        """Return the view of (N, 2) points seen from origin (x, y); None for no points."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        if not len(xy):
            return None
        dx, dy = xy[:, 0] - origin[0], xy[:, 1] - origin[1]
        azimuth = np.arctan2(dy, dx)
        heading = math.atan2(float(np.sin(azimuth).mean()), float(np.cos(azimuth).mean()))
        rel = _wrap(azimuth - heading)
        r = np.hypot(dx, dy)
        low, high, near, far = float(rel.min()), float(rel.max()), float(r.min()), float(r.max())
        return View(float(origin[0]), float(origin[1]), heading, low, high, near, far)

    def contains(self, xy: np.ndarray) -> np.ndarray:
        """Return True for the (N, 2) points inside the view."""
        xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        dx, dy = xy[:, 0] - self.x, xy[:, 1] - self.y
        rel = _wrap(np.arctan2(dy, dx) - self.heading)
        r = np.hypot(dx, dy)
        return (rel >= self.low) & (rel <= self.high) & (r >= self.near) & (r <= self.far)


def _wrap(angle: np.ndarray) -> np.ndarray:
    return np.mod(np.asarray(angle) + math.pi, 2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class DepthCloud:
    """One depth cloud as obstacles (level base frame), or why it is not usable (reason)."""

    obstacles: Obstacles = field(default_factory=Obstacles.empty)
    points: int = 0  # off the robot body, within range_max
    floor: Floor = field(default_factory=Floor)
    reason: str = ''
    view: View | None = None

    @staticmethod
    def unusable(reason: str) -> DepthCloud:
        """Return a cloud that is not merged (blind camera, missing TF, too old, malformed)."""
        return DepthCloud(reason=reason)


class CellGrid:
    """Occupied cells around base_link, grown by a radius: what confirms the next cloud."""

    def __init__(self, cell: float, half_size: float):
        self.cell = cell
        self.size = math.ceil(2.0 * half_size / cell)
        self.half = self.size * cell / 2.0

    def index(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return the (row, column) of every point and whether it lies on the grid."""
        ij = np.floor((np.asarray(xy, dtype=float).reshape(-1, 2) + self.half) / self.cell)
        inside = ((ij >= 0) & (ij < self.size)).all(axis=1)
        return ij.astype(np.int64), inside

    def grown(self, xy: np.ndarray, radius: float) -> np.ndarray:
        """Return the cells (size x size) within radius cells of a point (a box, not a disc)."""
        ij, inside = self.index(xy)
        r = math.ceil(radius / self.cell - 1e-9)
        occupied = np.zeros((self.size + 2 * r, self.size + 2 * r), dtype=np.int32)
        occupied[ij[inside, 0] + r, ij[inside, 1] + r] = 1
        total = np.zeros((occupied.shape[0] + 1, occupied.shape[1] + 1), dtype=np.int32)
        total[1:, 1:] = occupied.cumsum(axis=0).cumsum(axis=1)
        w = 2 * r + 1
        return (total[w:, w:] - total[:-w, w:] - total[w:, :-w] + total[:-w, :-w]) > 0

    def contains(self, grown: np.ndarray, xy: np.ndarray) -> np.ndarray:
        """Return True for the points on a grown cell."""
        ij, inside = self.index(xy)
        out = np.zeros(len(ij), dtype=bool)
        out[inside] = grown[ij[inside, 0], ij[inside, 1]]
        return out


class DepthFilter:
    """Turn depth camera clouds, one after the other, into obstacle points (pure, no ROS)."""

    def __init__(self, config: PerceptionConfig, robot: RobotProfile):
        self.config = config
        self.robot = robot
        d = config.depth
        self.grid = CellGrid(d.cell_size, d.range_max + d.confirm_radius + GRID_MARGIN)
        self._previous: tuple[float, np.ndarray] | None = None  # (stamp, its grown cells)

    def reset(self):
        """Forget the previous cloud: the next one confirms nothing."""
        self._previous = None

    def process(
        self,
        xyz: np.ndarray,
        sensor_to_base: np.ndarray,
        tilt: np.ndarray | None,
        stamp: float = 0.0,
    ) -> DepthCloud:
        """Return the obstacles of one cloud (camera frame); tilt: base roll/pitch, stamp: s."""
        d, f = self.config.depth, self.config.filter
        xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
        squared = np.einsum('ij,ij->i', xyz, xyz)
        xyz = xyz[(squared >= MIN_RANGE**2) & (squared <= d.range_max**2)]
        base = transform_points(sensor_to_base, xyz)
        base = base[~self_mask(base, self.robot.footprint, f.self_margin)]
        if len(base) < d.min_points:
            self.reset()
            return DepthCloud.unusable(f'camera blind: {len(base)} points (min {d.min_points})')
        if f.level_source == 'tf' and tilt is None:
            self.reset()
            return DepthCloud.unusable('no roll/pitch of the base (odom -> base TF)')
        leveled = level(base, tilt)
        xy, height = leveled[:, :2], leveled[:, 2] + self.robot.base_height
        view = View.of(xy, level(sensor_to_base[:3, 3], tilt)[0, :2])
        floor = Floor(note='floor_fit off')
        if d.floor_fit:
            floor = fit_floor(xy, height, math.radians(d.floor_max_tilt_deg), d.floor_max_offset)
            height = height - floor.height(xy)
        keep = (height >= f.height_min) & (height <= f.height_max)
        xy, height = xy[keep], height[keep]
        keep = dense(xy, d.cell_size, d.cell_points)
        xy, height = xy[keep], height[keep]
        keep = self._confirmed(xy, stamp)
        return DepthCloud(Obstacles(xy[keep], height[keep]), len(base), floor, view=view)

    def _confirmed(self, xy: np.ndarray, stamp: float) -> np.ndarray:
        """Return True for the points near an obstacle cell of the previous cloud; keep these."""
        radius = self.config.depth.confirm_radius
        if radius <= 0.0:
            return np.ones(len(xy), dtype=bool)
        previous, self._previous = self._previous, (stamp, self.grid.grown(xy, radius))
        if previous is None or not 0.0 < stamp - previous[0] <= CONFIRM_MAX_GAP_SEC:
            return np.zeros(len(xy), dtype=bool)
        return self.grid.contains(previous[1], xy)
