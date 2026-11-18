"""Path shift for holonomic robots: slide sideways around obstacles, keep the heading."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_navigation.core.config.shift import PathShiftConfig
from x30_navigation.core.path.route import Route
from x30_navigation.core.shift.corridor import Corridor
from x30_navigation.core.sweep import ALLOWANCE_M
from x30_utils.robot import RobotProfile
from x30_utils.safety import stopping_speed


@dataclass(frozen=True)
class ShiftOutput:
    """What path shift asks for this tick."""

    offset: float
    target: float
    speed_limit: float
    free_distance: float
    clear: bool
    blind: bool = False
    slide: float = 0.0  # m/s, +left: sideways toward the offset while forward is held
    searching: bool = False  # offsets up to search_offset (patience ran out)


HOLD_SPEED = 0.05  # m/s: a lower speed limit holds the forward motion (slide only)
SLIDE_TOLERANCE = 0.02  # m: closer to the committed offset needs no slide
SLIDE_GAIN = 1.5  # 1/s: slide speed per metre from the committed offset
SLIDE_MIN = 0.05  # m/s: slowest slide (beats the robot deadband)


class PathShift:
    """Offset choice and speed limit along one route."""

    def __init__(self, route: Route, robot: RobotProfile, config: PathShiftConfig):
        self.config = config
        self.robot = robot
        fp = robot.footprint
        n = round(config.max_offset / config.step)
        wide = max(n, round(config.search_offset / config.step))
        reach = max(config.max_offset, wide * config.step) + fp.half_width + config.clearance
        self.corridor = Corridor(route, fp, config.look_ahead, config.clearance, reach + 0.2)
        k = np.arange(-wide, wide + 1)
        self.candidates = k * config.step  # up to search_offset
        self.narrow = np.abs(k) <= n  # up to max_offset
        self.offset = 0.0
        self.target = 0.0
        self.return_since: float | None = None
        self.waited = 0.0  # s held with no clear offset
        self.searching = False
        self.centred = False  # in the middle of a tight lane

    def reset(self):
        """Back onto the route itself."""
        self.offset = self.target = 0.0
        self.return_since = None
        self.waited, self.searching, self.centred = 0.0, False, False

    def choose(
        self, s_pts, d_pts, s_robot: float, lateral: float | None = None
    ) -> tuple[float, float, bool]:
        """Return (offset, free distance, clear) for the obstacles in route coordinates."""
        cfg, fp = self.config, self.robot.footprint
        grid = self.candidates if self.searching else self.candidates[self.narrow]
        current = self.target
        start = self.offset if lateral is None else lateral
        # the current target keeps its own clearance (switch_margin for a change); a target off
        # the grid (a lane's middle) too
        offsets = np.append(grid, current)
        extra = np.append(np.full(len(grid), cfg.switch_margin), 0.0)
        low, high = self.corridor.bounds(
            s_pts, d_pts, s_robot, start, beside_gap=fp.half_width + cfg.clearance
        )
        low, high = min(low, start) - 1e-9, max(high, start) + 1e-9  # away from it: always
        reachable = (offsets >= low) & (offsets <= high)
        reachable &= np.abs(offsets) <= float(np.abs(grid).max()) + 1e-9
        free = self.corridor.free_distances(s_pts, d_pts, s_robot, offsets, extra)
        free[~reachable] = 0.0
        clear = np.isinf(free)
        if clear.any():
            cost = np.abs(offsets) + cfg.switch_cost * np.abs(offsets - current)
            cost[~clear] = math.inf
            i = int(cost.argmin())
            return float(offsets[i]), math.inf, True
        # no offset with its clearance: a reachable line with nothing ahead drives on, points
        # beside are passed parallel (the current one first, then the robot's own); else the
        # line with the most room ahead
        offsets, reachable = np.append(offsets, start), np.append(reachable, True)
        free = self.corridor.free_distances(s_pts, d_pts, s_robot, offsets, passing=True)
        free[~reachable] = 0.0
        best = free.max()
        ties = np.flatnonzero(reachable & (free >= best - 1e-6))
        keep = [i for i in (len(grid), len(offsets) - 1) if i in ties]
        i = keep[0] if keep else int(ties[np.abs(offsets[ties] - current).argmin()])
        return float(offsets[i]), float(free[i]), bool(np.isinf(free[i]))

    def update(
        self,
        s_robot: float,
        lateral: float,
        points: np.ndarray,
        blind: bool,
        now: float,
        dt: float,
        hold: bool = False,
        forward: bool = True,
        held: bool = False,
    ) -> ShiftOutput:
        """Advance one tick (s_robot: progress, lateral: signed offset, points: map frame)."""
        if blind:
            return ShiftOutput(
                self.offset, self.target, 0.0, 0.0, False, True, 0.0, self.searching
            )
        s_pts, d_pts = self.corridor.coordinates(s_robot, points)
        centre = None if hold else self._lane_centre(s_pts, d_pts, s_robot, lateral)
        if hold:
            self.target, self.return_since, clear = 0.0, None, True
            self.waited, self.searching, self.centred = 0.0, False, False
        elif centre is not None:
            self.target, self.return_since, clear = centre, None, True
        else:
            proposal, _, clear = self.choose(s_pts, d_pts, s_robot, lateral)
            self._accept(proposal, now)
        self._commit(lateral, dt)
        offsets = [self.target, self.offset, lateral]
        free = self.corridor.free_distances(s_pts, d_pts, s_robot, offsets, passing=True)
        free = float(free.min())
        limit = self.speed_limit(free) if forward else 0.0
        if not hold:
            self._patience(clear, held or limit < HOLD_SPEED, dt)
        slide = self.slide(lateral) if limit < HOLD_SPEED else 0.0
        return ShiftOutput(
            self.offset, self.target, limit, free, clear, False, slide, self.searching
        )

    def _lane_centre(self, s_pts, d_pts, s_robot: float, lateral: float) -> float | None:
        """Return the middle of the tight lane the robot drives in, or None."""
        cfg, fp = self.config, self.robot.footprint
        bounds = self.corridor.bounds
        low, high = bounds(s_pts, d_pts, s_robot, lateral, ahead_gap=fp.half_width + cfg.clearance)
        if math.isfinite(low) and math.isfinite(high):
            touch = fp.half_width + ALLOWANCE_M
            beside_low, beside_high = bounds(s_pts, d_pts, s_robot, lateral, beside_gap=touch)
            low, high = max(low, beside_low), min(high, beside_high)
        slack = (high - low) / 2.0  # beyond the clearance on either side, in the middle
        centre = (low + high) / 2.0 if math.isfinite(slack) else math.inf
        limit = max(cfg.max_offset, cfg.search_offset) if self.searching else cfg.max_offset
        tight = cfg.centre_room - cfg.clearance + (cfg.switch_margin if self.centred else 0.0)
        self.centred = 0.0 <= slack < tight and abs(centre) <= limit + 1e-9
        return centre if self.centred else None

    def _patience(self, clear: bool, held: bool, dt: float):
        """Search up to search_offset once held patience_sec with no clear offset."""
        cfg = self.config
        self.waited = 0.0 if clear or not held else self.waited + max(dt, 0.0)
        if self.searching:
            inside = max(abs(self.target), abs(self.offset)) <= cfg.max_offset + 1e-9
            self.searching = not (clear and inside)
        elif self.waited >= cfg.patience_sec and not self.narrow.all():
            self.searching = True

    def slide(self, lateral: float) -> float:
        """Return the sideways speed (m/s, +left) toward the committed offset."""
        error = self.offset - lateral
        if abs(error) <= SLIDE_TOLERANCE:
            return 0.0
        speed = min(self.config.offset_rate, max(SLIDE_MIN, SLIDE_GAIN * abs(error)))
        return math.copysign(speed, error)

    def _commit(self, lateral: float, dt: float):
        """Move the committed offset toward the target, at most max_lead ahead of the robot."""
        step = self.config.offset_rate * max(dt, 0.0)
        new = self.offset + max(-step, min(step, self.target - self.offset))
        reach = max(self.config.max_lead, abs(self.offset - lateral))
        self.offset = max(lateral - reach, min(lateral + reach, new))

    def speed_limit(self, free_distance: float) -> float:
        """Return the highest speed that still stops stop_margin before the obstacle."""
        if math.isinf(free_distance):
            return math.inf
        limits = self.robot.limits
        room = free_distance - self.config.stop_margin
        return stopping_speed(room, limits.max_decel, limits.reaction_time)

    def _accept(self, proposal: float, now: float):
        """Move away at once; back toward the route only after return_delay_sec."""
        target = self.target
        if proposal == target:
            self.return_since = None
            return
        if abs(proposal) < abs(target) and proposal * target >= 0.0:
            if self.return_since is None:
                self.return_since = now
            if now - self.return_since < self.config.return_delay_sec:
                return
        self.return_since = None
        self.target = proposal
