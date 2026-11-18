"""The docking control law: centre on the approach line, then walk along it to the docked pose."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.config.docking import ApproachConfig, CenteringConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.docking.geometry import DockErrors
from x30_utils.math import clamp_abs, wrap_angle


@dataclass(frozen=True)
class DockLimits:
    """Speed caps from the robot profile (max_vy is 0 for a diff robot)."""

    max_vx: float = 0.8
    max_vy: float = 0.4
    max_wz: float = 1.0


class DockController:
    """Commands toward the docked pose for one robot (omni or diff)."""

    def __init__(
        self,
        centering: CenteringConfig,
        approach: ApproachConfig,
        can_strafe: bool,
        limits: DockLimits,
        sign: float = 1.0,
    ):
        self.centering, self.approach = centering, approach
        self.can_strafe, self.limits, self.sign = can_strafe, limits, sign

    # ------------------------------------------------------------------ tests on the errors

    def heading_error(self, e: DockErrors) -> float:
        """Return the heading error the controller corrects (diff: toward the line ahead)."""
        if self.can_strafe or e.remaining <= self.approach.blind_distance:
            return e.yaw
        return wrap_angle(e.yaw - self._line_heading(e))

    def centered(self, e: DockErrors) -> bool:
        """Return True when the robot may walk toward the dock."""
        yaw_ok = abs(self.heading_error(e)) <= math.radians(self.centering.yaw_tolerance_deg)
        if not self.can_strafe:
            return yaw_ok  # a diff robot corrects the offset by walking
        return yaw_ok and abs(e.lateral) <= self.centering.lateral_tolerance

    def off_line(self, e: DockErrors) -> bool:
        """Return True when the robot must stop walking and centre again."""
        if e.remaining <= self.approach.blind_distance:
            return False  # too close to see well: go on, the settle check decides
        c = self.centering
        too_far = self.can_strafe and abs(e.lateral) > c.realign_lateral
        return too_far or abs(self.heading_error(e)) > math.radians(c.realign_yaw_deg)

    def arrived(self, e: DockErrors) -> bool:
        """Return True at the goal distance (or past it)."""
        return e.remaining <= self.approach.position_tolerance

    # ------------------------------------------------------------------ commands

    def center(self, e: DockErrors) -> Command:
        """Return a command that centres the robot without walking toward the dock."""
        if not self.can_strafe:
            return Command(0.0, 0.0, self._yaw_rate(self.heading_error(e)))
        return self._omni(e, 0.0)

    def walk(self, e: DockErrors) -> Command:
        """Return a command toward the dock, kept on the line."""
        speed = self._speed(e)
        if self.can_strafe:
            # slower while still beside the line, never stopped by it (off_line stops it)
            lateral = min(abs(e.lateral) / self.centering.realign_lateral, 1.0)
            return self._omni(e, speed * (1.0 - 0.5 * lateral))
        heading = self.heading_error(e)
        along = speed * max(math.cos(heading), 0.0)
        return Command(self.sign * along, 0.0, self._yaw_rate(heading))

    def straight(self, speed: float, away: bool = True) -> Command:
        """Return a straight move along the robot heading, away from the dock by default."""
        direction = -self.sign if away else self.sign
        return Command(direction * min(speed, self.limits.max_vx), 0.0, 0.0)

    # ------------------------------------------------------------------ internals

    def _speed(self, e: DockErrors) -> float:
        a = self.approach
        if e.remaining <= 0.0:
            return 0.0
        return min(a.speed, self.limits.max_vx, max(a.min_speed, a.kp_distance * e.remaining))

    def _omni(self, e: DockErrors, along: float) -> Command:
        a = self.approach
        lateral_cap = min(a.max_lateral_speed, self.limits.max_vy)
        gx = self.sign * along  # toward the dock in the goal frame
        gy = clamp_abs(-a.kp_lateral * e.lateral, lateral_cap)
        c, s = math.cos(e.yaw), math.sin(e.yaw)  # goal frame -> robot frame: rotate by -yaw
        vx, vy = c * gx + s * gy, -s * gx + c * gy
        return Command(vx, vy, self._yaw_rate(e.yaw))

    def _line_heading(self, e: DockErrors) -> float:
        """Heading (goal frame) a diff robot needs to reach the line look_ahead ahead."""
        motion = math.atan2(-e.lateral, self.sign * self.approach.look_ahead)
        return motion if self.sign > 0.0 else wrap_angle(motion + math.pi)

    def _yaw_rate(self, error: float) -> float:
        """Turn the heading error to zero (error: robot minus wanted, as DockErrors.yaw)."""
        a = self.approach
        cap = min(a.max_yaw_rate, self.limits.max_wz)
        tolerance = math.radians(self.centering.yaw_tolerance_deg) * 0.25
        if abs(error) < tolerance:
            return 0.0
        return clamp_abs(-a.kp_yaw * error, cap)
