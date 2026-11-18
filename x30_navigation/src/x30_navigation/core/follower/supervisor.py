"""Conditions that stop or slow the follower: stale poses and scans, stalls, off path, battery."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.follower.state import Inputs
from x30_utils.math import Pose2D, angle_diff


class Problem(Enum):
    """Why the robot cannot move now; the name starts the status message."""

    TF_STARTING = 'no robot pose yet'
    TF_STALE = 'the robot pose is too old'
    TF_CLOCK = 'the robot pose is from the future (clock mismatch?)'
    SCAN_STALE = 'no fresh perception/safety_scan'
    SCAN_BLIND = 'the safety scan is blind (blocked sectors)'
    AVOIDANCE_STALE = 'no fresh navigation/avoidance'
    EMERGENCY = 'obstacle in the emergency cone'


@dataclass(frozen=True)
class PoseCheck:
    """The pose to use (predicted when slightly old) or the problem."""

    pose: Pose2D | None
    problem: Problem | None
    predicted: bool = False


class StallWatch:
    """How long the robot stood still (measured pose), while it was meant to move."""

    def __init__(self, distance: float, yaw_deg: float):
        self.distance = distance
        self.yaw = math.radians(yaw_deg)
        self.anchor: Pose2D | None = None
        self.seconds = 0.0

    def reset(self):
        """Forget the standing time."""
        self.anchor, self.seconds = None, 0.0

    def update(self, pose: Pose2D | None, dt: float, counts: bool) -> float:
        """Return the standing time (s); moving or turning beyond the thresholds resets it."""
        if not counts or pose is None:
            self.reset()
            return 0.0
        anchor = self.anchor
        moved = anchor is None or (
            math.hypot(pose.x - anchor.x, pose.y - anchor.y) > self.distance
            or abs(angle_diff(pose.yaw, anchor.yaw)) > self.yaw
        )
        if moved:
            self.anchor, self.seconds = pose, 0.0
        else:
            self.seconds += max(dt, 0.0)
        return self.seconds


class Supervisor:
    """Stateful checks of one navigation run."""

    def __init__(self, config: FollowerConfig):
        self.config = config
        self.had_pose = False
        cfg = config.safety
        self.stall = StallWatch(cfg.stall_distance, cfg.stall_yaw_deg)
        self.emergency_clear_sec = None
        self.battery_low = False
        self.battery_critical = False

    def reset(self):
        """Forget the stall time (new start, new route)."""
        self.stall.reset()

    def configure(self, config: FollowerConfig):
        """Apply new parameters (the standing time is kept)."""
        self.config = config
        self.stall.distance = config.safety.stall_distance
        self.stall.yaw = math.radians(config.safety.stall_yaw_deg)

    def pose(self, inputs: Inputs) -> PoseCheck:
        """Check the TF pose; between tf_compensate_sec and tf_timeout_sec predict it forward."""
        cfg = self.config.safety
        if inputs.pose is None:
            return PoseCheck(None, Problem.TF_STALE if self.had_pose else Problem.TF_STARTING)
        age = inputs.pose.age
        if age < -cfg.clock_tolerance_sec:
            return PoseCheck(None, Problem.TF_CLOCK)
        if age > cfg.tf_timeout_sec or not inputs.pose.pose.is_finite():
            return PoseCheck(None, Problem.TF_STALE if self.had_pose else Problem.TF_STARTING)
        self.had_pose = True
        pose, twist = inputs.pose.pose, inputs.twist
        if age < cfg.tf_compensate_sec or twist is None:
            return PoseCheck(pose, None)
        moved = Pose2D(twist.vx * age, twist.vy * age, twist.wz * age)
        return PoseCheck(pose @ moved, None, True)

    def scan(self, inputs: Inputs) -> Problem | None:
        """Return a problem with the safety scan, or None."""
        scan = inputs.scan
        if scan is None or not 0.0 <= scan.age <= self.config.safety.scan_timeout_sec:
            return Problem.SCAN_STALE
        return Problem.SCAN_BLIND if scan.blocked else None

    def avoidance(self, inputs: Inputs, required: bool) -> Problem | None:
        """Return an avoidance problem: emergency always, staleness when required."""
        fresh = inputs.avoidance is not None
        fresh = fresh and inputs.avoidance_age <= self.config.safety.avoidance_timeout_sec
        if fresh and inputs.avoidance.emergency:
            self.emergency_clear_sec = 0.0
        elif self.emergency_clear_sec is not None:
            self.emergency_clear_sec += inputs.dt
            if self.emergency_clear_sec >= self.config.safety.emergency_release_sec:
                self.emergency_clear_sec = None
        if self.emergency_clear_sec is not None:
            return Problem.EMERGENCY
        if required and not fresh:
            return Problem.AVOIDANCE_STALE
        return None

    def blocked(self, pose: Pose2D | None, counts: bool, dt: float) -> bool:
        """Return True once the robot stood still for blocked_timeout_sec while meant to move."""
        seconds = self.stall.update(pose, dt, counts)
        return seconds >= self.config.safety.blocked_timeout_sec

    def force_blocked(self, pose: Pose2D):
        """Report BLOCKED from now on until the robot moves (a known blocked way)."""
        self.stall.update(pose, 0.0, True)
        self.stall.seconds = self.config.safety.blocked_timeout_sec

    def off_path(self, cross_track: float) -> tuple[bool, bool]:
        """Return (slow, stop) for the distance from the route beyond the path offset."""
        cfg = self.config.safety
        return cross_track > cfg.off_path_slow, cross_track > cfg.off_path_stop

    def battery(self, fraction: float) -> tuple[bool, bool]:
        """Update the battery levels from a 0..1 reading (others ignored); (low, critical)."""
        if not 0.0 <= fraction <= 1.0:
            return self.battery_low, self.battery_critical
        cfg = self.config
        up = cfg.battery_hysteresis
        self.battery_low = fraction < cfg.low_battery + (up if self.battery_low else 0.0)
        critical = cfg.critical_battery + (up if self.battery_critical else 0.0)
        self.battery_critical = fraction < critical
        return self.battery_low, self.battery_critical
