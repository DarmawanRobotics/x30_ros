"""Configuration of path_follower_node (robot geometry and limits come from robot.*)."""

from __future__ import annotations

from dataclasses import dataclass
import math

from x30_navigation.core.config.loader import ProcessingConfig
from x30_navigation.core.config.shift import BacktrackConfig, DetourConfig, PathShiftConfig
from x30_utils.config import nested, param

AVOIDANCE_MODES = ('path_shift', 'reactive', 'off')


@dataclass(frozen=True)
class SpeedConfig:
    """Speed profile along the route; accel and decel are capped by robot.limits."""

    cruise: float = param(0.4, min=0.01, max=5.0, unit='m/s', doc='cruise speed')
    min_speed: float = param(0.1, min=0.0, max=5.0, unit='m/s', doc='floor of the curve limit')
    approach_speed: float = param(
        0.15, min=0.01, max=5.0, unit='m/s', doc='speed at stop points and the route end'
    )
    lateral_accel: float = param(
        0.3, min=0.01, max=10.0, unit='m/s^2', doc='curve limit v = sqrt(a / curvature)'
    )
    accel: float = param(0.3, min=0.01, max=20.0, unit='m/s^2', doc='comfortable acceleration')
    decel: float = param(0.3, min=0.01, max=20.0, unit='m/s^2', doc='comfortable braking')
    smoothing: float = param(
        0.5, min=0.05, max=1.0, doc='weight of a new yaw rate / sideways command (1: off)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.min_speed > self.cruise:
            problems.append('min_speed must not exceed cruise')
        if self.approach_speed > self.cruise:
            problems.append('approach_speed must not exceed cruise')
        return problems


@dataclass(frozen=True)
class SafetyConfig:
    """Timeouts and margins of the supervisor (read-only at runtime)."""

    stop_margin: float = param(
        0.3, min=0.0, max=5.0, unit='m', live=False, doc='gap kept to obstacles when stopping'
    )
    yaw_margin_deg: float = param(
        6.0,
        min=0.0,
        max=90.0,
        unit='deg',
        live=False,
        doc='turning room kept to obstacles, >= the control field.yaw_margin (0.10 rad)',
    )
    scan_timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='older safety scans stop the robot'
    )
    avoidance_timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='older avoidance (reactive) stops it'
    )
    tf_timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='an older pose stops the robot'
    )
    tf_compensate_sec: float = param(
        0.1, min=0.0, max=5.0, unit='s', live=False, doc='older poses are predicted forward'
    )
    clock_tolerance_sec: float = param(
        0.1, min=0.0, max=5.0, unit='s', live=False, doc='a pose from further in the future: clock'
    )
    emergency_release_sec: float = param(
        1.0, min=0.0, max=30.0, unit='s', doc='an emergency stop holds this long after it clears'
    )
    lateral_margin: float = param(
        0.10,
        min=0.0,
        max=1.0,
        unit='m',
        live=False,
        doc='extra width of the guarded path, >= the control field.lateral_margin',
    )
    human_speed: float = param(
        0.5,
        min=0.0,
        max=3.0,
        unit='m/s',
        live=False,
        doc='assumed speed of a person walking toward the robot, >= the control field.human_speed',
    )
    pass_margin: float = param(
        0.15,
        min=0.0,
        max=5.0,
        unit='m',
        live=False,
        doc='gap kept to a point passed within lateral_margin, >= the control field.margin',
    )
    blocked_timeout_sec: float = param(
        10.0, min=0.5, max=600.0, unit='s', doc='standing still this long: BLOCKED'
    )
    stall_distance: float = param(
        0.1, min=0.0, max=2.0, unit='m', doc='moved less than this (measured): standing still'
    )
    stall_yaw_deg: float = param(
        10.0, min=0.0, max=180.0, unit='deg', doc='turned less than this: standing still'
    )
    off_path_slow: float = param(
        0.6, min=0.05, max=10.0, unit='m', doc='further from the route: slow to the floor speed'
    )
    off_path_stop: float = param(
        1.5, min=0.1, max=20.0, unit='m', doc='further from the route: OFF_PATH, stopped'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.tf_compensate_sec > self.tf_timeout_sec:
            problems.append('tf_compensate_sec must not exceed tf_timeout_sec')
        if self.off_path_slow > self.off_path_stop:
            problems.append('off_path_slow must not exceed off_path_stop')
        return problems

    def check_control(
        self,
        lateral_margin: float,
        human_speed: float = 0.0,
        margin: float = 0.0,
        yaw_margin: float = 0.0,
    ) -> list[str]:
        """Return why the guard would plan motions the control field refuses (its values)."""
        problems = []
        if math.radians(self.yaw_margin_deg) < yaw_margin - 1e-9:
            problems.append(
                f'safety.yaw_margin_deg {self.yaw_margin_deg:.1f} is below the control field'
                f' yaw_margin {math.degrees(yaw_margin):.1f} deg: the mux stops turns the guard'
                ' allows'
            )
        if self.pass_margin < margin - 1e-9:
            problems.append(
                f'safety.pass_margin {self.pass_margin:.2f} m is below the control field margin'
                f' {margin:.2f} m: the mux stops passes the guard plans'
            )
        if self.lateral_margin < lateral_margin - 1e-9:
            problems.append(
                f'safety.lateral_margin {self.lateral_margin:.2f} m is below the control field'
                f' lateral_margin {lateral_margin:.2f} m: the mux refuses passes the guard plans'
            )
        if self.human_speed < human_speed - 1e-9:
            problems.append(
                f'safety.human_speed {self.human_speed:.2f} m/s is below the control field'
                f' human_speed {human_speed:.2f} m/s: the mux stops what the guard allows'
            )
        return problems


@dataclass(frozen=True)
class ApproachConfig:
    """Stop points and the final x, y, yaw approach."""

    radius: float = param(
        0.5, min=0.05, max=5.0, unit='m', doc='this close (along the route) the approach starts'
    )
    skip_margin: float = param(
        0.5, min=0.05, max=10.0, unit='m', doc='progress this far past a stop: skipped'
    )
    timeout_sec: float = param(
        20.0, min=1.0, max=600.0, unit='s', doc='give up converging (paused time not counted)'
    )
    kp_xy: float = param(1.2, min=0.0, max=10.0, unit='1/s', doc='gain on the position error')
    kp_yaw: float = param(1.5, min=0.0, max=10.0, unit='1/s', doc='gain on the yaw error')
    max_speed: float = param(0.2, min=0.01, max=2.0, unit='m/s', doc='approach speed cap')
    max_yaw_rate: float = param(0.6, min=0.01, max=5.0, unit='rad/s', doc='yaw rate cap')
    min_speed: float = param(
        0.04, min=0.0, max=1.0, unit='m/s', doc='floor to beat the robot deadband'
    )
    position_tolerance: float = param(0.05, min=0.005, max=1.0, unit='m', doc='arrived within')
    yaw_tolerance_deg: float = param(3.0, min=0.1, max=45.0, unit='deg', doc='aligned within')


@dataclass(frozen=True)
class JoinConfig:
    """Walking onto the route when navigation starts beside it."""

    max_distance: float = param(
        3.0, min=0.0, max=50.0, unit='m', doc='a robot this far walks onto the route (0: off)'
    )
    timeout_sec: float = param(
        60.0, min=1.0, max=600.0, unit='s', doc='give up and follow from where it is'
    )
    on_route: float = param(0.15, min=0.01, max=2.0, unit='m', doc='joined within this')


@dataclass(frozen=True)
class HookConfig:
    """std_srvs/Trigger services called by the follower (read-only)."""

    start: tuple[str, ...] = param(
        (), live=False, doc='called on start; the robot waits for every success'
    )
    stop: tuple[str, ...] = param((), live=False, doc='called on stop (not waited for)')
    inspection: tuple[str, ...] = param((), live=False, doc='called at every stop point')
    timeout_sec: float = param(
        5.0, min=0.1, max=120.0, unit='s', live=False, doc='start and stop hook timeout'
    )
    inspection_timeout_sec: float = param(
        20.0, min=0.1, max=600.0, unit='s', doc='a stop waits this long for inspections'
    )


@dataclass(frozen=True)
class InspectionConfig:
    """Backing up after the inspection at a stop point, before the route goes on."""

    reverse_distance_m: float = param(
        0.0, min=0.0, max=5.0, unit='m', doc='back up this far after the dwell (0: off)'
    )
    reverse_speed: float = param(
        0.1, min=0.01, max=1.0, unit='m/s', doc='backing-up speed (360 deg guard, rear sectors)'
    )


@dataclass(frozen=True)
class RegulatorConfig:
    """Slowing down on sharp turns (steering controllers)."""

    enabled: bool = param(False, doc='scale the speed down when turning hard')
    reference_yaw_rate: float = param(
        0.3, min=0.0, max=5.0, unit='rad/s', doc='turning slower than this needs no slow-down'
    )
    kp: float = param(0.5, min=0.0, max=10.0, unit='s/rad', doc='scale per rad/s above it')
    ki: float = param(0.0, min=0.0, max=10.0, unit='1/rad', doc='scale per integrated rad')
    integral_limit: float = param(1.0, min=0.0, max=10.0, unit='rad', doc='anti-windup clamp')
    min_scale: float = param(0.3, min=0.0, max=1.0, doc='never slower than this fraction')


@dataclass(frozen=True)
class FollowerConfig:
    """path_follower_node parameters."""

    control_rate: float = param(20.0, min=1.0, max=200.0, unit='Hz', live=False, doc='loop rate')
    controller: str = param('holonomic', doc='path controller (x30.controllers)')
    avoidance_mode: str = param(
        'path_shift', choices=AVOIDANCE_MODES, doc='path_shift: slide sideways (holonomic)'
    )
    loop_route: bool = param(False, doc='restart a closed route at its end (patrol)')
    loop_close_distance: float = param(
        1.0, min=0.0, max=20.0, unit='m', doc='end this close to the start: a closed route'
    )
    require_localization: bool = param(True, doc='pause while localization/status is NONE')
    pose_wait_sec: float = param(
        2.0,
        min=0.0,
        max=30.0,
        unit='s',
        doc='navigation/start waits this long for the robot pose before refusing',
    )
    control_node: str = param(
        'control_mux_node',
        live=False,
        doc='control mux whose field the guard must cover (checked once; empty: no check)',
    )
    low_battery: float = param(0.2, min=0.0, max=1.0, doc='below: finish the lap, no new one')
    critical_battery: float = param(0.1, min=0.0, max=1.0, doc='below: pause (reason battery)')
    battery_hysteresis: float = param(0.05, min=0.0, max=0.5, doc='recharge above + this')
    speed: SpeedConfig = nested(SpeedConfig)
    safety: SafetyConfig = nested(SafetyConfig)
    path_shift: PathShiftConfig = nested(PathShiftConfig)
    detour: DetourConfig = nested(DetourConfig)
    backtrack: BacktrackConfig = nested(BacktrackConfig)
    approach: ApproachConfig = nested(ApproachConfig)
    join: JoinConfig = nested(JoinConfig)
    hooks: HookConfig = nested(HookConfig)
    inspection: InspectionConfig = nested(InspectionConfig)
    regulator: RegulatorConfig = nested(RegulatorConfig)
    processing: ProcessingConfig = nested(ProcessingConfig)
    markers: bool = param(True, doc='publish RViz markers')

    def validate(self) -> list[str]:
        """Return cross-field problems (nested sections validate themselves)."""
        from x30_navigation.core.controllers import CONTROLLERS

        problems = []
        if self.controller not in CONTROLLERS:
            problems.append(f'controller: unknown {self.controller!r}, use {CONTROLLERS.names()}')
        if self.critical_battery > self.low_battery:
            problems.append('critical_battery must not exceed low_battery')
        return problems
