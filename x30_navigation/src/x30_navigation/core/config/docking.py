"""Configuration of docking_node (robot geometry, limits and frames come from robot.*)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

DIRECTIONS = ('forward', 'backward')
TAG_AXES = ('x', 'y', 'z')


@dataclass(frozen=True)
class TagConfig:
    """The fiducial on the dock, as a TF frame published by the detector (AprilTag or ArUco)."""

    frame: str = param('charging_dock', live=False, doc='TF frame of the dock tag')
    normal_axis: str = param(
        'z', choices=TAG_AXES, doc='tag axis along the plate normal (its sign is found)'
    )
    timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', doc='an older detection is not in view'
    )
    max_range: float = param(4.0, min=0.2, max=20.0, unit='m', doc='further detections ignored')
    smoothing: float = param(
        0.3, min=0.01, max=1.0, doc='weight of a new detection in the dock estimate (1: none)'
    )
    max_jump: float = param(
        0.25, min=0.01, max=5.0, unit='m', doc='a detection moving the dock further: outlier'
    )
    max_outliers: int = param(
        5, min=1, max=100, doc='this many outliers in a row: start the estimate over'
    )


@dataclass(frozen=True)
class GoalConfig:
    """Where the robot stands when it is docked, relative to the tag."""

    distance: float = param(
        0.75, min=0.05, max=5.0, unit='m', doc='base_link to the tag along its normal'
    )
    lateral_offset: float = param(
        0.0, min=-2.0, max=2.0, unit='m', doc='goal beside the tag centre (+left facing it)'
    )
    yaw_offset_deg: float = param(
        0.0, min=-45.0, max=45.0, unit='deg', doc='goal heading beside the tag normal'
    )
    direction: str = param(
        'forward', choices=DIRECTIONS, doc='forward: faces the tag; backward: backs in'
    )


@dataclass(frozen=True)
class CenteringConfig:
    """Centring the tag on the approach line before (and while) walking toward the dock."""

    lateral_tolerance: float = param(
        0.03, min=0.002, max=0.5, unit='m', doc='centred within this beside the line'
    )
    yaw_tolerance_deg: float = param(
        2.0, min=0.1, max=30.0, unit='deg', doc='aligned within this to the line'
    )
    realign_lateral: float = param(
        0.10, min=0.005, max=1.0, unit='m', doc='further beside the line: stop and centre'
    )
    realign_yaw_deg: float = param(
        8.0, min=0.5, max=60.0, unit='deg', doc='further off the heading: stop and centre'
    )
    hold_sec: float = param(
        0.3, min=0.0, max=10.0, unit='s', doc='centred this long before walking on'
    )
    timeout_sec: float = param(
        30.0, min=1.0, max=600.0, unit='s', doc='give up centring (an attempt fails)'
    )


@dataclass(frozen=True)
class ApproachConfig:
    """Walking toward the dock, centred on the line."""

    speed: float = param(0.12, min=0.01, max=1.0, unit='m/s', doc='speed toward the dock')
    min_speed: float = param(
        0.03, min=0.0, max=0.5, unit='m/s', doc='floor to beat the robot deadband'
    )
    max_lateral_speed: float = param(
        0.08, min=0.0, max=1.0, unit='m/s', doc='sideways correction cap (omni)'
    )
    max_yaw_rate: float = param(0.4, min=0.01, max=3.0, unit='rad/s', doc='yaw rate cap')
    kp_distance: float = param(0.8, min=0.0, max=10.0, unit='1/s', doc='gain on the distance')
    kp_lateral: float = param(1.5, min=0.0, max=10.0, unit='1/s', doc='gain beside the line')
    kp_yaw: float = param(1.5, min=0.0, max=10.0, unit='1/s', doc='gain on the heading')
    look_ahead: float = param(
        0.3, min=0.05, max=3.0, unit='m', doc='steering look-ahead on the line (diff)'
    )
    blind_distance: float = param(
        0.25,
        min=0.0,
        max=3.0,
        unit='m',
        doc='this close the dock estimate freezes and the tag may leave the view',
    )
    position_tolerance: float = param(
        0.02, min=0.002, max=0.5, unit='m', doc='arrived within this of the goal distance'
    )
    timeout_sec: float = param(
        60.0, min=1.0, max=600.0, unit='s', doc='give up the approach (an attempt fails)'
    )


@dataclass(frozen=True)
class SettleConfig:
    """Standing still on the dock, then checking the pose against the tag."""

    time_sec: float = param(1.0, min=0.0, max=30.0, unit='s', doc='stand still this long')
    distance_tolerance: float = param(
        0.04, min=0.002, max=0.5, unit='m', doc='docked within this along the normal'
    )
    lateral_tolerance: float = param(
        0.03, min=0.002, max=0.5, unit='m', doc='docked within this beside the line'
    )
    yaw_tolerance_deg: float = param(
        4.0, min=0.1, max=45.0, unit='deg', doc='docked within this heading'
    )


@dataclass(frozen=True)
class RetryConfig:
    """Another attempt after a failed one: back off along the line, search, centre again."""

    max_retries: int = param(2, min=0, max=20, doc='attempts after the first (0: none)')
    backoff_distance: float = param(
        0.4, min=0.0, max=3.0, unit='m', doc='move this far away from the dock first'
    )
    speed: float = param(0.1, min=0.01, max=1.0, unit='m/s', doc='backing-off speed')


@dataclass(frozen=True)
class SupervisionConfig:
    """Timeouts while moving: tag, pose and a robot that does not move."""

    search_timeout_sec: float = param(
        5.0, min=0.1, max=600.0, unit='s', doc='no dock tag this long after a start: failed'
    )
    lost_timeout_sec: float = param(
        3.0, min=0.1, max=60.0, unit='s', doc='tag gone this long beyond blind_distance'
    )
    pose_timeout_sec: float = param(
        0.5, min=0.05, max=5.0, unit='s', live=False, doc='an older odom pose stops the robot'
    )
    stall_timeout_sec: float = param(
        8.0, min=0.5, max=120.0, unit='s', doc='commanded but not moving this long: held'
    )
    stall_distance: float = param(
        0.02, min=0.0, max=1.0, unit='m', doc='moved less than this: not moving'
    )
    stall_yaw_deg: float = param(
        2.0, min=0.0, max=90.0, unit='deg', doc='turned less than this: not moving'
    )


@dataclass(frozen=True)
class DockHookConfig:
    """std_srvs/Trigger services of the robot driver, called in order (read-only)."""

    sit: tuple[str, ...] = param((), live=False, doc='called once docked (e.g. /robot/sit)')
    charge: tuple[str, ...] = param(
        (), live=False, doc='called after sitting (e.g. /robot/charge/start)'
    )
    stop_charge: tuple[str, ...] = param(
        (), live=False, doc='called first when undocking (e.g. /robot/charge/stop)'
    )
    stand: tuple[str, ...] = param(
        (), live=False, doc='called before backing off the dock (e.g. /robot/stand)'
    )
    navigation_stop: str = param(
        'navigation/stop', live=False, doc='stops the path follower when docking starts ("": no)'
    )
    timeout_sec: float = param(
        15.0, min=0.1, max=600.0, unit='s', live=False, doc='each hook call times out'
    )
    sit_settle_sec: float = param(
        1.5, min=0.0, max=60.0, unit='s', doc='wait after sitting before the charge hooks'
    )
    stand_settle_sec: float = param(
        1.5, min=0.0, max=60.0, unit='s', doc='wait after standing before backing off'
    )


@dataclass(frozen=True)
class ChargeConfig:
    """Confirming that the robot charges (sensor_msgs/BatteryState power_supply_status)."""

    confirm_timeout_sec: float = param(
        30.0, min=0.0, max=600.0, unit='s', doc='wait for the battery to charge (0: do not)'
    )
    battery_timeout_sec: float = param(
        5.0, min=0.1, max=120.0, unit='s', doc='an older battery state counts as unknown'
    )


@dataclass(frozen=True)
class UndockConfig:
    """Leaving the dock after the stand hooks."""

    distance: float = param(
        0.5, min=0.0, max=5.0, unit='m', doc='move this far away from the dock (0: stay)'
    )
    speed: float = param(0.1, min=0.01, max=1.0, unit='m/s', doc='undocking speed')
    timeout_sec: float = param(30.0, min=1.0, max=600.0, unit='s', doc='give up undocking')


@dataclass(frozen=True)
class DockingConfig:
    """docking_node parameters."""

    control_rate: float = param(20.0, min=1.0, max=200.0, unit='Hz', live=False, doc='loop rate')
    control_node: str = param(
        'control_mux_node', live=False, doc='mux whose field is checked against the goal ("": no)'
    )
    tag: TagConfig = nested(TagConfig)
    goal: GoalConfig = nested(GoalConfig)
    centering: CenteringConfig = nested(CenteringConfig)
    approach: ApproachConfig = nested(ApproachConfig)
    settle: SettleConfig = nested(SettleConfig)
    retry: RetryConfig = nested(RetryConfig)
    supervision: SupervisionConfig = nested(SupervisionConfig)
    hooks: DockHookConfig = nested(DockHookConfig)
    charge: ChargeConfig = nested(ChargeConfig)
    undock: UndockConfig = nested(UndockConfig)

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        c, a = self.centering, self.approach
        if c.lateral_tolerance >= c.realign_lateral:
            problems.append('centering.lateral_tolerance must be below realign_lateral')
        if c.yaw_tolerance_deg >= c.realign_yaw_deg:
            problems.append('centering.yaw_tolerance_deg must be below realign_yaw_deg')
        if a.min_speed > a.speed:
            problems.append('approach.min_speed must not exceed approach.speed')
        if a.position_tolerance > self.settle.distance_tolerance:
            problems.append('approach.position_tolerance must not exceed settle tolerance')
        if a.blind_distance >= self.tag.max_range:
            problems.append('approach.blind_distance must be below tag.max_range')
        return problems
