"""Configuration of planner_node."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param


@dataclass(frozen=True)
class CostmapConfig:
    """Planning grid and inflation; the radii come from the robot footprint."""

    resolution: float = param(
        0.1, min=0.01, max=1.0, unit='m', doc='planning cell size, the map is max-pooled to it'
    )
    margin: float = param(
        0.1,
        min=0.0,
        max=1.0,
        unit='m',
        doc='clearance added to the footprint (keep >= half a cell)',
    )
    inflation_radius: float = param(
        0.8, min=0.0, max=5.0, unit='m', doc='paths keep away from walls up to here'
    )
    cost_scaling: float = param(
        3.0, min=0.0, max=50.0, unit='1/m', doc='how fast the wall cost falls off'
    )
    unknown_is_obstacle: bool = param(True, doc='never plan through unmapped space')
    occupied_threshold: int = param(50, min=1, max=100, doc='map values from here are obstacles')


@dataclass(frozen=True)
class SearchConfig:
    """A*, goal snapping, start escape and the path handed to the follower."""

    goal_tolerance: float = param(
        0.5, min=0.0, max=5.0, unit='m', doc='a goal in an obstacle moves to free space this close'
    )
    start_escape: float = param(
        1.0, min=0.0, max=5.0, unit='m', doc='a robot inside the inflation walks out up to here'
    )
    heuristic_weight: float = param(
        1.0, min=1.0, max=5.0, doc='> 1: faster search, slightly longer paths'
    )
    timeout_sec: float = param(5.0, min=0.1, max=60.0, unit='s', doc='search time limit')
    spacing: float = param(0.1, min=0.02, max=1.0, unit='m', doc='distance between waypoints')
    smooth_weight_data: float = param(0.1, min=0.0, max=1.0, doc='pull toward the search path')
    smooth_weight_smooth: float = param(0.3, min=0.0, max=0.49, doc='pull toward the neighbours')
    footprint_retries: int = param(
        5, min=0, max=20, doc='searches around poses where the oriented footprint collides'
    )


@dataclass(frozen=True)
class MonitorConfig:
    """How the planner follows the follower while it walks a plan."""

    arrival_tolerance: float = param(
        0.35, min=0.01, max=5.0, unit='m', doc='GOAL_REACHED counts this close to the plan goal'
    )
    stop_on_arrival: bool = param(True, doc='stop the follower at the goal')
    replan_after_sec: float = param(
        3.0, min=0.1, max=300.0, unit='s', doc='follower BLOCKED this long: plan again'
    )
    max_replans: int = param(5, min=0, max=100, doc='per goal, then failed')
    cancel_after_sec: float = param(
        1.0, min=0.1, max=60.0, unit='s', doc='follower INACTIVE this long: cancelled'
    )


@dataclass(frozen=True)
class PlannerConfig:
    """planner_node parameters."""

    costmap: CostmapConfig = nested(CostmapConfig)
    search: SearchConfig = nested(SearchConfig)
    monitor: MonitorConfig = nested(MonitorConfig)
    use_live_obstacles: bool = param(True, doc='replans see the obstacles of the safety scan')
    scan_timeout_sec: float = param(
        1.0, min=0.05, max=10.0, unit='s', live=False, doc='older safety scans are ignored'
    )
    tf_timeout_sec: float = param(
        0.5, min=0.05, max=10.0, unit='s', live=False, doc='an older robot pose is unknown'
    )
    pose_wait_sec: float = param(
        2.0,
        min=0.0,
        max=30.0,
        unit='s',
        doc='go_to / plan wait this long for a fresh robot pose before refusing',
    )
    publish_costmap: bool = param(False, doc='planner/costmap for RViz (debug)')
    status_hz: float = param(2.0, min=0.1, max=50.0, unit='Hz', doc='planner/status while active')
    service_timeout_sec: float = param(
        5.0, min=0.1, max=60.0, unit='s', doc='calls to the follower'
    )
