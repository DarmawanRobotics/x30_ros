"""Configuration of grid_mapper_node, site_manager_node and route_recorder_node."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

SITES_DIR = '~/x30/map'


@dataclass(frozen=True)
class GridConfig:
    """Log-odds grid: resolution, growth and the occupancy model."""

    resolution: float = param(0.05, min=0.01, max=1.0, unit='m', live=False, doc='cell size')
    initial_size_m: float = param(
        20.0, min=1.0, max=1000.0, unit='m', live=False, doc='side of the first grid'
    )
    grow_m: float = param(
        5.0, min=0.1, max=100.0, unit='m', live=False, doc='extra border added when it grows'
    )
    max_size_m: float = param(
        150.0,
        min=1.0,
        max=2000.0,
        unit='m',
        live=False,
        doc='largest side; hits beyond are dropped (memory: (size / resolution)^2 * 4 bytes)',
    )
    log_odds_hit: float = param(0.85, min=0.0, max=10.0, live=False, doc='added per hit')
    log_odds_miss: float = param(-0.4, min=-10.0, max=0.0, live=False, doc='added per crossing')
    log_odds_min: float = param(-2.0, min=-50.0, max=0.0, live=False, doc='clamp, free side')
    log_odds_max: float = param(3.5, min=0.0, max=50.0, live=False, doc='clamp, occupied side')
    occupied_threshold: float = param(
        0.65, min=0.5, max=1.0, live=False, doc='probability above which a cell is occupied'
    )
    free_threshold: float = param(
        0.35, min=0.0, max=0.5, live=False, doc='probability below which a cell is free'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.initial_size_m > self.max_size_m:
            return ['initial_size_m must not exceed max_size_m']
        return []


@dataclass(frozen=True)
class ScanConfig:
    """Which points of a scan are obstacles."""

    voxel: float = param(0.05, min=0.0, max=1.0, unit='m', live=False, doc='downsampling, 0: off')
    min_range: float = param(0.3, min=0.0, max=10.0, unit='m', live=False, doc='from the sensor')
    max_range: float = param(
        15.0, min=0.5, max=200.0, unit='m', live=False, doc='hits and free space beyond: ignored'
    )
    angular_bins: int = param(
        720, min=8, max=7200, live=False, doc='free space is carved to the nearest hit per bin'
    )
    height_min: float = param(
        0.05, min=-1.0, max=3.0, unit='m', live=False, doc='band bottom above the floor'
    )
    height_max: float = param(
        1.5, min=0.0, max=10.0, unit='m', live=False, doc='band top above the floor'
    )
    self_margin: float = param(
        0.05, min=0.0, max=1.0, unit='m', live=False, doc='footprint grown by this is the robot'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.min_range >= self.max_range:
            problems.append('min_range must be below max_range')
        if self.height_min >= self.height_max:
            problems.append('height_min must be below height_max')
        return problems


@dataclass(frozen=True)
class GateConfig:
    """Scans that are not integrated: pose jumps and map->odom corrections in flight."""

    max_jump_m: float = param(
        0.5,
        min=0.01,
        max=100.0,
        unit='m',
        live=False,
        doc='sensor motion between scans beyond this plus max speed * dt: a jump',
    )
    max_jump_deg: float = param(
        30.0, min=1.0, max=180.0, unit='deg', live=False, doc='yaw beyond this plus max_wz * dt'
    )
    correction_window_sec: float = param(
        0.5,
        min=0.0,
        max=10.0,
        unit='s',
        live=False,
        doc='map->odom compared this long before the scan and at the newest time',
    )
    correction_tol_m: float = param(
        0.02, min=0.0, max=1.0, unit='m', live=False, doc='map->odom change counted as correction'
    )
    correction_tol_deg: float = param(
        0.5, min=0.0, max=10.0, unit='deg', live=False, doc='map->odom yaw change, same'
    )
    resume_score: float = param(
        0.6,
        min=0.0,
        max=1.0,
        live=False,
        doc='after a jump, a scan agreeing this well with the map resumes integration',
    )


@dataclass(frozen=True)
class RefineConfig:
    """Optional scan-to-map refinement of the TF pose before integrating."""

    enabled: bool = param(False, live=False, doc='match each scan on the current grid')
    xy_window: float = param(
        0.2, min=0.01, max=2.0, unit='m', live=False, doc='search window around the TF pose'
    )
    yaw_window_deg: float = param(
        3.0, min=0.1, max=30.0, unit='deg', live=False, doc='yaw search window'
    )
    min_score: float = param(
        0.5, min=0.0, max=1.0, live=False, doc='mean likelihood needed to accept a match'
    )
    max_sigma_m: float = param(
        0.045, min=0.001, max=1.0, unit='m', live=False, doc='largest accepted position sigma'
    )
    min_occupied: int = param(
        200, min=10, max=1000000, live=False, doc='occupied cells near the robot needed to match'
    )


@dataclass(frozen=True)
class MapperConfig:
    """grid_mapper_node."""

    sites_dir: str = param(SITES_DIR, live=False, doc='one directory per site')
    cloud_topic: str = param(
        '/cloud_registered_body', live=False, doc='deskewed scan in a frame fixed to the robot'
    )
    rate_hz: float = param(2.0, min=0.1, max=50.0, unit='Hz', live=False, doc='scans integrated')
    publish_hz: float = param(
        1.0, min=0.1, max=10.0, unit='Hz', live=False, doc='mapping/grid while mapping'
    )
    max_cloud_age_sec: float = param(
        1.0, min=0.1, max=30.0, unit='s', live=False, doc='scans without TF this long: dropped'
    )
    save_margin_m: float = param(
        1.0, min=0.0, max=20.0, unit='m', live=False, doc='unknown border kept when saving'
    )
    grid: GridConfig = nested(GridConfig)
    scan: ScanConfig = nested(ScanConfig)
    gate: GateConfig = nested(GateConfig)
    refine: RefineConfig = nested(RefineConfig)


@dataclass(frozen=True)
class SiteConfig:
    """site_manager_node."""

    sites_dir: str = param(SITES_DIR, live=False, doc='one directory per site')
    site: str = param('', live=False, doc='site loaded at start; empty: the one of the last run')
    state_file: str = param('~/.x30/site', live=False, doc='remembers the active site')
    create_site: bool = param(
        False, live=False, doc='create `site` at start when missing: no map, routes and tags'
    )
    start_mapping: bool = param(
        False, live=False, doc='map `site` at start when it is a new site (mapping mode)'
    )
    mapping: bool = param(
        True,
        live=False,
        doc='mapping/start, stop and save with grid_mapper_node; false: a robot without a map',
    )
    service_timeout_sec: float = param(
        5.0, min=0.5, max=60.0, unit='s', live=False, doc='waiting for localization and mapper'
    )
    refresh_sec: float = param(
        2.0, min=0.5, max=60.0, unit='s', live=False, doc='site/status refresh (tags, routes)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.create_site and self.start_mapping:
            return ['create_site (odometry only) and start_mapping (mapping mode): one of them']
        if self.start_mapping and not self.mapping:
            return ['start_mapping needs mapping (a robot without a map maps nothing)']
        return []


@dataclass(frozen=True)
class RecorderConfig:
    """route_recorder_node."""

    sites_dir: str = param(SITES_DIR, live=False, doc='routes go to <sites_dir>/<site>/routes')
    output_path: str = param(
        '', live=False, doc='directory for routes while no site is active; empty: refuse'
    )
    min_spacing_m: float = param(
        0.2, min=0.01, max=10.0, unit='m', doc='a new waypoint after this much motion'
    )
    record_rate_hz: float = param(
        10.0, min=0.5, max=100.0, unit='Hz', live=False, doc='pose sampling while recording'
    )
    flush_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', doc='the route file is rewritten at most this often'
    )
    max_tf_age_sec: float = param(
        0.5, min=0.01, max=10.0, unit='s', doc='older map->base: no sample'
    )
