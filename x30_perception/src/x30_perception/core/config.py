"""Configuration of lidar_sector_node (parameters input.*, front.*, safety.*, filter.* ...)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

INPUT_TYPES = ('livox', 'pointcloud2')
LEVEL_SOURCES = ('tf', 'none')


@dataclass(frozen=True)
class InputConfig:
    """Where the points come from."""

    type: str = param(
        'livox',
        choices=INPUT_TYPES,
        live=False,
        doc='livox: raw Livox CustomMsg bytes; pointcloud2: any PointCloud2',
    )
    livox_topic: str = param('livox/lidar', live=False, doc='Livox CustomMsg topic')
    pointcloud_topic: str = param(
        '/cloud_registered_body', live=False, doc='PointCloud2 topic (frame fixed to the robot)'
    )
    frame: str = param('livox_frame', live=False, doc='sensor frame when the header has none')
    timeout_sec: float = param(
        0.5,
        min=0.05,
        max=10.0,
        unit='s',
        live=False,
        doc='no usable cloud for this long: every sector is published BLOCKED',
    )


@dataclass(frozen=True)
class SectorConfig:
    """Front scan for avoidance (perception/sector_scan)."""

    num_sectors: int = param(32, min=1, max=1440, live=False, doc='sectors over the fov')
    fov_deg: float = param(180.0, min=1.0, max=360.0, unit='deg', live=False, doc='centred ahead')
    range_min: float = param(0.1, min=0.0, max=10.0, unit='m', live=False, doc='closer: ignored')
    range_max: float = param(8.0, min=0.1, max=100.0, unit='m', live=False, doc='free beyond')
    k: int = param(
        2, min=1, max=100, live=False, doc='a sector needs k points; its range is the k-th nearest'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return [] if self.range_min < self.range_max else ['range_min must be below range_max']


@dataclass(frozen=True)
class SafetyConfig:
    """360 deg scan for the protective field (perception/safety_scan)."""

    enabled: bool = param(True, live=False, doc='publish perception/safety_scan')
    num_sectors: int = param(72, min=4, max=1440, live=False, doc='sectors over 360 deg')
    range_min: float = param(0.1, min=0.0, max=10.0, unit='m', live=False, doc='closer: ignored')
    range_max: float = param(4.0, min=0.1, max=100.0, unit='m', live=False, doc='free beyond')
    k: int = param(
        2, min=1, max=100, live=False, doc='a sector needs k points; its range is the k-th nearest'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return [] if self.range_min < self.range_max else ['range_min must be below range_max']


@dataclass(frozen=True)
class FilterConfig:
    """Which points are obstacles: self filter, leveling, height band, ground, Livox noise."""

    self_margin: float = param(
        0.05, min=0.0, max=1.0, unit='m', live=False, doc='footprint grown by this is the robot'
    )
    level_source: str = param(
        'tf',
        choices=LEVEL_SOURCES,
        live=False,
        doc='tf: roll/pitch of base in odom (unknown -> BLOCKED); none: base assumed level',
    )
    tilt_max_age: float = param(
        0.2, min=0.01, max=5.0, unit='s', live=False, doc='oldest odom->base usable for leveling'
    )
    height_min: float = param(
        0.10, min=-1.0, max=3.0, unit='m', live=False, doc='band bottom above the floor'
    )
    height_max: float = param(
        1.60, min=0.0, max=5.0, unit='m', live=False, doc='band top above the floor'
    )
    ground_segmentation: bool = param(
        False, live=False, doc='estimate the floor height per cell (ramps, uneven floors)'
    )
    ground_cell: float = param(
        0.3, min=0.05, max=5.0, unit='m', live=False, doc='ground segmentation cell size'
    )
    ground_max_step: float = param(
        0.05, min=0.0, max=0.5, unit='m', live=False, doc='largest local floor offset accepted'
    )
    drop_adhesion_noise: bool = param(True, live=False, doc='Livox tag bits 0-1 (glue points)')
    drop_particle_noise: bool = param(True, live=False, doc='Livox tag bits 2-3 (rain, dust)')
    drop_other_noise: bool = param(False, live=False, doc='Livox tag bits 4-5 (other)')
    noise_level: int = param(
        2, min=1, max=2, live=False, doc='drop at this tag confidence: 1 medium, 2 low'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return [] if self.height_min < self.height_max else ['height_min must be below height_max']


@dataclass(frozen=True)
class MemoryConfig:
    """Short obstacle memory in the odom frame, merged into the safety scan."""

    enabled: bool = param(True, live=False, doc='remember obstacles that left the view')
    memory_sec: float = param(
        10.0, min=0.0, max=60.0, unit='s', live=False, doc='remembered points decay after this'
    )
    radius: float = param(
        5.0,
        min=0.1,
        max=20.0,
        unit='m',
        live=False,
        doc='points this close are remembered (merged only while below the lowest beam)',
    )
    resolution: float = param(
        0.05, min=0.01, max=1.0, unit='m', live=False, doc='memory cell size'
    )
    max_cells: int = param(20000, min=100, max=1000000, live=False, doc='memory size bound')
    lower_fov_deg: float = param(
        -7.0,
        min=-90.0,
        max=0.0,
        unit='deg',
        live=False,
        doc='lowest beam elevation of the sensor (Mid-360: -7), for the near-field blind zone',
    )
    max_jump: float = param(
        0.3, min=0.0, max=10.0, unit='m', live=False, doc='odom jump beyond the motion: forget'
    )


@dataclass(frozen=True)
class BlindConfig:
    """When the LiDAR counts as blind (every sector BLOCKED)."""

    min_points: int = param(
        100, min=0, max=1000000, live=False, doc='fewer valid points in a cloud: blind'
    )
    regions: int = param(
        4, min=1, max=36, live=False, doc='azimuth regions around the sensor z axis'
    )
    min_region_points: int = param(
        0, min=0, max=100000, live=False, doc='fewer points in any region: blind (0: off)'
    )


@dataclass(frozen=True)
class DepthConfig:
    """A depth camera's cloud merged into both scans: the low obstacles near the robot."""

    enabled: bool = param(False, live=False, doc='merge a depth camera cloud into both scans')
    topic: str = param(
        '/camera/camera/depth/color/points', live=False, doc='PointCloud2 of the depth camera'
    )
    frame: str = param(
        'camera_depth_optical_frame', live=False, doc='camera frame when the header has none'
    )
    timeout_sec: float = param(
        0.3,
        min=0.05,
        max=5.0,
        unit='s',
        live=False,
        doc='an older cloud is not merged: the scans come from the LiDAR alone',
    )
    required: bool = param(
        False, live=False, doc='no usable depth cloud: every sector BLOCKED (false: LiDAR alone)'
    )
    range_max: float = param(
        3.0,
        min=0.2,
        max=20.0,
        unit='m',
        live=False,
        doc='farther from the camera: dropped (the stereo depth error grows with range squared)',
    )
    min_points: int = param(
        200,
        min=0,
        max=10000000,
        live=False,
        doc='fewer points in a cloud (robot body excluded): the camera is blind, not merged',
    )
    cell_size: float = param(
        0.05, min=0.01, max=1.0, unit='m', live=False, doc='obstacle cell edge length'
    )
    cell_points: int = param(
        4,
        min=1,
        max=1000,
        live=False,
        doc='an obstacle needs this many points in one cell (a stereo speckle is one point)',
    )
    confirm_radius: float = param(
        0.15,
        min=0.0,
        max=2.0,
        unit='m',
        live=False,
        doc='an obstacle cell needs one this close in the previous cloud too (0: off)',
    )
    memory_sec: float = param(
        6.0,
        min=0.0,
        max=60.0,
        unit='s',
        live=False,
        doc='camera obstacles stay this long while out of its view, below the LiDAR (0: off)',
    )
    floor_fit: bool = param(
        True, live=False, doc='heights above the floor the camera sees (mount, body pitch)'
    )
    floor_max_tilt_deg: float = param(
        5.0,
        min=0.0,
        max=30.0,
        unit='deg',
        live=False,
        doc='a fitted floor tilted more than this from the nominal one is not used',
    )
    floor_max_offset: float = param(
        0.05,
        min=0.0,
        max=1.0,
        unit='m',
        live=False,
        doc='a fitted floor further than this from the nominal one under the robot is not used',
    )


@dataclass(frozen=True)
class PerceptionConfig:
    """Everything lidar_sector_node reads besides the robot profile."""

    input: InputConfig = nested(InputConfig)
    front: SectorConfig = nested(SectorConfig)
    safety: SafetyConfig = nested(SafetyConfig)
    filter: FilterConfig = nested(FilterConfig)
    memory: MemoryConfig = nested(MemoryConfig)
    blind: BlindConfig = nested(BlindConfig)
    depth: DepthConfig = nested(DepthConfig)
    publish_markers: bool = param(True, live=False, doc='perception/obstacles for RViz')

    def validate(self) -> list[str]:
        """Return cross-field problems of every section (build() only calls the top level)."""
        problems = []
        for name in ('front', 'safety', 'filter'):
            problems += [f'{name}.{p}' for p in getattr(self, name).validate()]
        return problems
