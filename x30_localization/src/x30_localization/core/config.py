"""Configuration of the localization node (parameters, with units and ranges)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

GATE_CONFIDENCES = (0.95, 0.99, 0.999)


@dataclass(frozen=True)
class EstimatorConfig:
    """SE(2) EKF over map->odom: drift per motion, gate, uncertainty limits."""

    sigma_xy_per_sqrt_m: float = param(
        0.03, min=0.0, max=1.0, unit='m/sqrt(m)', doc='position drift: 1-sigma after 1 m of travel'
    )
    sigma_yaw_deg_per_sqrt_m: float = param(
        0.1, min=0.0, max=10.0, unit='deg/sqrt(m)', doc='heading drift per metre of travel'
    )
    sigma_yaw_deg_per_sqrt_rad: float = param(
        0.5, min=0.0, max=10.0, unit='deg/sqrt(rad)', doc='heading drift per radian turned'
    )
    gate_confidence: float = param(
        0.99, choices=GATE_CONFIDENCES, doc='chi-square gate (3 dof) of automatic corrections'
    )
    # no map to correct the drift but tags: the state stays LOCALIZED, sigma reports the drift
    max_sigma_xy: float = param(
        100.0, min=0.01, max=100.0, unit='m', doc='above: state UNCERTAIN (position)'
    )
    max_sigma_yaw_deg: float = param(
        180.0, min=0.1, max=180.0, unit='deg', doc='above: state UNCERTAIN (heading)'
    )
    # the origin is the start spot: the robot is put on it by the dock or by hand, as well as that
    origin_sigma_xy: float = param(
        0.05, min=0.0, max=1.0, unit='m', doc='position of the start spot (assume_origin)'
    )
    origin_sigma_yaw_deg: float = param(
        2.0, min=0.0, max=45.0, unit='deg', doc='heading on the start spot (assume_origin)'
    )
    min_sigma_xy: float = param(
        0.01, min=0.0, max=1.0, unit='m', doc='floor after a correction (no overconfidence)'
    )
    min_sigma_yaw_deg: float = param(
        0.25, min=0.0, max=10.0, unit='deg', doc='heading floor after a correction'
    )
    motion_step_m: float = param(
        0.05, min=0.001, max=1.0, unit='m', doc='odometry motion counted in steps (jitter-free)'
    )
    motion_step_deg: float = param(
        2.0, min=0.01, max=45.0, unit='deg', doc='odometry rotation counted in steps'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if self.min_sigma_xy >= self.max_sigma_xy:
            problems.append('min_sigma_xy must be below max_sigma_xy')
        if self.min_sigma_yaw_deg >= self.max_sigma_yaw_deg:
            problems.append('min_sigma_yaw_deg must be below max_sigma_yaw_deg')
        return problems


@dataclass(frozen=True)
class TagConfig:
    """Fiducial measurements: detection frames, trust model, recording."""

    camera_frame: str = param(
        'camera_color_optical_frame', live=False, doc='frame the detector puts the tags under'
    )
    frames: tuple[str, ...] = param(
        ('base_map', 'charging_dock'), doc='tag TF frames to record (the detector tag frames)'
    )
    max_age_sec: float = param(0.5, min=0.01, max=10.0, unit='s', doc='older detections ignored')
    max_distance_m: float = param(4.0, min=0.1, max=50.0, unit='m', doc='farther tags ignored')
    max_view_angle_deg: float = param(
        60.0, min=1.0, max=89.0, unit='deg', doc='tags seen more obliquely are ignored'
    )
    sigma_xy_at_1m: float = param(
        0.02, min=0.001, max=1.0, unit='m', doc='position sigma at 1 m, grows with distance^2'
    )
    sigma_yaw_deg_at_1m: float = param(
        1.0, min=0.01, max=30.0, unit='deg', doc='heading sigma at 1 m, grows with distance'
    )
    update_hz: float = param(10.0, min=0.5, max=50.0, unit='Hz', live=False, doc='tag checks')
    min_interval_sec: float = param(
        0.5, min=0.0, max=60.0, unit='s', doc='per tag, between two fused detections'
    )
    auto_localize: bool = param(True, doc='while not localized: localize from any visible tag')
    auto_correct: bool = param(True, doc='while localized: fuse visible tags through the gate')
    record_duration_sec: float = param(2.0, min=0.2, max=60.0, unit='s', doc='record_tags')
    record_min_samples: int = param(5, min=1, max=1000, doc='detections needed per tag')
    record_max_std_m: float = param(
        0.05, min=0.001, max=1.0, unit='m', doc='noisier recordings are refused'
    )
    record_max_std_deg: float = param(
        3.0, min=0.01, max=45.0, unit='deg', doc='noisier recordings are refused (heading)'
    )
    marker_size: float = param(0.15, min=0.01, max=2.0, unit='m', doc='tag plate in RViz')


@dataclass(frozen=True)
class LevelingConfig:
    """Gravity alignment of odom from the IMU while the robot stands still."""

    enabled: bool = param(True, live=False, doc='level odom with the IMU (off: odom = base)')
    imu_topic: str = param('/imu/data', live=False, doc='IMU in the LIO body frame axes')
    window_sec: float = param(1.0, min=0.1, max=10.0, unit='s', doc='still window')
    max_rate_deg_s: float = param(
        3.0, min=0.1, max=90.0, unit='deg/s', doc='still: gyro rate below this'
    )
    max_accel_spread: float = param(
        0.03, min=0.001, max=0.5, doc='still: accelerometer std / mean norm below this'
    )
    max_tilt_deg: float = param(
        20.0, min=1.0, max=60.0, unit='deg', doc='larger start tilts are refused (wrong frame?)'
    )
    timeout_sec: float = param(
        15.0, min=0.0, max=600.0, unit='s', doc='then the bridge goes up unleveled; 0: wait'
    )


@dataclass(frozen=True)
class LocalizationConfig:
    """Frames of the bridge, files and rates of the localization node."""

    sites_dir: str = param('~/x30/map', live=False, doc='load_tags paths must be inside')
    tags_file: str = param('', live=False, doc='tags file at start (empty: none until load_tags)')
    lio_odom_frame: str = param('camera_init', live=False, doc='LIO world frame')
    lio_body_frame: str = param('body', live=False, doc='LIO body frame (the IMU)')
    imu_frame: str = param('imu_link', live=False, doc='URDF frame of the LIO body')
    publish_hz: float = param(
        20.0, min=1.0, max=200.0, unit='Hz', live=False, doc='map->odom on /tf'
    )
    pose_hz: float = param(
        10.0, min=0.0, max=100.0, unit='Hz', live=False, doc='localization/pose; 0: off'
    )
    assume_start_origin: bool = param(
        True, live=False, doc='the odom origin is the map origin at start (no tag needed)'
    )
    estimator: EstimatorConfig = nested(EstimatorConfig)
    tag: TagConfig = nested(TagConfig)
    leveling: LevelingConfig = nested(LevelingConfig)
