"""Configuration of sim_robot_node (parameters lidar.*, odometry.*, latency.* and the rest)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param

MOUNT_VALUES = 5  # x, y, z, azimuth_deg, elevation_deg per LiDAR

# Application manual 6.2: the origins in base_link and the direction of each dome (the Mid-360's
# z axis): front up 25 deg, front down 46 deg, rear 5 deg and 56 deg below the horizon
X30_LIDARS = (
    (0.44367, 0.0, 0.05415, 0.0, 25.0),
    (0.43297, 0.0, -0.05519, 0.0, -46.0),
    (-0.46576, 0.0, 0.03484, 180.0, -5.0),
    (-0.41074, 0.0, -0.05601, 180.0, -56.0),
)


@dataclass(frozen=True)
class LidarConfig:
    """The X30's four Livox Mid-360, merged into one cloud in base_link like /livox/lidar."""

    rate_hz: float = param(10.0, min=0.5, max=50.0, unit='Hz', live=False, doc='scans per second')
    frame: str = param('livox_frame', live=False, doc='frame of the merged cloud (= base_link)')
    mounts: tuple[float, ...] = param(
        tuple(v for mount in X30_LIDARS for v in mount),
        live=False,
        doc='per LiDAR x y z (m, base_link), dome azimuth and elevation (deg)',
    )
    azimuth_rays: int = param(360, min=8, max=3600, live=False, doc='rays per ring over 360 deg')
    rings: int = param(16, min=1, max=128, live=False, doc='elevations over the vertical fov')
    fov_min_deg: float = param(-7.0, min=-90.0, max=90.0, unit='deg', live=False, doc='lowest')
    fov_max_deg: float = param(52.0, min=-90.0, max=90.0, unit='deg', live=False, doc='highest')
    range_min: float = param(0.1, min=0.0, max=10.0, unit='m', live=False, doc='closer: no return')
    range_max: float = param(40.0, min=0.5, max=500.0, unit='m', live=False, doc='no return')
    noise_m: float = param(
        0.01, min=0.0, max=1.0, unit='m', live=False, doc='range noise, 1 sigma'
    )
    point_dropout: float = param(
        0.0, min=0.0, max=1.0, live=False, doc='fraction of returns lost at random'
    )
    floor_returns: bool = param(True, live=False, doc='rays pointing down hit the floor')

    @property
    def mount_list(self) -> list[tuple[float, ...]]:
        """Return (x, y, z, azimuth_deg, elevation_deg) of every LiDAR."""
        v = self.mounts
        return [tuple(v[i : i + MOUNT_VALUES]) for i in range(0, len(v), MOUNT_VALUES)]

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if not self.mounts or len(self.mounts) % MOUNT_VALUES:
            problems.append('mounts needs five values per LiDAR: x y z azimuth_deg elevation_deg')
        if self.fov_min_deg >= self.fov_max_deg:
            problems.append('fov_min_deg must be below fov_max_deg')
        if self.range_min >= self.range_max:
            problems.append('range_min must be below range_max')
        return problems


@dataclass(frozen=True)
class OdometryNoiseConfig:
    """Drift of a simulated odometry, per metre travelled."""

    sigma_x: float = param(
        0.005, min=0.0, max=1.0, unit='m/sqrt(m)', live=False, doc='along-track noise'
    )
    sigma_y: float = param(
        0.005, min=0.0, max=1.0, unit='m/sqrt(m)', live=False, doc='cross-track noise (slip)'
    )
    sigma_yaw_deg: float = param(
        0.1, min=0.0, max=10.0, unit='deg/sqrt(m)', live=False, doc='heading noise'
    )
    scale_error: float = param(
        0.0, min=-0.5, max=0.5, live=False, doc='measured / true distance - 1'
    )
    yaw_bias_deg_per_m: float = param(
        0.0, min=-10.0, max=10.0, unit='deg/m', live=False, doc='heading bias'
    )
    rotation_weight: float = param(
        0.2, min=0.0, max=10.0, unit='m/rad', live=False, doc='turning counts as this travel'
    )


@dataclass(frozen=True)
class LegOdometryNoiseConfig(OdometryNoiseConfig):
    """Drift of x30_driver's leg odometry: more than the LiDAR-inertial odometry's."""

    sigma_x: float = param(
        0.02, min=0.0, max=1.0, unit='m/sqrt(m)', live=False, doc='along-track noise'
    )
    sigma_y: float = param(
        0.02, min=0.0, max=1.0, unit='m/sqrt(m)', live=False, doc='cross-track noise (slip)'
    )
    sigma_yaw_deg: float = param(
        0.5, min=0.0, max=10.0, unit='deg/sqrt(m)', live=False, doc='heading noise'
    )


@dataclass(frozen=True)
class LioConfig:
    """The FAST-LIO stand-in: per scan /Odometry, TF and /cloud_registered_body (body: IMU)."""

    odom_frame: str = param('camera_init', live=False, doc='FAST-LIO world frame')
    body_frame: str = param('body', live=False, doc='FAST-LIO body frame (the IMU)')
    blind: float = param(
        0.7, min=0.0, max=10.0, unit='m', live=False, doc='no points this close to base_link'
    )


@dataclass(frozen=True)
class ImuConfig:
    """The body IMU (/imu/data): axes of base_link, at its URDF origin."""

    rate_hz: float = param(200.0, min=0.0, max=1000.0, unit='Hz', live=False, doc='0: off')
    topic: str = param('/imu/data', live=False, doc='sensor_msgs/Imu, m/s^2 and rad/s')
    frame: str = param('imu_link', live=False, doc='URDF frame of the IMU')
    xyz: tuple[float, ...] = param(
        (0.13045, -0.001, 0.0149), unit='m', live=False, doc='imu_link in base_link (URDF)'
    )

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        return [] if len(self.xyz) == 3 else ['xyz needs three values']


@dataclass(frozen=True)
class LatencyConfig:
    """Sensor stamps are t - latency; commands act latency after they arrive."""

    lidar_sec: float = param(0.05, min=0.0, max=1.0, unit='s', live=False, doc='cloud age')
    odometry_sec: float = param(0.01, min=0.0, max=1.0, unit='s', live=False, doc='pose age')
    command_sec: float = param(0.02, min=0.0, max=1.0, unit='s', live=False, doc='cmd_vel delay')


@dataclass(frozen=True)
class CameraConfig:
    """A camera with a fiducial detector: the tags it sees, as TF camera frame -> tag."""

    rate_hz: float = param(
        10.0, min=0.0, max=60.0, unit='Hz', live=False, doc='detections per second (0: off)'
    )
    tags: tuple[str, ...] = param(
        ('base_map', 'charging_dock'), live=False, doc='world tags the detector publishes on TF'
    )
    frame: str = param(
        'camera_color_optical_frame', live=False, doc='optical frame (x30_localization tag/)'
    )
    publish_mount: bool = param(
        True, live=False, doc='base_link -> frame on /tf_static (false: the URDF has it)'
    )
    mount_xyz: tuple[float, ...] = param(
        (0.50, 0.0, 0.10), unit='m', live=False, doc='camera in base_link'
    )
    yaw_deg: float = param(
        0.0, min=-180.0, max=180.0, unit='deg', live=False, doc='0: forward, 180: rear camera'
    )
    fov_deg: float = param(
        80.0, min=1.0, max=180.0, unit='deg', live=False, doc='horizontal field of view'
    )
    range_min: float = param(
        0.25, min=0.0, max=10.0, unit='m', live=False, doc='closer the tag fills the image'
    )
    range_max: float = param(4.0, min=0.1, max=50.0, unit='m', live=False, doc='detection range')
    max_incidence_deg: float = param(
        70.0, min=1.0, max=89.0, unit='deg', live=False, doc='a plate seen more obliquely: lost'
    )
    noise_m: float = param(0.003, min=0.0, max=0.5, unit='m', live=False, doc='position noise')
    noise_deg: float = param(0.5, min=0.0, max=20.0, unit='deg', live=False, doc='yaw noise')

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        problems = []
        if len(self.mount_xyz) != 3:
            problems.append('mount_xyz needs three values')
        if self.range_min >= self.range_max:
            problems.append('range_min must be below range_max')
        return problems


@dataclass(frozen=True)
class BatteryConfig:
    """The battery as x30_driver reports it (sensor_msgs/BatteryState)."""

    start: float = param(0.9, min=0.0, max=1.0, live=False, doc='charge at start')
    drain_per_hour: float = param(0.2, min=0.0, max=10.0, live=False, doc='charge lost per hour')
    charge_per_hour: float = param(
        1.0, min=0.0, max=100.0, live=False, doc='charge gained per hour on the dock'
    )
    report_charging: bool = param(
        False, live=False, doc='power_supply_status (x30_driver reports UNKNOWN)'
    )


@dataclass(frozen=True)
class SimConfig:
    """Everything sim_robot_node reads besides the robot profile."""

    world_file: str = param('', live=False, doc='world YAML path or name in config/worlds')
    scenario_file: str = param('', live=False, doc='scenario YAML path or name; empty: none')
    driver_ns: str = param('x30', live=False, doc='x30_driver namespace: /<ns>/cmd_vel, odom')
    seed: int = param(0, min=0, max=2**31 - 1, live=False, doc='RNG seed (a scenario may set it)')
    rate_hz: float = param(100.0, min=5.0, max=1000.0, unit='Hz', live=False, doc='physics')
    odom_rate_hz: float = param(
        50.0, min=1.0, max=500.0, unit='Hz', live=False, doc='<driver_ns>/odom (leg odometry)'
    )
    publish_tf: bool = param(False, live=False, doc='odom -> base_link from the leg odometry')
    max_dt: float = param(
        0.1, min=0.001, max=1.0, unit='s', live=False, doc='longest physics step (stalls)'
    )
    cmd_timeout_sec: float = param(
        0.5, min=0.05, max=10.0, unit='s', live=False, doc='older command: the body stops'
    )
    obstacle_radius: float = param(0.25, min=0.01, max=5.0, unit='m', doc='sim/obstacles size')
    obstacle_height: float = param(1.7, min=0.01, max=5.0, unit='m', doc='sim/obstacles height')
    lidar: LidarConfig = nested(LidarConfig)
    lio: LioConfig = nested(LioConfig)
    imu: ImuConfig = nested(ImuConfig)
    odometry: OdometryNoiseConfig = nested(OdometryNoiseConfig)
    leg_odometry: LegOdometryNoiseConfig = nested(LegOdometryNoiseConfig)
    latency: LatencyConfig = nested(LatencyConfig)
    camera: CameraConfig = nested(CameraConfig)
    battery: BatteryConfig = nested(BatteryConfig)
