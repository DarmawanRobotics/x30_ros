"""Configuration of wait_still (StillConfig) and odometry_monitor_node (MonitorConfig)."""

from __future__ import annotations

from dataclasses import dataclass

from x30_utils.config import nested, param


@dataclass(frozen=True)
class StillConfig:
    """When the IMU counts as still and level enough to start the estimator (wait_still)."""

    imu_topic: str = param('/imu/data', live=False, doc='IMU topic (sensor_msgs/Imu)')
    still_sec: float = param(
        2.0, min=0.2, max=60.0, unit='s', doc='this long without motion before the start'
    )
    max_gyro_rad_s: float = param(
        0.03, min=0.0, max=10.0, unit='rad/s', doc='largest angular rate in the window'
    )
    max_acc_rel_std: float = param(
        0.01, min=0.0, max=1.0, doc='std of |acc| / mean |acc| (unit-free: m/s^2 or g)'
    )
    max_tilt_deg: float = param(
        3.0, min=0.0, max=90.0, unit='deg', doc='gravity against the IMU z axis; 0: off'
    )
    min_samples: int = param(20, min=2, max=10000, doc='IMU samples needed in the window')
    timeout_sec: float = param(
        60.0, min=0.0, max=3600.0, unit='s', doc='start anyway (with an error); 0: wait forever'
    )


@dataclass(frozen=True)
class RestartConfig:
    """When a pose means the estimator restarted (its frame was reset)."""

    gap_sec: float = param(
        2.0, min=0.1, max=60.0, unit='s', doc='a message gap this long followed by the origin'
    )
    origin_radius_m: float = param(
        0.05, min=0.0, max=1.0, unit='m', doc='a fresh estimator starts this close to the origin'
    )
    origin_yaw_deg: float = param(
        1.0, min=0.0, max=180.0, unit='deg', doc='and with this little yaw'
    )


@dataclass(frozen=True)
class JumpConfig:
    """When a step between two poses is impossible for the robot."""

    speed_factor: float = param(
        3.0, min=1.0, max=100.0, doc='a step faster than this times the robot speed limit'
    )
    min_step_m: float = param(
        0.3, min=0.0, max=100.0, unit='m', doc='and longer than this is a jump'
    )


@dataclass(frozen=True)
class SpeedConfig:
    """Speed residual test: r = v_lio - v_leg, chi^2 = r^2 / (var_lio + var_leg), CUSUM."""

    window_sec: float = param(
        0.5, min=0.05, max=10.0, unit='s', doc='estimator speed from the poses of this window'
    )
    sigma_lio_mps: float = param(
        0.05, min=0.001, max=10.0, unit='m/s', doc='std of the estimator speed'
    )
    sigma_leg_mps: float = param(
        0.15, min=0.001, max=10.0, unit='m/s', doc='std of the leg / wheel odometry speed'
    )
    cusum_drift: float = param(
        3.84, min=0.0, max=1000.0, doc='kappa, subtracted from every chi^2 (chi^2(1) 95 %: 3.84)'
    )
    cusum_threshold: float = param(
        20.0, min=0.1, max=10000.0, doc='h: the CUSUM above this is a mismatch'
    )
    leg_timeout_sec: float = param(
        0.3, min=0.01, max=10.0, unit='s', doc='older leg odometry is not compared'
    )


@dataclass(frozen=True)
class TiltConfig:
    """Level of the odometry frame, measured against gravity while standing still."""

    max_deg: float = param(
        5.0, min=0.0, max=90.0, unit='deg', doc='odom z against gravity above this; 0: off'
    )
    still_sec: float = param(1.0, min=0.2, max=60.0, unit='s', doc='IMU window')
    max_gyro_rad_s: float = param(0.03, min=0.0, max=10.0, unit='rad/s', doc='still: rate')
    max_acc_rel_std: float = param(0.01, min=0.0, max=1.0, doc='still: std of |acc| / mean')
    min_samples: int = param(20, min=2, max=10000, doc='IMU samples needed in the window')


@dataclass(frozen=True)
class TripConfig:
    """Trip meter: distance and speeds of the estimator since the last reset (odometry/trip)."""

    enabled: bool = param(True, live=False, doc='odometry/trip and odometry/trip_reset')
    file: str = param(
        '~/.x30/trip.json', live=False, doc='totals kept across restarts; empty: not kept'
    )
    step_m: float = param(
        0.05, min=0.001, max=1.0, unit='m', doc='distance grows in steps of this (pose noise: 0)'
    )
    window_sec: float = param(
        1.0, min=0.1, max=10.0, unit='s', doc='the current speed over this window'
    )
    moving_mps: float = param(
        0.05, min=0.0, max=1.0, unit='m/s', doc='slower counts as standing (moving time, speed 0)'
    )
    gap_sec: float = param(
        1.0, min=0.1, max=60.0, unit='s', doc='a longer pose gap counts nothing across it'
    )
    max_mps: float = param(
        3.0, min=0.1, max=50.0, unit='m/s', doc='a faster step is a jump, not travelled'
    )
    period_sec: float = param(
        0.5, min=0.05, max=10.0, unit='s', live=False, doc='odometry/trip period'
    )
    save_sec: float = param(
        10.0, min=1.0, max=3600.0, unit='s', live=False, doc='totals written at most this often'
    )


@dataclass(frozen=True)
class MonitorConfig:
    """odometry_monitor_node: when the estimator's odometry can be trusted."""

    odom_topic: str = param('/Odometry', live=False, doc='estimator odometry (nav_msgs/Odometry)')
    leg_odom_topic: str = param(
        '', live=False, doc='robot leg / wheel odometry for the speed test; empty: off'
    )
    imu_topic: str = param(
        '/imu/data', live=False, doc='IMU with the axes of the odometry body; empty: no tilt'
    )
    status_period_sec: float = param(
        0.2, min=0.01, max=10.0, unit='s', live=False, doc='odometry/status period'
    )
    stale_sec: float = param(0.5, min=0.05, max=60.0, unit='s', doc='no odometry this long: LOST')
    hold_sec: float = param(
        5.0, min=0.0, max=600.0, unit='s', doc='a jump or restart stays LOST this long'
    )
    max_z_drift_m: float = param(
        0.5, min=0.0, max=100.0, unit='m', doc='|z| above this is DEGRADED (flat floor); 0: off'
    )
    restart: RestartConfig = nested(RestartConfig)
    jump: JumpConfig = nested(JumpConfig)
    speed: SpeedConfig = nested(SpeedConfig)
    tilt: TiltConfig = nested(TiltConfig)
    trip: TripConfig = nested(TripConfig)

    def validate(self) -> list[str]:
        """Return cross-field problems."""
        if self.restart.gap_sec <= self.stale_sec:
            return ['restart.gap_sec must be above stale_sec']
        return []
