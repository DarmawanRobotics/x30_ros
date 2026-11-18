"""OdometryMonitor: whether the estimator's odometry can be trusted (LOST, DEGRADED, OK)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from x30_odometry.core.residual import SpeedResidual
from x30_odometry.core.restart import PoseSample, is_jump, restart_reason, step_of
from x30_odometry.core.still import StillWindow, angle_to_up
from x30_utils.math import quat_to_matrix


class OdometryState(Enum):
    """Trust in the odometry (values of x30_msgs/OdometryStatus)."""

    INITIALIZING = 0
    OK = 1
    DEGRADED = 2
    LOST = 3


class EventKind(Enum):
    """One-off problems of a single pose."""

    JUMP = 'jump'
    RESTART = 'restart'


@dataclass(frozen=True)
class Event:
    """A jump or a restart found on a pose."""

    kind: EventKind
    message: str


@dataclass(frozen=True)
class MonitorStatus:
    """The monitor's verdict: state, estimator speed (m/s), body tilt in odom (rad), why."""

    state: OdometryState
    speed: float
    tilt: float
    message: str


class OdometryMonitor:
    """Checks every estimator pose and derives the odometry state."""

    def __init__(self, config, max_speed_mps: float):
        self.config = config
        self.max_speed_mps = max_speed_mps
        self.residual = SpeedResidual(config.speed)
        self.imu = StillWindow(config.tilt.still_sec, config.tilt.min_samples)
        self.last: PoseSample | None = None
        self.last_rx: float | None = None
        self.event: tuple[Event, float] | None = None  # (event, until)
        self.tilt = 0.0  # body in odom, last measured while still
        self.frame_tilt = 0.0  # odom z against gravity, last measured while still

    def configure(self, config, max_speed_mps: float):
        """Apply a new configuration (live parameter change)."""
        self.config, self.max_speed_mps = config, max_speed_mps
        self.residual.configure(config.speed)
        self.imu.window_sec, self.imu.min_samples = config.tilt.still_sec, config.tilt.min_samples

    def on_imu(self, now: float, gyro, acc):
        """Add an IMU sample (axes of the odometry body)."""
        self.imu.add(now, gyro, acc)

    def on_leg(self, now: float, vx: float, vy: float):
        """Add the robot odometry twist (base frame, m/s)."""
        self.residual.add_leg(now, vx, vy)

    def on_odometry(self, now: float, pose: PoseSample) -> Event | None:
        """Check a new estimator pose received at now; return a jump or restart, if any."""
        prev = self.last
        if prev is not None and pose.stamp == prev.stamp:
            return None  # the same pose again (republished)
        self.last_rx, self.last = now, pose
        if prev is None:
            self._track(now, pose)
            return None
        dt = pose.stamp - prev.stamp
        step = step_of(prev, pose)
        jumped = dt >= 0.0 and is_jump(step, dt, self.max_speed_mps, self.config.jump)
        reason = restart_reason(prev, pose, jumped, self.config.restart)
        if reason:
            self.residual.reset()
            self.tilt = self.frame_tilt = 0.0
            self._track(now, pose)
            return self._flag(now, Event(EventKind.RESTART, f'restarted: {reason}'))
        if jumped:
            self.residual.reset()  # the speed over the jump means nothing
            return self._flag(now, Event(EventKind.JUMP, f'jumped {step:.2f} m in {dt:.2f} s'))
        self._track(now, pose)
        return None

    def status(self, now: float) -> MonitorStatus:
        """Return the odometry state now."""
        speed, tilt = self.residual.speed, self.tilt
        if self.last_rx is None:
            return MonitorStatus(OdometryState.INITIALIZING, 0.0, tilt, 'waiting for odometry')
        age = now - self.last_rx
        if age > self.config.stale_sec:
            return MonitorStatus(OdometryState.LOST, 0.0, tilt, f'no odometry for {age:.1f} s')
        if self.event is not None and now < self.event[1]:
            return MonitorStatus(OdometryState.LOST, speed, tilt, self.event[0].message)
        problems = self.problems()
        if problems:
            return MonitorStatus(OdometryState.DEGRADED, speed, tilt, '; '.join(problems))
        return MonitorStatus(OdometryState.OK, speed, tilt, '')

    def problems(self) -> list[str]:
        """Return the lasting problems of the odometry (DEGRADED)."""
        c = self.config
        out = []
        z = float(self.last.position[2]) if self.last is not None else 0.0
        if c.max_z_drift_m > 0.0 and abs(z) > c.max_z_drift_m:
            out.append(f'z drift {z:+.2f} m (restart it standing still)')
        if c.tilt.max_deg > 0.0 and self.frame_tilt > math.radians(c.tilt.max_deg):
            out.append(f'odom tilted {math.degrees(self.frame_tilt):.1f} deg against gravity')
        if self.residual.alarm:
            out.append(
                f'speed {self.residual.speed:.2f} m/s, robot {self.residual.leg_speed:.2f} m/s'
            )
        return out

    def _track(self, now: float, pose: PoseSample):
        self.residual.add_pose(now, pose.stamp, float(pose.position[0]), float(pose.position[1]))
        self._measure_tilt(now, pose)

    def _measure_tilt(self, now: float, pose: PoseSample):
        t = self.config.tilt
        stillness = self.imu.measure(now)
        if stillness is None or stillness.problems(t.max_gyro_rad_s, t.max_acc_rel_std):
            return
        rotation = quat_to_matrix(pose.orientation)  # body -> odom
        self.tilt = angle_to_up(rotation[:, 2])
        self.frame_tilt = angle_to_up(rotation @ stillness.up)

    def _flag(self, now: float, event: Event) -> Event:
        self.event = (event, now + self.config.hold_sec)
        return event
