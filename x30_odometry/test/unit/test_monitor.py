"""OdometryMonitor state transitions."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_odometry.core.config import MonitorConfig, TiltConfig
from x30_odometry.core.monitor import EventKind, OdometryMonitor, OdometryState
from x30_odometry.core.restart import PoseSample

MAX_SPEED = 1.0


def quat(roll=0.0, pitch=0.0, yaw=0.0):
    """Quaternion (x, y, z, w) of roll / pitch / yaw (rad)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class Sim:
    """Feeds a monitor at 10 Hz with receipt time == stamp."""

    def __init__(self, config=None):
        self.monitor = OdometryMonitor(config or MonitorConfig(), MAX_SPEED)
        self.t, self.x, self.events = 100.0, 0.0, []

    def odom(self, dx=0.0, x=None, z=0.0, orientation=(0.0, 0.0, 0.0, 1.0), dt=0.1, leg=None):
        self.t += dt
        self.x = self.x + dx if x is None else x
        if leg is not None:
            self.monitor.on_leg(self.t, leg, 0.0)
        p = PoseSample(self.t, np.array([self.x, 0.0, z]), orientation)
        event = self.monitor.on_odometry(self.t, p)
        if event:
            self.events.append(event)
        return self.monitor.status(self.t)

    def run(self, seconds, **kwargs):
        status = None
        for _ in range(round(seconds * 10)):
            status = self.odom(**kwargs)
        return status

    def imu(self, seconds, up_body, gyro=0.0, rate_hz=100.0):
        t0 = self.t
        for k in range(int(seconds * rate_hz)):
            self.monitor.on_imu(t0 + k / rate_hz, (0.0, 0.0, gyro), up_body)


def test_initializing_until_the_first_pose():
    sim = Sim()
    status = sim.monitor.status(0.0)
    assert status.state is OdometryState.INITIALIZING and status.message


def test_smooth_motion_is_ok_and_reports_speed():
    sim = Sim()
    status = sim.run(2.0, dx=0.05)
    assert status.state is OdometryState.OK and abs(status.speed - 0.5) < 1e-6


def test_stale_odometry_is_lost():
    sim = Sim()
    sim.run(1.0, dx=0.05)
    status = sim.monitor.status(sim.t + 1.0)
    assert status.state is OdometryState.LOST and status.message.startswith('no odometry')


def test_duplicate_stamp_is_ignored():
    sim = Sim()
    sim.run(1.0, dx=0.05)
    status = sim.odom(x=sim.x + 2.0, dt=0.0)  # the same stamp again
    assert status.state is OdometryState.OK and not sim.events


def test_jump_is_lost_for_hold_sec_then_ok():
    sim = Sim()
    sim.run(1.0, dx=0.05)
    status = sim.odom(dx=2.0)
    assert status.state is OdometryState.LOST and status.message.startswith('jumped 2.00 m')
    assert [e.kind for e in sim.events] == [EventKind.JUMP]
    status = sim.run(MonitorConfig().hold_sec + 0.2, dx=0.05)
    assert status.state is OdometryState.OK


def test_turn_in_place_near_the_start_keeps_ok():
    """#7: turning in place 2 cm from the start pose is not a restart."""
    sim = Sim()
    sim.run(0.5, x=0.02, orientation=quat(yaw=math.radians(90.0)))
    for yaw_deg in range(90, -1, -5):
        status = sim.odom(x=0.02, orientation=quat(yaw=math.radians(yaw_deg)))
    assert status.state is OdometryState.OK and not sim.events


def test_gap_back_at_the_origin_is_a_restart():
    sim = Sim()
    sim.run(3.0, dx=0.1)
    status = sim.odom(x=0.0, dt=6.0)
    assert [e.kind for e in sim.events] == [EventKind.RESTART]
    assert status.state is OdometryState.LOST and status.message.startswith('restarted')
    assert sim.run(MonitorConfig().hold_sec + 0.2).state is OdometryState.OK


def test_jump_back_to_the_origin_is_a_restart():
    sim = Sim()
    sim.run(3.0, dx=0.1)
    sim.odom(x=0.0)
    assert [e.kind for e in sim.events] == [EventKind.RESTART]


def test_z_drift_is_degraded():
    sim = Sim()
    status = sim.run(1.0, dx=0.05, z=0.8)
    assert status.state is OdometryState.DEGRADED and 'z drift' in status.message
    off = Sim(MonitorConfig(max_z_drift_m=0.0))
    assert off.run(1.0, dx=0.05, z=0.8).state is OdometryState.OK


def test_speed_mismatch_is_degraded():
    sim = Sim()
    sim.run(1.0, dx=0.0, leg=0.0)
    status = sim.run(3.0, dx=0.0, leg=0.6)
    assert status.state is OdometryState.DEGRADED and 'robot 0.60 m/s' in status.message


def test_level_body_in_a_tilted_odom_is_degraded():
    """O1: started tilted, standing level now: the body reads 6 deg tilted, against gravity."""
    sim = Sim()
    tilted = quat(pitch=math.radians(6.0))
    sim.imu(1.5, up_body=(0.0, 0.0, 1.0))
    status = sim.odom(orientation=tilted)
    assert status.state is OdometryState.DEGRADED and 'odom tilted 6.0 deg' in status.message
    assert math.degrees(status.tilt) == pytest.approx(6.0, abs=0.05)


def test_tilted_body_in_a_level_odom_is_ok():
    """A robot standing on a 10 deg ramp: the body is tilted, odom is level."""
    sim = Sim()
    pitch = math.radians(10.0)
    sim.imu(1.5, up_body=(-math.sin(pitch), 0.0, math.cos(pitch)))
    status = sim.odom(orientation=quat(pitch=pitch))
    assert status.state is OdometryState.OK
    assert math.degrees(status.tilt) == pytest.approx(10.0, abs=0.05)


def test_tilt_is_only_measured_while_still():
    sim = Sim(MonitorConfig(tilt=TiltConfig(max_deg=3.0)))
    sim.imu(1.5, up_body=(0.0, 0.0, 1.0), gyro=0.3)
    status = sim.odom(orientation=quat(roll=math.radians(8.0)))
    assert status.state is OdometryState.OK and status.tilt == 0.0


def test_restart_clears_the_tilt():
    sim = Sim()
    sim.run(3.0, dx=0.1)
    sim.imu(1.5, up_body=(0.0, 0.0, 1.0))
    sim.odom(dx=0.0, orientation=quat(roll=math.radians(7.0)))
    assert sim.monitor.frame_tilt > 0.1
    sim.odom(x=0.0, dt=6.0)
    assert sim.monitor.frame_tilt == 0.0 and sim.monitor.tilt == 0.0


def test_reconfigure_applies_new_thresholds():
    sim = Sim()
    sim.monitor.configure(MonitorConfig(max_z_drift_m=0.1), MAX_SPEED)
    assert sim.run(0.5, z=0.2).state is OdometryState.DEGRADED
