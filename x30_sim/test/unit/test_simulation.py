"""Simulation: FAST-LIO at the scan's stamp, the leg odometry beside it, dropouts per sensor."""

from __future__ import annotations

from dataclasses import replace
from pathlib import Path

import pytest

from x30_sim.core.body import Twist
from x30_sim.core.config import OdometryNoiseConfig, SimConfig
from x30_sim.core.scenario import scenario_from_dict
from x30_sim.core.simulation import Simulation
from x30_sim.core.world import World
from x30_utils.math import Pose2D
from x30_utils.robot import RobotProfile

WORLD = World.from_yaml(str(Path(__file__).resolve().parents[2] / 'config/worlds/mall_loop.yaml'))
QUIET = OdometryNoiseConfig(sigma_x=0.0, sigma_y=0.0, sigma_yaw_deg=0.0)
EXACT = replace(SimConfig(), odometry=QUIET, leg_odometry=QUIET)
FORWARD = Twist(0.4)


def drive(sim: Simulation, seconds: float, command: Twist = FORWARD, dt: float = 0.01):
    """Step the simulation under a command; return the last time."""
    t = 0.0
    sim.command(t, command)
    sim.step(t)
    for _ in range(round(seconds / dt)):
        t += dt
        sim.command(t, command)
        sim.step(t)
    return t


def test_fast_lio_pose_is_the_one_of_the_scan():
    """/Odometry carries the scan's stamp: the pose of that time, not of now."""
    sim = Simulation(EXACT, RobotProfile(), WORLD)
    now = drive(sim, 2.0)
    scan = sim.scan(now)
    assert scan.stamp == pytest.approx(now - EXACT.latency.lidar_sec)
    lio = sim.lio(scan.stamp, now)
    true = WORLD.start.between(sim.truth.at(scan.stamp))
    assert lio.distance(true) < 1e-6 and lio.distance(sim.odom.at(now)) > 0.005
    assert len(scan.points) > 1000


def test_leg_odometry_drifts_on_its_own_and_survives_a_fast_lio_restart():
    scenario = scenario_from_dict(
        {'odometry_restarts': [1.0], 'leg_odometry': {'scale_error': 0.1}}
    )
    sim = Simulation(EXACT, RobotProfile(), WORLD, scenario)
    now = drive(sim, 2.0)
    assert sim.restarts == 1
    leg = sim.leg_odometry(now)
    assert leg.stamp == pytest.approx(now - EXACT.latency.odometry_sec)
    travelled = WORLD.start.distance(sim.truth.at(leg.stamp))
    assert leg.odom_base.x == pytest.approx(1.1 * travelled, rel=0.01)  # 10 % long, no restart
    assert sim.lio(now, now).x == pytest.approx(0.4, abs=0.01)  # restarted at 1 s, 0.4 m/s
    assert leg.twist == sim.body.twist


def test_dropouts_silence_each_sensor():
    dropouts = [
        {'start': 1.0, 'duration': 1.0, 'sensor': 'lidar'},
        {'start': 3.0, 'duration': 1.0, 'sensor': 'odometry'},
        {'start': 5.0, 'duration': 1.0, 'sensor': 'leg_odometry'},
        {'start': 7.0, 'duration': 1.0, 'sensor': 'imu'},
        {'start': 9.0, 'duration': 1.0, 'sensor': 'all'},
    ]
    sim = Simulation(EXACT, RobotProfile(), WORLD, scenario_from_dict({'dropouts': dropouts}))
    sim.step(0.0)
    sim.step(0.1)

    def heard(t: float) -> tuple[bool, ...]:
        return (
            sim.scan(t) is not None,
            sim.lio(t - 0.05, t) is not None,
            sim.leg_odometry(t) is not None,
            sim.imu(t) is not None,
            bool(sim.tag_detections(t)) or sim.camera.tags == {},
        )

    sim.body.pose = Pose2D(-4.5, 0.0, 3.14159)  # the dock tag in view
    assert heard(0.5) == (True, True, True, True, True)
    assert heard(1.5) == (False, True, True, True, True)
    assert heard(3.5) == (True, False, True, True, True)
    assert heard(5.5) == (True, True, False, True, True)
    assert heard(7.5) == (True, True, True, False, True)
    assert heard(9.5) == (False, False, False, False, False)
