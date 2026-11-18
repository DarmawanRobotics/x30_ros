"""The simulated dock: the tag camera (view, range, walls) and the dock station services."""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path

import numpy as np
import pytest

from x30_sim.core.body import Twist
from x30_sim.core.config import BatteryConfig, CameraConfig, SimConfig
from x30_sim.core.dock import DockSpec, DockStation, TagCamera
from x30_sim.core.simulation import Simulation
from x30_sim.core.world import World
from x30_utils.math import Pose2D, compose, default_rng
from x30_utils.robot import RobotProfile

WORLD = World.from_yaml(str(Path(__file__).resolve().parents[2] / 'config/worlds/mall_loop.yaml'))
EXACT = replace(CameraConfig(), noise_m=0.0)


def camera(config=EXACT):
    return TagCamera(config, WORLD, default_rng(0))


def test_camera_sees_the_dock_tag_in_front_with_the_world_convention():
    pose = Pose2D(-4.5, 0.0, math.pi)  # base_link 1.5 m in front of the west wall, facing it
    seen = camera().detections(pose)
    assert set(seen) == {'charging_dock'}
    tag_in_map = compose(pose.to_matrix(), seen['charging_dock'])
    assert tag_in_map[:3, 3] == pytest.approx([-6.0, 0.0, 0.3], abs=1e-9)
    assert tag_in_map[:2, 2] == pytest.approx([1.0, 0.0], abs=1e-9)  # z toward the viewer


def test_camera_misses_tags_behind_out_of_range_too_close_or_hidden():
    assert not camera().detections(Pose2D(-4.5, 0.0, 0.0))  # facing away
    assert not camera().detections(Pose2D(-5.4, 0.0, math.pi))  # camera closer than range_min
    far = replace(EXACT, range_max=0.9)  # the camera is 1.0 m from the tag, 0.5 m ahead of base
    assert not camera(far).detections(Pose2D(-4.5, 0.0, math.pi))
    # from the shops' east side the block of shops hides the west wall
    assert not camera(replace(EXACT, range_max=20.0)).detections(Pose2D(4.5, 0.0, math.pi))
    rear = replace(EXACT, yaw_deg=180.0)
    assert set(camera(rear).detections(Pose2D(-4.5, 0.0, 0.0))) == {'charging_dock'}


def test_camera_optical_frame_looks_along_its_yaw():
    """base_link -> optical frame: z ahead (or behind), x to the right, y down."""
    front = camera().base_camera
    assert front[:3, 3] == pytest.approx([0.5, 0.0, 0.1])
    assert front[:3, 2] == pytest.approx([1.0, 0.0, 0.0], abs=1e-9)
    assert front[:3, 0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-9)
    assert front[:3, 1] == pytest.approx([0.0, 0.0, -1.0], abs=1e-9)
    rear = camera(replace(EXACT, yaw_deg=180.0)).base_camera
    assert rear[:3, 2] == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)


def test_station_charges_only_sitting_on_the_contacts():
    station = DockStation(DockSpec.from_dict(WORLD.dock), WORLD)
    contact = station.contact
    assert contact.x == pytest.approx(-5.0) and contact.y == pytest.approx(0.0)  # 1.0 m off
    assert abs(math.remainder(contact.yaw - math.pi, 2 * math.pi)) < 1e-9
    assert not station.start_charge(contact)[0]  # standing
    ok, message = station.sit(speed=0.3)
    assert not ok and 'walking' in message and not station.sitting
    assert station.sit()[0]
    ok, message = station.start_charge(Pose2D(contact.x + 0.2, 0.0, math.pi))
    assert not ok and 'not on the dock' in message
    assert station.start_charge(contact)[0] and station.charging
    assert not station.stand()[0]  # charging
    assert station.stop_charge()[0] and station.stand()[0] and not station.sitting


def test_sitting_holds_the_body_and_charging_fills_the_battery():
    battery = replace(BatteryConfig(), charge_per_hour=36.0, drain_per_hour=0.0)
    config = replace(SimConfig(), battery=battery)
    sim = Simulation(config, RobotProfile(), WORLD)
    sim.body.pose = sim.station.contact
    sim.step(0.0)
    sim.station.sit()
    sim.command(0.0, Twist(0.5, 0.0, 0.0))
    for k in range(1, 11):
        sim.step(0.05 * k)
    assert sim.body.pose.distance(sim.station.contact) < 1e-9  # commands ignored
    start = sim.battery
    assert sim.station.start_charge(sim.body.pose)[0]
    for k in range(11, 31):
        sim.step(0.05 * k)
    assert sim.battery == pytest.approx(start + 0.01, abs=1e-6)  # 36 per hour, 1 s
    assert np.isfinite(sim.battery)
