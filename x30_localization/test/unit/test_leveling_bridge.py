"""O1 leveling + bridge, T1/T3 timeline stamps."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_localization.core.bridge import make_bridge
from x30_localization.core.leveling import (
    StillLimits,
    StillWindow,
    level_rotation,
    tilt_angle,
)
from x30_localization.core.timeline import MapOdomTimeline
from x30_utils.math import (
    Pose2D,
    compose,
    default_rng,
    inverse,
    make_transform,
    quat_from_rpy,
    tilt_of,
    yaw_of,
)

G = 9.81
# URDF base_link -> livox_imu_frame: offset, and the LiDAR mounted pitched 10 deg, yawed 3 deg
BASE_IMU = make_transform((0.14, -0.02, 0.13), quat_from_rpy(0.0, math.radians(10.0), 0.05))


def imu_reading(world_base: np.ndarray) -> np.ndarray:
    """Specific force (up, in the IMU axes) for a base pose in a gravity-aligned world."""
    r_world_imu = (world_base @ BASE_IMU)[:3, :3]
    return r_world_imu.T @ np.array([0.0, 0.0, G])


def still_window(force, n=100, rate=0.0) -> StillWindow:
    window = StillWindow(StillLimits(window_sec=1.0))
    rng = default_rng(3)
    for i in range(n):
        noise = rng.normal(0.0, 0.01, 3)
        window.add(i * 0.01, force + noise, (rate, 0.0, 0.0))
    return window


def test_level_rotation_removes_the_start_tilt_of_the_base():
    """Base starts rolled 4 deg, pitched -3 deg: odom must come out level, yaw kept."""
    start = make_transform((0.0, 0.0, 0.35), quat_from_rpy(math.radians(4), math.radians(-3), 0.7))
    mean, reason = still_window(imu_reading(start)).mean_specific_force()
    assert reason == ''
    level, reason = level_rotation(mean, BASE_IMU, np.eye(4), math.radians(20.0))
    assert reason == ''
    roll, pitch = tilt_of(_m(level))
    assert roll == pytest.approx(math.radians(4), abs=2e-3)
    assert pitch == pytest.approx(math.radians(-3), abs=2e-3)
    # the bridge then gives odom->base = L at start: the base tilt, seen from a level odom
    bridge = make_bridge(BASE_IMU, level)
    odom_base = bridge.odom_base(np.eye(4))
    assert np.allclose(odom_base[:3, 3], 0.0)
    assert yaw_of(odom_base) == pytest.approx(0.0, abs=1e-12)  # odom x = base heading at start
    world_odom = start @ inverse(odom_base)
    assert tilt_angle(world_odom[:3, :3]) == pytest.approx(0.0, abs=2e-3)


def test_level_uses_the_lio_rotation_since_start():
    """Leveling later (robot turned and moved): R_camera_init_body undoes the motion."""
    start = make_transform((0, 0, 0.35), quat_from_rpy(math.radians(-5), math.radians(2), 0.0))
    turn = make_transform((1.0, 0.5, 0.0), quat_from_rpy(0.0, 0.0, 1.2))
    now = start @ turn
    init_body = inverse(BASE_IMU) @ turn @ BASE_IMU  # LIO: the IMU since start
    level, reason = level_rotation(imu_reading(now), BASE_IMU, init_body, math.radians(20.0))
    assert reason == ''
    roll, pitch = tilt_of(_m(level))
    assert (roll, pitch) == pytest.approx((math.radians(-5), math.radians(2)), abs=1e-6)


def test_level_bridge_keeps_lio_motion_planar_on_a_tilted_start():
    """Unleveled, driving straight on a level floor climbs in odom; leveled it does not."""
    start = make_transform((0, 0, 0.35), quat_from_rpy(0.0, math.radians(-4), 0.0))
    drive = make_transform((5.0, 0.0, 0.0))
    world_base = drive @ start  # drove 5 m along the floor
    init_body = inverse(start @ BASE_IMU) @ world_base @ BASE_IMU
    level, _ = level_rotation(imu_reading(start), BASE_IMU, np.eye(4), math.radians(20))
    flat = make_bridge(BASE_IMU, level).odom_base(init_body)
    tilted = make_bridge(BASE_IMU, None).odom_base(init_body)
    assert flat[2, 3] == pytest.approx(0.0, abs=1e-6)
    assert abs(tilted[2, 3]) > 0.3  # 5 m * sin(4 deg)
    assert flat[0, 3] == pytest.approx(5.0, abs=1e-6)


def test_bridge_without_leveling_is_the_old_chain():
    bridge = make_bridge(BASE_IMU, None)
    assert np.allclose(bridge.odom_init, BASE_IMU)
    assert np.allclose(compose(bridge.odom_init, bridge.body_base), np.eye(4))
    assert not bridge.leveled


def test_still_window_refuses_motion_shaking_and_large_tilt():
    flat = np.array([0.0, 0.0, G])
    assert still_window(flat, rate=0.2).mean_specific_force()[1].startswith('moving')
    shaky = StillWindow()
    for i in range(100):
        shaky.add(i * 0.01, flat + np.array([0.0, 0.0, 3.0 * (-1) ** i]), (0.0, 0.0, 0.0))
    assert shaky.mean_specific_force()[1].startswith('shaking')
    assert 'samples' in StillWindow().mean_specific_force()[1]
    upside = level_rotation((0.0, 0.0, -G), np.eye(4), np.eye(4), math.radians(20.0))
    assert upside[0] is None and 'tilt' in upside[1]
    in_g = level_rotation((0.0, 0.05, 1.0), np.eye(4), np.eye(4), math.radians(20.0))
    assert in_g[0] is not None  # units do not matter, only the direction


def test_timeline_steps_within_epsilon_never_ramps():
    """T1/T3: the old value is stamped just before the new one."""
    line = MapOdomTimeline(epsilon=1e-3)
    assert line.tick(1.0) == []
    first = line.change(Pose2D(1.0, 0.0, 0.0), 1.0)
    assert first == [(Pose2D(1.0, 0.0, 0.0), 1.0)]
    assert line.tick(1.05) == [(Pose2D(1.0, 0.0, 0.0), 1.05)]
    out = line.change(Pose2D(2.0, 0.0, 0.0), 1.07)
    assert out == [(Pose2D(1.0, 0.0, 0.0), pytest.approx(1.069)), (Pose2D(2.0, 0.0, 0.0), 1.07)]
    assert line.tick(1.07) == []  # never twice at the same stamp
    # a correction right after a tick still gets increasing stamps
    line.tick(1.1)
    out = line.change(Pose2D(3.0, 0.0, 0.0), 1.1)
    stamps = [s for _, s in out]
    assert stamps == sorted(stamps) and stamps[0] > 1.1 - 1e-9 and stamps[-1] > stamps[0]


def _m(r3):
    m = np.eye(4)
    m[:3, :3] = r3
    return m
