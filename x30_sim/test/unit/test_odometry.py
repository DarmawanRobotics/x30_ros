"""Odometry drift: random-walk statistics, biases, seeded runs, restarts and the FAST-LIO chain."""

from __future__ import annotations

import math
from pathlib import Path
import re

import numpy as np
import pytest
import yaml

from x30_sim.core.config import ImuConfig, OdometryNoiseConfig
from x30_sim.core.odometry import DriftingOdometry, lio_transform
from x30_sim.core.scenario import make_rngs
from x30_utils.math import Pose2D, compose, default_rng, inverse, make_transform

QUIET = OdometryNoiseConfig(sigma_x=0.0, sigma_y=0.0, sigma_yaw_deg=0.0)
REPO = Path(__file__).resolve().parents[3]


def drive(config: OdometryNoiseConfig, rng, distance: float = 10.0, steps: int = 100):
    """Drive straight along x from the origin; return the final odom pose."""
    odom = DriftingOdometry(config, rng, Pose2D())
    for i in range(1, steps + 1):
        odom.update(Pose2D(distance * i / steps, 0.0, 0.0))
    return odom.pose


def test_perfect_without_noise():
    """S1: with every error zero the odometry is the true motion."""
    odom = DriftingOdometry(QUIET, default_rng(0), Pose2D(1.0, 2.0, 0.5))
    pose = odom.update(Pose2D(2.0, 3.0, 1.0))
    expected = Pose2D(1.0, 2.0, 0.5).between(Pose2D(2.0, 3.0, 1.0))
    assert pose.to_array() == pytest.approx(expected.to_array())


def test_random_walk_grows_with_the_square_root_of_distance():
    """S1: error std after D metres is sigma * sqrt(D) (seeded, 400 runs)."""
    config = OdometryNoiseConfig(
        sigma_x=0.02, sigma_y=0.01, sigma_yaw_deg=0.0, rotation_weight=0.0
    )
    rng = default_rng(42)
    finals = np.array([drive(config, rng).to_array() for _ in range(400)])
    errors = finals - [10.0, 0.0, 0.0]
    assert abs(errors[:, 0].mean()) < 0.01
    assert errors[:, 0].std() == pytest.approx(0.02 * math.sqrt(10.0), rel=0.12)
    assert errors[:, 1].std() == pytest.approx(0.01 * math.sqrt(10.0), rel=0.12)


def test_heading_noise_spreads_the_lateral_error():
    """Yaw noise integrates into a lateral error much larger than the heading error alone."""
    config = OdometryNoiseConfig(sigma_x=0.0, sigma_y=0.0, sigma_yaw_deg=0.5)
    rng = default_rng(7)
    finals = np.array([drive(config, rng).to_array() for _ in range(200)])
    assert finals[:, 2].std() == pytest.approx(math.radians(0.5) * math.sqrt(10.0), rel=0.15)
    assert finals[:, 1].std() > 0.05


def test_scale_error_and_yaw_bias_accumulate():
    """A 2 % scale error and 0.1 deg/m yaw bias give 10.2 m and 1 deg after 10 m."""
    config = OdometryNoiseConfig(
        sigma_x=0.0, sigma_y=0.0, sigma_yaw_deg=0.0, scale_error=0.02, yaw_bias_deg_per_m=0.1
    )
    pose = drive(config, default_rng(0))
    assert math.hypot(pose.x, pose.y) == pytest.approx(10.2, rel=1e-3)
    assert pose.yaw == pytest.approx(math.radians(1.0), rel=1e-6)


def test_turning_in_place_drifts_with_rotation_weight():
    """Rotation counts as travel: yaw noise also grows while turning on the spot."""
    config = OdometryNoiseConfig(sigma_x=0.0, sigma_y=0.0, sigma_yaw_deg=1.0, rotation_weight=1.0)
    odom = DriftingOdometry(config, default_rng(1), Pose2D())
    for i in range(1, 63):
        odom.update(Pose2D(0.0, 0.0, 0.1 * i))
    assert odom.pose.yaw != pytest.approx(math.remainder(6.2, 2 * math.pi), abs=1e-6)
    assert odom.distance == 0.0


def test_same_seed_same_run():
    """Scenarios are reproducible: the seed fixes the drift; another seed changes it."""
    config = OdometryNoiseConfig(sigma_x=0.05, sigma_y=0.05, sigma_yaw_deg=1.0)
    a = drive(config, make_rngs(5)[1])
    b = drive(config, make_rngs(5)[1])
    c = drive(config, make_rngs(6)[1])
    assert a == b and a != c


def test_restart_starts_a_new_origin():
    """After a restart odom is the base at the current pose again."""
    odom = DriftingOdometry(QUIET, default_rng(0), Pose2D())
    odom.update(Pose2D(3.0, 0.0, 0.0))
    odom.restart(Pose2D(3.0, 0.0, 0.0))
    assert odom.pose == Pose2D() and odom.distance == 0.0
    assert odom.update(Pose2D(4.0, 0.0, 0.0)).x == pytest.approx(1.0)


def test_lio_pose_closes_the_bridge_chain():
    """odom -> camera_init -> body -> base_link (localization's bridge) gives odom -> base."""
    base_imu = make_transform(ImuConfig().xyz)
    odom_base = Pose2D(1.0, 2.0, 2.0).to_matrix()
    lio = lio_transform(odom_base, base_imu)
    chain = compose(base_imu, lio, inverse(base_imu))
    assert np.allclose(chain, odom_base)
    assert np.allclose(lio_transform(np.eye(4), base_imu), np.eye(4))
    assert abs(lio[2, 3]) < 1e-12  # planar motion: the IMU stays at its height


def test_imu_is_the_urdf_imu_link_and_fast_lio_extrinsic():
    """The body IMU of application manual 6.2, as the URDF and FAST-LIO's extrinsic_T have it."""
    xyz = ImuConfig().xyz
    assert xyz == pytest.approx((0.13045, -0.001, 0.0149))
    urdf = (REPO / 'x30_description' / 'urdf' / 'x30.urdf.xacro').read_text()
    joint = re.search(r'<joint name="imu_joint".*?<origin xyz="([^"]+)"', urdf, re.S)
    assert tuple(float(v) for v in joint.group(1).split()) == pytest.approx(xyz)
    fast_lio = yaml.safe_load((REPO / 'x30_odometry' / 'config' / 'fast_lio.yaml').read_text())
    mapping = fast_lio.get('mapping', fast_lio)
    assert tuple(mapping['extrinsic_T']) == pytest.approx(tuple(-v for v in xyz))
