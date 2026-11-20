"""Localization policy: L1 (gated continuous correction), #1, #2, #3, #14."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from x30_localization.core.config import EstimatorConfig, TagConfig
from x30_localization.core.estimator import MapOdomEstimator, State
from x30_localization.core.localizer import Localizer
from x30_localization.core.sources import PoseEstimateSource
from x30_localization.core.sources.base import Measurement
from x30_utils.math import Pose2D, inverse, make_transform, quat_from_rpy

BASE_CAM = make_transform((0.25, 0.0, 0.1), quat_from_rpy(-math.pi / 2, 0.0, -math.pi / 2))
MAP_TAG = make_transform((2.5, 0.3, 0.4), quat_from_rpy(-math.pi / 2, 0.0, math.pi / 2))
FIX = np.diag([0.02**2, 0.02**2, math.radians(1.0) ** 2])


def localizer(**tag):
    loc = Localizer(MapOdomEstimator(EstimatorConfig()), TagConfig(min_interval_sec=0.0, **tag))
    loc.tags = {'base_map': MAP_TAG}
    return loc


def cam_tag_seen_from(map_base: Pose2D) -> np.ndarray:
    return inverse(map_base.to_matrix() @ BASE_CAM) @ MAP_TAG


def test_tag_in_view_localizes_then_corrects_through_the_gate():
    loc = localizer()
    truth_map_odom = Pose2D(1.0, -1.0, 0.6)
    robot = Pose2D(0.2, 0.1, 0.15)  # the tag 2.3 m ahead, slightly to the left
    odom = truth_map_odom.inverse() @ robot
    loc.track(odom, 1.0)
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(robot), 1.0)
    assert isinstance(m, Measurement), m
    samples = loc.on_tag(m, odom, 1.0)
    assert samples and loc.estimator.state is State.LOCALIZED
    assert loc.estimator.map_odom.to_array() == pytest.approx(truth_map_odom.to_array())
    # a small consistent drift is corrected continuously
    drifted = Pose2D(0.03, 0.0, 0.0) @ robot
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(drifted), 2.0)
    assert loc.on_tag(m, odom, 2.0)
    # a tag that would teleport the robot 3 m is refused (detector error, wrong id)
    jump = Pose2D(3.0, 0.0, 0.0) @ robot
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(jump), 3.0)
    before = loc.estimator.map_odom
    assert loc.on_tag(m, odom, 3.0) == []
    assert loc.estimator.map_odom == before and 'rejected' in loc.message


def test_the_origin_is_known_as_well_as_the_start_spot():
    """Put on its start spot by the dock or by hand: the tags in view correct the spot."""
    near = Pose2D(0.04, -0.03, math.radians(1.5))  # the spot a little off
    off = []
    for xy, yaw_deg in ((0.05, 2.0), (0.01, 0.25)):  # the defaults; as sure as after a fix
        config = EstimatorConfig(origin_sigma_xy=xy, origin_sigma_yaw_deg=yaw_deg)
        loc = Localizer(MapOdomEstimator(config), TagConfig(min_interval_sec=0.0))
        loc.tags = {'base_map': MAP_TAG}
        loc.track(Pose2D(), 0.0)
        loc.assume_origin(0.0)
        status = loc.status(0.0)
        assert status.sigma_xy == pytest.approx(xy)
        assert math.degrees(status.sigma_yaw) == pytest.approx(yaw_deg)
        for i in range(10):
            m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(near), 1.0 + i)
            assert loc.on_tag(m, Pose2D(), 1.0 + i)  # within the gate
        off.append(loc.estimator.base_pose().distance(near))
    assert off[0] < 0.6 * off[1]  # the start spot's own uncertainty: the tags win sooner


def test_explicit_localize_takes_a_far_consistent_tag():
    """auto_correct gates; localization/start (explicit) relocalizes regardless."""
    loc = localizer()
    loc.track(Pose2D(), 0.0)
    loc.assume_origin(0.0)
    far = Pose2D(0.5, -0.8, 0.4)  # consistent, but 0.9 m from where the robot is believed
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(far), 1.0)
    assert isinstance(m, Measurement), m
    assert loc.on_tag(m, Pose2D(), 1.0) == []  # gated
    loc.localize(m, Pose2D(), 1.0)
    assert loc.estimator.base_pose().to_array() == pytest.approx(far.to_array())


def test_correction_samples_step_without_ramp():
    """#14 / T1: the old map->odom is stamped epsilon before the new one."""
    loc = localizer()
    loc.track(Pose2D(), 0.0)
    loc.assume_origin(0.0)
    loc.track(Pose2D(), 0.05)
    m = Measurement(Pose2D(0.02, 0.0, 0.0), FIX, 0.06, 'pose_estimate')
    _, samples = loc.correct(m, Pose2D(), 0.06)
    (old, t_old), (new, t_new) = samples
    assert old == Pose2D() and new != Pose2D()
    assert 0.0 < t_new - t_old <= 1e-3 + 1e-12


def test_record_refused_when_not_localized_and_when_circular():
    loc = localizer()
    loc.track(Pose2D(), 0.0)
    assert 'not localized' in loc.start_record(('base_map',), 0.0)
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(Pose2D()), 0.0)
    loc.on_tag(m, Pose2D(), 0.0)
    assert 'circular' in loc.start_record(('base_map',), 0.0)
    loc.reset(1.0, 'odometry restarted')
    assert 'not localized' in loc.start_record(('base_map',), 1.0)


def test_record_session_writes_the_tag_and_survives_a_corrupt_file(tmp_path):
    """Records the tag pose seen from a yawed robot; a corrupt file gives success=false."""
    loc = localizer(record_duration_sec=1.0, record_min_samples=3)
    loc.tags = {}
    odom = Pose2D(1.0, 2.0, 0.8)
    loc.track(odom, 0.0)
    loc.localize(Measurement(Pose2D(0.2, 0.1, 0.15), FIX, 0.0, 'pose_estimate'), odom, 0.0)
    robot = loc.estimator.base_pose()
    assert loc.start_record(('base_map',), 0.0) == ''
    assert loc.start_record(('base_map',), 0.0) == 'already recording'
    cam_tag = cam_tag_seen_from(robot)
    for i in range(5):
        loc.record_sample('base_map', odom.to_matrix(), BASE_CAM, cam_tag, 0.1 * (i + 1))
    path = tmp_path / 'tags.json'
    path.write_text('{broken')
    ok, message = loc.finish_record(str(path))
    assert not ok and 'untouched' in message and path.read_text() == '{broken'
    assert loc.start_record(('base_map',), 2.0) == ''
    for i in range(5):
        loc.record_sample('base_map', odom.to_matrix(), BASE_CAM, cam_tag, 3.0 + i)
    far = np.array(cam_tag)
    far[:3, 3] *= 3.0
    loc.record_sample('base_map', odom.to_matrix(), BASE_CAM, far, 9.0)  # ignored: too far
    path.unlink()
    ok, message = loc.finish_record(str(path))
    assert ok, message
    saved = json.loads(path.read_text())['base_map']
    assert [saved[k] for k in 'xyz'] == pytest.approx([2.5, 0.3, 0.4], abs=1e-6)
    assert np.allclose(loc.tags['base_map'], MAP_TAG, atol=1e-6)


def test_record_discarded_when_localization_changes_meanwhile(tmp_path):
    loc = localizer(record_min_samples=1)
    loc.track(Pose2D(), 0.0)
    loc.assume_origin(0.0)
    assert loc.start_record(('base_map',), 0.0) == ''
    loc.record_sample('base_map', np.eye(4), BASE_CAM, cam_tag_seen_from(Pose2D()), 0.1)
    loc.reset(0.5, 'reset')
    ok, message = loc.finish_record(str(tmp_path / 't.json'))
    assert not ok and 'changed' in message


def test_no_correction_from_a_tag_being_recorded():
    loc = localizer()
    loc.track(Pose2D(), 0.0)
    loc.assume_origin(0.0)
    assert loc.start_record(('base_map',), 0.0) == ''
    m = loc.measure_tag('base_map', BASE_CAM, cam_tag_seen_from(Pose2D(0.01, 0, 0)), 0.5)
    assert loc.on_tag(m, Pose2D(), 0.5) == []


def test_status_and_uncertain_state_after_a_long_drive():
    """No map corrects the drift: LOCALIZED all along by default, UNCERTAIN past a set sigma."""
    for config, state in (
        (EstimatorConfig(), State.LOCALIZED),
        (EstimatorConfig(max_sigma_xy=0.5), State.UNCERTAIN),
    ):
        loc = Localizer(MapOdomEstimator(config), TagConfig(min_interval_sec=0.0))
        status = loc.status(0.0)
        assert status.state is State.NONE and math.isinf(status.sigma_xy)
        loc.track(Pose2D(), 0.0)
        loc.assume_origin(0.0)
        assert loc.status(0.0).source == 'origin'
        for i in range(1, 400):
            loc.track(Pose2D(float(i), 0.0, 0.0), float(i))
        status = loc.status(400.0)
        assert status.state is state and status.since_correction == pytest.approx(400.0)
        assert status.sigma_xy > 0.5  # the drift, reported


def test_unknown_tag_and_missing_file():
    loc = localizer()
    assert 'not in the tags file' in loc.measure_tag('dock', BASE_CAM, np.eye(4), 0.0)
    loc.track(Pose2D(), 0.0)
    loc.assume_origin(0.0)
    loc.start_record(('dock',), 0.0)
    ok, message = loc.finish_record('')
    assert not ok and 'no tag recorded' in message


def test_pose_estimate_is_trusted_over_the_rviz_ui_covariance():
    source = PoseEstimateSource()
    rviz = np.diag([0.25, 0.25, 0.0685])
    m = source.measure(Pose2D(1.0, 2.0, 0.5), rviz, 3.0)
    assert np.sqrt(m.covariance[0, 0]) == pytest.approx(0.05)
    tight = np.diag([1e-4, 1e-4, 1e-5])
    assert np.allclose(source.measure(Pose2D(), tight).covariance, tight)
    assert isinstance(source.measure(Pose2D(math.nan, 0.0, 0.0)), str)
