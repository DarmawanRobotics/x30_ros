"""In-process node tests: leveled bridge, tags, record_tags while TF flows, path guard."""

from __future__ import annotations

import json
import math
import threading
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from harness import BASE_HEIGHT, START_TILT, TAG_IN_MAP, Harness
import numpy as np
import pytest
import rospy
from std_srvs.srv import Trigger
from tf2_msgs.msg import TFMessage

from x30_msgs.msg import LocalizationStatus
from x30_msgs.srv import LoadFile
from x30_utils.io import write_tags
from x30_utils.math import Pose2D, quat_from_rpy, tilt_of
from x30_utils.testing.harness import Collector, connected, wait_for

NONE, LOCALIZED = LocalizationStatus.NONE, LocalizationStatus.LOCALIZED


@pytest.fixture
def loc(ros, tmp_path):
    harness = Harness(
        {
            'sites_dir': str(tmp_path),
            'assume_start_origin': False,  # NONE until a tag or a pose estimate
            'tag.record_duration_sec': 1.0,
            'tag.record_min_samples': 3,
            'tag.min_interval_sec': 0.0,
            'leveling.window_sec': 0.3,
            'tag.frames': ['base_map'],
        }
    )
    yield harness
    harness.close()


def test_bridge_levels_odom_and_reset_clears(loc):
    """Odom is gravity-aligned; base_link at start sits at the odom origin."""
    assert wait_for(lambda: loc.state() == NONE, 5.0)
    assert loc.bridge_up()
    assert loc.node.bridge.leveled
    assert wait_for(lambda: loc.lookup('odom', 'base_link') is not None, 5.0)
    odom_base = loc.lookup('odom', 'base_link')
    assert np.allclose(odom_base[:3, 3], 0.0, atol=1e-3)
    assert tilt_of(odom_base) == pytest.approx(START_TILT, abs=0.01)
    assert loc.call(Trigger, 'localization/assume_origin').success
    assert wait_for(lambda: loc.state() == LOCALIZED, 3.0)
    assert loc.status.last.source == 'origin'
    result = loc.call(Trigger, 'localization/reset')
    assert result.success and 'origin' in result.message
    assert wait_for(lambda: loc.state() == NONE, 3.0)


def test_record_tags_keeps_tf_flowing_and_relocalizes(loc, tmp_path):
    """record_tags refuses unless localized; while it records, map->odom keeps coming."""
    path = tmp_path / 'site' / 'tags.json'
    created = loc.call(LoadFile, 'localization/load_tags', path='site/tags.json', create=True)
    assert created.success, created.message
    assert json.loads(path.read_text()) == {}
    assert loc.bridge_up()
    refused = loc.call(Trigger, 'localization/record_tags')
    assert not refused.success and 'not localized' in refused.message
    with loc.robot.lock:
        loc.robot.robot = Pose2D(0.3, -0.2, 0.25)
    assert wait_for(lambda: abs(loc.node.odom_base().yaw - 0.25) < 1e-3, 3.0)  # odometry caught up
    pose = PoseWithCovarianceStamped()
    pose.header.frame_id = 'map'
    pose.pose.pose.position.x, pose.pose.pose.position.y = 0.3, -0.2
    pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = math.sin(0.125), math.cos(0.125)
    publisher = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1)
    try:
        assert connected(publisher)
        publisher.publish(pose)
        assert wait_for(lambda: loc.state() == LOCALIZED, 3.0)
    finally:
        publisher.unregister()
    assert loc.status.last.source == 'pose_estimate'

    loc.robot.tag_visible = True
    tf = Collector(TFMessage, '/tf', queue_size=1000)
    answer = []
    call = threading.Thread(
        target=lambda: answer.append(loc.call(Trigger, 'localization/record_tags'))
    )
    started = time.monotonic()
    try:
        assert connected(tf)
        call.start()
        call.join(8.0)
        assert answer, 'record_tags did not answer'
        stamps = [
            t.header.stamp for m in tf.messages for t in m.transforms if t.child_frame_id == 'odom'
        ]
    finally:
        tf.close()
    result = answer[0]
    assert result.success, result.message
    assert time.monotonic() - started >= 0.9
    assert len(stamps) >= 10, 'map->odom stopped while recording'
    saved = json.loads(path.read_text())['base_map']
    # map z is measured from base_link at start (the odom origin), base_height above the floor
    expected = TAG_IN_MAP[:3, 3] - (0.0, 0.0, BASE_HEIGHT)
    assert [saved[k] for k in 'xyz'] == pytest.approx(list(expected), abs=0.01)

    # the odometry restarts elsewhere: the tag in view localizes again (auto_localize)
    loc.robot.tag_visible = False
    time.sleep(0.6)  # the last detection gets older than tag.max_age_sec
    loc.node.on_restart(None)
    assert wait_for(lambda: loc.state() == NONE, 3.0)
    with loc.robot.lock:
        loc.robot.robot = Pose2D(0.5, 0.3, -0.2)
    loc.robot.tag_visible = True
    assert wait_for(lambda: loc.state() == LOCALIZED, 5.0)
    assert loc.status.last.source == 'tag:base_map'
    estimator = loc.node.localizer.estimator
    target = Pose2D(0.5, 0.3, -0.2)
    assert wait_for(lambda: estimator.base_pose().distance(target) < 0.02, 3.0)
    assert estimator.base_pose().yaw == pytest.approx(-0.2, abs=0.01)
    circular = loc.call(Trigger, 'localization/record_tags')
    assert not circular.success and 'circular' in circular.message


def test_load_tags_path_guard_and_corrupt_file(loc, tmp_path):
    """Paths outside sites_dir are refused; a corrupt file never crashes the node."""
    outside = loc.call(LoadFile, 'localization/load_tags', path='../evil/tags.json', create=True)
    assert not outside.success and 'outside' in outside.message
    assert not (tmp_path.parent / 'evil').exists()
    absolute = loc.call(LoadFile, 'localization/load_tags', path='/tmp/x.json', create=False)
    assert not absolute.success
    empty = loc.call(LoadFile, 'localization/load_tags', path=' ', create=False)
    assert not empty.success
    broken = tmp_path / 'broken' / 'tags.json'
    broken.parent.mkdir()
    broken.write_text('{"base_map": ')
    loaded = loc.call(LoadFile, 'localization/load_tags', path=str(broken), create=False)
    assert loaded.success and '0 tag(s)' in loaded.message and 'problem' in loaded.message
    assert loc.bridge_up()
    assert loc.call(Trigger, 'localization/assume_origin').success
    loc.robot.tag_visible = True
    recorded = loc.call(Trigger, 'localization/record_tags')
    assert not recorded.success and 'untouched' in recorded.message
    assert broken.read_text() == '{"base_map": '
    start = loc.call(Trigger, 'localization/start')
    assert not start.success and 'no tags' in start.message


def test_tags_file_with_tilde_is_expanded(ros, tmp_path, monkeypatch):
    """~ in tags_file is the home directory, not a literal '~' directory."""
    monkeypatch.setenv('HOME', str(tmp_path))
    site = tmp_path / 'site'
    site.mkdir()
    pose = dict(zip('xyz', TAG_IN_MAP[:3, 3].tolist()))
    tag = {**pose, 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
    (site / 'tags.json').write_text(json.dumps({'base_map': tag}))
    harness = Harness({'sites_dir': str(tmp_path), 'tags_file': '~/site/tags.json'})
    try:
        assert harness.node.tag_files.path == str(site / 'tags.json')
        assert set(harness.node.localizer.tags) == {'base_map'}
    finally:
        harness.close()


def test_assume_start_origin_only_at_the_first_start(ros, tmp_path):
    """After an odometry restart the old origin is no map frame: stay NONE (safer)."""
    harness = Harness({'sites_dir': str(tmp_path), 'leveling.window_sec': 0.3})  # the default
    try:
        assert wait_for(lambda: harness.state() == LOCALIZED, 10.0)
        assert harness.status.last.source == 'origin'
        harness.node.on_restart(None)
        assert wait_for(lambda: harness.state() == NONE, 3.0)
        assert wait_for(lambda: not harness.node.relevel, 5.0)  # bridge leveled again
        time.sleep(0.3)
        assert harness.state() == NONE
    finally:
        harness.close()


def test_a_tag_is_fused_while_fast_lio_lags_the_camera(ros, tmp_path):
    """Detections always newer than the odometry's TF: each waits for it, none is dropped."""
    tags = tmp_path / 'tags.json'
    at = TAG_IN_MAP[:3, 3] - (0.0, 0.0, BASE_HEIGHT)  # map z from base_link at start
    write_tags(str(tags), {'base_map': (*at, *quat_from_rpy(-math.pi / 2, 0.0, math.pi / 2))})
    harness = Harness(
        {
            'sites_dir': str(tmp_path),
            'tags_file': str(tags),
            'assume_start_origin': False,  # the tag localizes, not the origin
            'leveling.window_sec': 0.3,
            'tag.frames': ['base_map'],
        }
    )
    try:
        harness.robot.lio_lag = 0.15  # longer than a poll (tag/update_hz 10)
        assert harness.bridge_up()
        harness.robot.tag_visible = True
        assert wait_for(lambda: harness.state() == LOCALIZED, 5.0), harness.status.last
        assert harness.status.last.source == 'tag:base_map'
    finally:
        harness.close()
