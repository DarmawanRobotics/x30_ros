"""Tag files and the record session: review #2, #3."""

from __future__ import annotations

import json
import math

import numpy as np
import pytest

from x30_localization.core.tags import (
    RecordLimits,
    RecordSession,
    average,
    load_tags,
    merge_tags,
    recordable,
)
from x30_utils.math import default_rng, make_transform, quat_from_rpy


def tag(x, y, z=0.5, yaw=0.0):
    return make_transform((x, y, z), quat_from_rpy(-math.pi / 2, 0.0, yaw))


def test_load_is_tolerant(tmp_path):
    assert load_tags('') == ({}, [])
    assert load_tags(str(tmp_path / 'missing.json')) == ({}, [])
    broken = tmp_path / 'broken.json'
    broken.write_text('{"a": {"x": 1')
    tags, problems = load_tags(str(broken))
    assert tags == {} and problems
    partial = tmp_path / 'partial.json'
    pose = {**dict.fromkeys('xyz', 0), 'qx': 0, 'qy': 0, 'qz': 0, 'qw': 1}
    partial.write_text(json.dumps({'a': {'x': 1}, 'b': pose}))
    tags, problems = load_tags(str(partial))
    assert list(tags) == ['b'] and len(problems) == 1


def test_merge_keeps_other_tags_and_backs_up(tmp_path):
    path = tmp_path / 'site' / 'tags.json'
    merged, error = merge_tags(str(path), {'a': tag(1.0, 2.0)})
    assert error == '' and set(merged) == {'a'}
    merged, error = merge_tags(str(path), {'b': tag(3.0, 4.0, yaw=1.0)})
    assert error == '' and set(merged) == {'a', 'b'}
    assert (tmp_path / 'site' / 'tags.json.bak').exists()
    tags, problems = load_tags(str(path))
    assert problems == [] and np.allclose(tags['b'], tag(3.0, 4.0, yaw=1.0), atol=1e-6)


def test_merge_refuses_to_overwrite_a_corrupt_file(tmp_path):
    """Review #3: a corrupt file is an error, never a crash and never silently replaced."""
    path = tmp_path / 'tags.json'
    path.write_text('not json')
    merged, error = merge_tags(str(path), {'a': tag(1.0, 2.0)})
    assert merged == {} and 'untouched' in error
    assert path.read_text() == 'not json'


def test_merge_reports_unwritable_paths(tmp_path):
    blocker = tmp_path / 'file'
    blocker.write_text('')
    merged, error = merge_tags(str(blocker / 'tags.json'), {'a': tag(0.0, 0.0)})
    assert merged == {} and 'cannot write' in error


def test_recordable_refuses_when_not_localized_or_circular():
    """Review #2: never record anchors in a wrong frame or from themselves."""
    frames = ('base_map', 'dock')
    allowed, reason = recordable(False, set(), frames)
    assert allowed == () and 'not localized' in reason
    allowed, reason = recordable(True, {'tag:base_map', 'tag:dock'}, frames)
    assert allowed == () and 'circular' in reason
    allowed, reason = recordable(True, {'origin', 'tag:base_map'}, frames)
    assert allowed == ('dock',) and reason == ''
    assert recordable(True, {'pose_estimate'}, frames) == (frames, '')


def test_session_averages_counts_each_stamp_once_and_rejects_noise():
    session = RecordSession(('a', 'b', 'c'), end_time=2.0, limits=RecordLimits(min_samples=5))
    rng = default_rng(0)
    for i in range(20):
        dx, dy = rng.normal(0.0, 0.005, 2)
        assert session.add('a', tag(1.0 + dx, 2.0 + dy, yaw=0.3), stamp=0.1 * i)
        assert not session.add('a', tag(9.0, 9.0), stamp=0.1 * i)  # same detection again
        session.add('b', tag(rng.normal(0.0, 0.5), 0.0), stamp=0.1 * i)  # far too noisy
    session.add('c', tag(0.0, 0.0), stamp=0.0)
    assert not session.add('unknown', tag(0.0, 0.0), 0.0)
    assert not session.done(1.9) and session.done(2.0)
    good, lines = session.result()
    assert set(good) == {'a'}
    assert good['a'][:3, 3] == pytest.approx([1.0, 2.0, 0.5], abs=0.01)
    assert any('too noisy' in line for line in lines)
    assert any('seen 1 times' in line for line in lines)


def test_average_handles_quaternion_sign_and_yaw_wrap():
    near_pi = [tag(0.0, 0.0, yaw=math.pi - 0.01), tag(0.0, 0.0, yaw=-math.pi + 0.01)]
    avg = average(near_pi)
    assert abs(math.atan2(avg.pose[1, 0], avg.pose[0, 0])) == pytest.approx(math.pi, abs=1e-6)
    assert avg.std_yaw == pytest.approx(0.01, abs=1e-3)
    assert np.allclose(avg.pose[:3, 2], tag(0.0, 0.0, yaw=math.pi)[:3, 2], atol=1e-3)
