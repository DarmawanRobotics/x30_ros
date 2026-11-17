"""Route recording: localized only, incremental atomic writes, site safety, ~ expansion."""

from __future__ import annotations

from datetime import datetime
import math
import os

import pytest

from x30_mapping.core.config import RecorderConfig
from x30_mapping.core.routes import RouteRecorder, route_path
from x30_utils.io import read_route
from x30_utils.math import Pose2D

WHEN = datetime(2026, 10, 24, 14, 3, 5)


def recorder(tmp_path, **kw) -> RouteRecorder:
    return RouteRecorder(RecorderConfig(sites_dir=str(tmp_path), flush_sec=1.0, **kw))


def test_refuses_when_not_localized(tmp_path):
    """A TF that exists is not a localized robot: no recording, no samples, no stop points."""
    rec = recorder(tmp_path)
    ok, message = rec.start('lobby', localized=False, when=WHEN, now=0.0)
    assert not ok and 'not localized' in message and not rec.active
    assert rec.start('lobby', True, WHEN, 0.0)[0]
    assert not rec.sample(Pose2D(1.0, 0.0), localized=False)
    assert not rec.mark_stop(Pose2D(1.0, 0.0), False, 5.0, 1.0)[0]
    assert rec.recording.points == []


def test_route_goes_to_the_site_and_is_written_incrementally(tmp_path):
    rec = recorder(tmp_path)
    ok, _ = rec.start('lobby', True, WHEN, 0.0)
    path = rec.recording.path
    assert ok and path == str(tmp_path / 'lobby' / 'routes' / '2026-10-24_14-03-05.csv')
    assert rec.sample(Pose2D(0.0, 0.0, 0.0), True)
    assert not rec.sample(Pose2D(0.1, 0.0, 0.0), True)  # closer than min_spacing_m
    assert rec.sample(Pose2D(0.5, 0.0, 0.5), True)
    assert not rec.flush(0.5)  # not yet flush_sec
    assert rec.flush(1.5) and len(read_route(path)) == 2
    ok, message = rec.mark_stop(Pose2D(0.6, 0.1, 1.0), True, 4.0, 1.6)
    assert ok and 'stop point 1' in message
    points = read_route(path)  # written right away
    assert points[-1].is_stop and points[-1].yaw == pytest.approx(1.0, abs=1e-3)
    assert not rec.mark_stop(Pose2D(0.6, 0.1), True, 0.0, 1.7)[0]
    ok, message, saved = rec.stop(2.0)
    assert ok and saved == path and '3 waypoints, 1 stop points' in message
    assert not rec.active and not [n for n in os.listdir(os.path.dirname(path)) if n[0] == '.']


def test_name_collisions_get_a_suffix(tmp_path):
    (tmp_path / '2026-10-24_14-03-05.csv').write_text('x')
    assert route_path(str(tmp_path), WHEN).endswith('2026-10-24_14-03-05-1.csv')


def test_output_path_is_expanded_and_empty_refuses(tmp_path, monkeypatch):
    """#11: ~ in output_path is expanded; without a site and output_path nothing is recorded."""
    monkeypatch.setenv('HOME', str(tmp_path))
    assert not recorder(tmp_path).start('', True, WHEN, 0.0)[0]
    rec = recorder(tmp_path, output_path='~/routes')
    assert rec.start('', True, WHEN, 0.0)[0]
    assert rec.recording.path.startswith(str(tmp_path / 'routes'))
    assert '~' not in rec.recording.path


def test_site_change_ends_the_recording(tmp_path):
    """A route is never continued into another site."""
    rec = recorder(tmp_path)
    rec.start('lobby', True, WHEN, 0.0)
    rec.sample(Pose2D(0.0, 0.0), True)
    assert rec.on_site('lobby', 1.0) == ''
    message = rec.on_site('hall', 1.0)
    assert 'route of lobby saved' in message and not rec.active
    assert len(read_route(str(tmp_path / 'lobby' / 'routes' / '2026-10-24_14-03-05.csv'))) == 1


def test_archived_routes_dir_is_not_recreated(tmp_path):
    """A reset that archived routes/ ends the recording instead of writing into a new frame."""
    rec = recorder(tmp_path)
    rec.start('lobby', True, WHEN, 0.0)
    rec.sample(Pose2D(0.0, 0.0), True)
    os.rename(tmp_path / 'lobby' / 'routes', tmp_path / 'lobby' / 'backup')
    with pytest.raises(OSError):
        rec.flush(5.0)
    ok, message, _ = rec.stop(6.0)
    assert not ok and 'disappeared' in message
    assert not (tmp_path / 'lobby' / 'routes').exists()


def test_empty_recording_is_a_failure(tmp_path):
    rec = recorder(tmp_path)
    rec.start('lobby', True, WHEN, 0.0)
    ok, message, path = rec.stop(1.0)
    assert not ok and path == '' and 'nothing recorded' in message
    assert math.isfinite(rec.config.min_spacing_m)
