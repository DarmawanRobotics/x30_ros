"""Reports within the disk: whole days, the oldest first, never today's, never a link."""

from __future__ import annotations

from datetime import date
import os

from x30_inspection.core.retention import prune, report_days

TODAY = date(2026, 11, 20)


def reports(tmp_path, *days: str) -> str:
    """Return a reports directory with a report in each day."""
    for day in days:
        (tmp_path / day / '101500_trafo').mkdir(parents=True)
        (tmp_path / day / '101500_trafo' / 'report.json').write_text('{}')
    return str(tmp_path)


def test_days_older_than_keep_days_go_and_nothing_else(tmp_path):
    root = reports(tmp_path, '20261101', '20261110', '20261119', '20261120')
    (tmp_path / 'notes').mkdir()  # not a day: never touched
    assert prune(root, TODAY, keep_days=10, min_free_gb=0.0) == (['20261101'], '')
    assert report_days(root) == ['20261110', '20261119', '20261120']
    assert (tmp_path / 'notes').is_dir()


def test_the_oldest_days_go_while_the_disk_is_short_never_today(tmp_path):
    root = reports(tmp_path, '20261118', '20261119', '20261120')

    def free(_path) -> float:  # every day removed frees 1 GB
        gone = sum(not (tmp_path / day).exists() for day in ('20261118', '20261119'))
        return 3.0 + gone

    assert prune(root, TODAY, 0, 6.0, free=free) == (['20261118', '20261119'], '')
    assert report_days(root) == ['20261120']  # today's stays, though the disk is still short


def test_enough_room_keeps_every_day_and_a_link_is_never_followed(tmp_path):
    root = reports(tmp_path, '20261001')
    outside = tmp_path.parent / 'elsewhere'
    outside.mkdir(exist_ok=True)
    os.symlink(outside, tmp_path / '20260901')
    assert prune(root, TODAY, 0, 6.0, free=lambda path: 50.0) == ([], '')
    assert prune(root, TODAY, 0, 6.0, free=lambda path: 1.0) == (['20261001'], '')
    assert outside.is_dir() and (tmp_path / '20260901').is_symlink()


def test_no_reports_directory_yet(tmp_path):
    assert prune(str(tmp_path / 'none'), TODAY, 30, 6.0, free=lambda path: 1.0) == ([], '')
