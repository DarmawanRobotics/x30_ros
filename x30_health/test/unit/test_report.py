"""Aggregation: worst level, issue lines (errors first) and the speed scale (H1)."""

from __future__ import annotations

from x30_health.core.checks.base import Issue, Level
from x30_health.core.config import DegradationConfig
from x30_health.core.report import aggregate

OK = Issue(Level.OK, 'system:memory', 'memory 40.0 % used')
LIDAR_SLOW = Issue(Level.WARN, 'sensor:/livox/lidar', '/livox/lidar 2.0 Hz (min 5.0)')
TF_LATE = Issue(Level.WARN, 'tf:camera_init->body', 'camera_init->body 1.8 s old')
DISK = Issue(Level.WARN, 'system:disk', '1.0 GB free at ~')
LOST = Issue(Level.ERROR, 'state:odometry', 'odometry lost')


def test_all_ok():
    report = aggregate([OK], DegradationConfig())
    assert report.level is Level.OK and report.speed_scale == 1.0 and report.lines() == []


def test_nothing_to_report():
    report = aggregate([], DegradationConfig())
    assert report.level is Level.OK and report.speed_scale == 1.0


def test_lidar_rate_low_or_tf_late_halves_the_speed():
    assert aggregate([OK, LIDAR_SLOW], DegradationConfig()).speed_scale == 0.5
    report = aggregate([TF_LATE], DegradationConfig())
    assert report.level is Level.WARN and report.speed_scale == 0.5


def test_other_warnings_keep_the_speed():
    report = aggregate([DISK], DegradationConfig())
    assert report.level is Level.WARN and report.speed_scale == 1.0
    assert aggregate([DISK], DegradationConfig(warn_scale=0.8)).speed_scale == 0.8


def test_error_stops_and_comes_first():
    report = aggregate([DISK, LIDAR_SLOW, LOST, OK], DegradationConfig())
    assert report.level is Level.ERROR and report.speed_scale == 0.0
    assert report.lines()[0] == 'odometry lost'
    assert len(report.findings) == 4 and len(report.issues) == 3


def test_lowest_matching_rule_wins_and_rules_never_raise_an_error():
    rules = ('sensor:*=0.7', 'sensor:/livox/*=0.3', 'state:*=1.0')
    assert aggregate([LIDAR_SLOW], DegradationConfig(rules=rules)).speed_scale == 0.3
    assert aggregate([LOST], DegradationConfig(rules=rules)).speed_scale == 0.0
