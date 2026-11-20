"""x30_health's findings as NETRA's health: level, issues, topic rates, the computer."""

from __future__ import annotations

from x30_fleet.core import health


def test_findings_drop_the_diagnostics_prefix():
    found = health.findings([('health: system:cpu', {'value': '12.5'}), ('plain', {'a': '1'})])
    assert found == {'system:cpu': {'value': '12.5'}, 'plain': {'a': '1'}}


def test_summary_in_netra_names():
    found = health.findings([
        ('health: system:cpu', {'value': '12.46'}),
        ('health: system:memory', {'value': '40', 'used_gb': '6.43', 'total_gb': '16'}),
        ('health: system:temperature', {'value': 'n/a'}),
        ('health: sensor:/livox/lidar', {'hz': '9.96', 'min_hz': '8'}),
        ('health: sensor:/Odometry', {'hz': 'nan'}),
    ])  # fmt: skip
    out = health.summary(1, ['lidar slow'], 0.5, found)
    assert (out['ok'], out['level'], out['issues'], out['speed_scale']) == (
        False, 'warn', ['lidar slow'], 0.5)  # fmt: skip
    assert out['topics'] == {'/livox/lidar': 10.0, '/Odometry': 0.0}
    assert (out['cpu_percent'], out['memory_percent'], out['memory_used_gb']) == (12.5, 40.0, 6.4)
    assert out['memory_total_gb'] == 16.0
    assert out['cpu_temp_c'] is None and out['gpu_percent'] is None


def test_an_unknown_level_is_an_error_and_a_bad_scale_zero():
    out = health.summary(7, [], float('nan'), {})
    assert (out['ok'], out['level'], out['speed_scale']) == (False, 'error', 0.0)
