"""The computer, from fake /sys and /proc files: whitelist, unreadable values (#17), limits."""

from __future__ import annotations

from pathlib import Path

import pytest

from x30_health.core.checks.base import Level
from x30_health.core.checks.system import SystemCheck
from x30_health.core.config import SystemConfig
from x30_health.core.sysinfo import (
    CpuUsage,
    SystemPaths,
    disk_free_gb,
    gpu_load_path,
    gpu_percent,
    memory_kb,
    thermal_zones,
)

MEMINFO = 'MemTotal: 16000000 kB\nMemFree: 1000 kB\nMemAvailable: 12000000 kB\n'


def zone(root: Path, index: int, kind: str, temp: str | None):
    path = root / f'thermal_zone{index}'
    path.mkdir(parents=True)
    (path / 'type').write_text(f'{kind}\n')
    if temp is not None:
        (path / 'temp').write_text(f'{temp}\n')


@pytest.fixture
def fake(tmp_path):
    thermal = tmp_path / 'thermal'
    zone(thermal, 0, 'CPU-therm', '52000')
    zone(thermal, 1, 'GPU-therm', '48500')
    zone(thermal, 2, 'PMIC-Die', '100000')  # Jetson: always 100 degC, not whitelisted
    zone(thermal, 3, 'Tj-therm', '200000')  # implausible reading
    (tmp_path / 'meminfo').write_text(MEMINFO)
    (tmp_path / 'stat').write_text('cpu  100 0 100 700 100 0 0 0 0 0\n')
    return tmp_path, SystemPaths(
        thermal=str(thermal),
        meminfo=str(tmp_path / 'meminfo'),
        stat=str(tmp_path / 'stat'),
        gpu_patterns=(str(tmp_path / 'gpu*' / 'load'),),
    )


def by_name(issues):
    return {i.name: i for i in issues}


def test_whitelisted_zones_only(fake):
    _, paths = fake
    zones = thermal_zones(paths.thermal, ('cpu*', 'gpu*', 'tj*'))
    assert [z.type for z in zones] == ['CPU-therm', 'GPU-therm', 'Tj-therm']
    assert zones[2].temp_c is None  # 200 degC is a sensor fault
    issue = by_name(SystemCheck(SystemConfig(), paths).run(0.0))['system:temperature']
    assert issue.level is Level.OK and issue.values['value'] == 52.0
    assert 'temp_PMIC-Die' not in issue.values and 'temp_GPU-therm' in issue.values


def test_hot_zone_warns(fake):
    _, paths = fake
    issue = by_name(SystemCheck(SystemConfig(max_temp_c=50.0), paths).run(0.0))
    assert issue['system:temperature'].level is Level.WARN
    assert 'CPU-therm 52.0 degC' in issue['system:temperature'].message


def test_unreadable_values_warn(tmp_path):
    """#17: no temperature, memory or disk reading must never report OK."""
    zone(tmp_path / 'thermal', 0, 'CPU-therm', None)
    paths = SystemPaths(
        thermal=str(tmp_path / 'thermal'),
        meminfo=str(tmp_path / 'missing'),
        stat=str(tmp_path / 'missing'),
        gpu_patterns=(),
    )
    config = SystemConfig(disk_path=str(tmp_path / 'no' / 'such' / 'dir'), max_cpu_percent=90.0)
    issues = by_name(SystemCheck(config, paths, load_reader=lambda: None).run(0.0))
    for name in ('temperature', 'memory', 'disk', 'load', 'cpu'):
        issue = issues[f'system:{name}']
        assert issue.level is Level.WARN and 'unreadable' in issue.message, name
    assert '(CPU-therm)' in issues['system:temperature'].message
    assert 'system:gpu' not in issues  # no Jetson: no GPU finding


def test_no_whitelisted_zone_warns(tmp_path):
    zone(tmp_path, 0, 'acpitz', '40000')
    paths = SystemPaths(thermal=str(tmp_path), gpu_patterns=())
    issue = by_name(SystemCheck(SystemConfig(), paths).run(0.0))['system:temperature']
    assert issue.level is Level.WARN and 'no whitelisted zone' in issue.message


def test_limits_of_zero_are_off(tmp_path):
    paths = SystemPaths(thermal=str(tmp_path), meminfo=str(tmp_path / 'x'), gpu_patterns=())
    config = SystemConfig(
        max_temp_c=0.0, max_memory_percent=0.0, min_disk_free_gb=0.0, max_load_per_cpu=0.0
    )
    issues = by_name(SystemCheck(config, paths).run(0.0))
    assert set(issues) <= {'system:cpu'}


def test_memory_disk_load(fake):
    _, paths = fake
    config = SystemConfig(max_memory_percent=20.0, min_disk_free_gb=1e5, max_load_per_cpu=1.0)
    issues = by_name(SystemCheck(config, paths, load_reader=lambda: 1.5).run(0.0))
    assert issues['system:memory'].level is Level.WARN
    assert issues['system:memory'].values['value'] == 25.0
    assert issues['system:disk'].level is Level.WARN  # nobody has 100 TB free
    assert issues['system:load'].level is Level.WARN


def test_cpu_between_two_readings(fake):
    tmp, paths = fake
    system = SystemCheck(SystemConfig(max_cpu_percent=20.0), paths)
    assert by_name(system.run(0.0))['system:cpu'].message == 'CPU usage: measuring'
    (tmp / 'stat').write_text('cpu  200 0 200 1300 100 0 0 0 0 0\n')
    issue = by_name(system.run(1.0))['system:cpu']
    assert issue.values['value'] == 25.0 and issue.level is Level.WARN
    assert CpuUsage(str(tmp / 'missing')).percent() is None


def test_jetson_gpu(fake):
    tmp, paths = fake
    load = tmp / 'gpu.0' / 'load'
    load.parent.mkdir()
    load.write_text('412\n')
    assert gpu_load_path(paths.gpu_patterns) == str(load) and gpu_percent(str(load)) == 41.2
    system = SystemCheck(SystemConfig(max_gpu_percent=40.0), paths)
    assert by_name(system.run(0.0))['system:gpu'].level is Level.WARN
    load.write_text('garbage')
    assert 'unreadable' in by_name(system.run(1.0))['system:gpu'].message


def test_readers(fake, tmp_path):
    tmp, paths = fake
    assert memory_kb(paths.meminfo) == (4000000, 16000000)
    (tmp / 'zero').write_text('MemTotal: 0 kB\nMemAvailable: 0 kB\n')
    assert memory_kb(str(tmp / 'zero')) is None  # no division by zero
    assert disk_free_gb(str(tmp_path)) > 0.0
    assert disk_free_gb(str(tmp_path / 'missing')) is None
