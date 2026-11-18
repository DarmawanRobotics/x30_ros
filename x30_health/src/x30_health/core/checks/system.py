"""The computer: whitelisted thermal zones, memory, disk, load, CPU and the Jetson GPU."""

from __future__ import annotations

from x30_health.core.checks.base import Check, Issue, Level, fmt, unreadable
from x30_health.core.config import SystemConfig
from x30_health.core.sysinfo import (
    CpuUsage,
    SystemPaths,
    disk_free_gb,
    gpu_load_path,
    gpu_percent,
    load_per_cpu,
    memory_kb,
    thermal_zones,
)


def _limit(name: str, value: float, limit: float, text: str, above: bool = True) -> Issue:
    """Return OK or WARN for a reading against a limit (limit 0: off, always OK)."""
    bad = limit > 0.0 and (value > limit if above else value < limit)
    level = Level.WARN if bad else Level.OK
    return Issue(level, name, text, {'value': value, 'limit': limit})


class SystemCheck(Check):
    """One finding per reading; an unreadable reading with a limit set is a warning (#17)."""

    def __init__(self, config: SystemConfig, paths: SystemPaths | None = None, load_reader=None):
        self.config = config
        self.paths = paths or SystemPaths()
        self.cpu = CpuUsage(self.paths.stat)
        self.gpu_path = gpu_load_path(self.paths.gpu_patterns)
        self.load_reader = load_reader or load_per_cpu

    def configure(self, config: SystemConfig):
        """Apply live changes."""
        self.config = config

    def run(self, now: float) -> list[Issue]:
        """Return the findings of every enabled reading."""
        out = [
            self._temperature(),
            self._memory(),
            self._disk(),
            self._load(),
            self._cpu(),
            self._gpu(),
        ]
        return [issue for issue in out if issue is not None]

    def _temperature(self) -> Issue | None:
        limit = self.config.max_temp_c
        if limit <= 0.0:
            return None
        zones = thermal_zones(self.paths.thermal, self.config.thermal_zones)
        temps = {f'temp_{z.type}': z.temp_c for z in zones if z.temp_c is not None}
        if not temps:
            names = ', '.join(z.type for z in zones) or 'no whitelisted zone'
            return unreadable('system:temperature', f'temperature ({names})')
        hottest = max(temps, key=temps.get)
        issue = _limit(
            'system:temperature',
            temps[hottest],
            limit,
            f'{hottest[5:]} {fmt(temps[hottest])} degC (max {fmt(limit)})',
        )
        return Issue(issue.level, issue.name, issue.message, {**issue.values, **temps})

    def _memory(self) -> Issue | None:
        limit = self.config.max_memory_percent
        if limit <= 0.0:
            return None
        memory = memory_kb(self.paths.meminfo)
        if memory is None:
            return unreadable('system:memory', 'memory')
        percent = 100.0 * memory[0] / memory[1]
        issue = _limit('system:memory', percent, limit, f'memory {fmt(percent)} % used')
        gb = {'used_gb': memory[0] / 1048576.0, 'total_gb': memory[1] / 1048576.0}
        return Issue(issue.level, issue.name, issue.message, {**issue.values, **gb})

    def _disk(self) -> Issue | None:
        limit = self.config.min_disk_free_gb
        if limit <= 0.0:
            return None
        free = disk_free_gb(self.config.disk_path)
        if free is None:
            return unreadable('system:disk', f'disk {self.config.disk_path}')
        text = f'{fmt(free)} GB free at {self.config.disk_path}'
        return _limit('system:disk', free, limit, text, above=False)

    def _load(self) -> Issue | None:
        limit = self.config.max_load_per_cpu
        if limit <= 0.0:
            return None
        load = self.load_reader()
        if load is None:
            return unreadable('system:load', 'load average')
        return _limit('system:load', load, limit, f'load {fmt(load, 2)} per CPU')

    def _cpu(self) -> Issue | None:
        limit = self.config.max_cpu_percent
        percent = self.cpu.percent()
        if not self.cpu.readable:
            return unreadable('system:cpu', 'CPU usage') if limit > 0.0 else None
        if percent is None:
            return Issue(Level.OK, 'system:cpu', 'CPU usage: measuring')
        return _limit('system:cpu', percent, limit, f'CPU {fmt(percent)} % busy')

    def _gpu(self) -> Issue | None:
        if self.gpu_path is None:
            return None
        percent = gpu_percent(self.gpu_path)
        if percent is None:
            return unreadable('system:gpu', 'GPU load')
        limit = self.config.max_gpu_percent
        return _limit('system:gpu', percent, limit, f'GPU {fmt(percent)} % busy')
