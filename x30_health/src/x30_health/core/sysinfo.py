"""Readers of the computer's state (/sys, /proc); every reader returns None when unreadable."""

from __future__ import annotations

from dataclasses import dataclass
from fnmatch import fnmatchcase
import glob
import os
import shutil

# Readings outside this range are sensor faults, not temperatures.
VALID_TEMP_C = (-40.0, 150.0)

# Jetson GPU load (0..1000) in sysfs: Orin on JetPack 6, Orin on JetPack 5, Nano / Xavier.
GPU_LOAD_PATTERNS = (
    '/sys/devices/platform/bus@0/*.gpu/load',
    '/sys/devices/platform/gpu.0/load',
    '/sys/devices/gpu.0/load',
    '/sys/devices/platform/*.gpu/load',
    '/sys/devices/*.ga10b/load',
    '/sys/devices/*.gv11b/load',
)


@dataclass(frozen=True)
class SystemPaths:
    """Where the readers look (tests point them at fake files)."""

    thermal: str = '/sys/class/thermal'
    meminfo: str = '/proc/meminfo'
    stat: str = '/proc/stat'
    gpu_patterns: tuple[str, ...] = GPU_LOAD_PATTERNS


@dataclass(frozen=True)
class Zone:
    """One thermal zone: its type and temperature (None: unreadable or implausible)."""

    type: str
    temp_c: float | None


def _read(path: str) -> str | None:
    try:
        with open(path, encoding='utf-8') as f:
            return f.read().strip()
    except (OSError, UnicodeDecodeError):
        return None


def thermal_zones(root: str, whitelist) -> list[Zone]:
    """Return the zones whose type matches a whitelist pattern (case-insensitive)."""
    patterns = [p.lower() for p in whitelist]
    zones = []
    for zone in sorted(glob.glob(os.path.join(root, 'thermal_zone*'))):
        kind = _read(os.path.join(zone, 'type'))
        if kind is None or not any(fnmatchcase(kind.lower(), p) for p in patterns):
            continue
        zones.append(Zone(kind, _temperature(_read(os.path.join(zone, 'temp')))))
    return zones


def _temperature(text: str | None) -> float | None:
    try:
        value = int(text) / 1000.0
    except (TypeError, ValueError):
        return None
    low, high = VALID_TEMP_C
    return value if low < value < high else None


def memory_kb(path: str) -> tuple[int, int] | None:
    """Return (used, total) memory in kB (used = total - available), or None."""
    text = _read(path)
    if text is None:
        return None
    try:
        info = {
            line.split(':')[0]: int(line.split()[1]) for line in text.splitlines() if ':' in line
        }
        used, total = info['MemTotal'] - info['MemAvailable'], info['MemTotal']
    except (KeyError, ValueError, IndexError):
        return None
    return (used, total) if total > 0 else None


def disk_free_gb(path: str) -> float | None:
    """Return the free space in GB of the file system at path, or None."""
    try:
        return shutil.disk_usage(os.path.expandvars(os.path.expanduser(path))).free / 1e9
    except OSError:
        return None


def load_per_cpu() -> float | None:
    """Return the 1 min load average divided by the CPU count, or None."""
    try:
        return os.getloadavg()[0] / max(os.cpu_count() or 1, 1)
    except OSError:
        return None


class CpuUsage:
    """Busy share of all cores between two readings of /proc/stat."""

    def __init__(self, path: str):
        self.path = path
        self._last: tuple[int, int] | None = None
        self.readable = True

    def percent(self) -> float | None:
        """Return the busy share in percent since the last call (None on the first one)."""
        text = _read(self.path)
        try:
            fields = [int(x) for x in (text or '').splitlines()[0].split()[1:9]]
        except (ValueError, IndexError):
            fields = []
        self.readable = len(fields) >= 4
        if not self.readable:
            return None
        idle, total = fields[3] + (fields[4] if len(fields) > 4 else 0), sum(fields)
        last, self._last = self._last, (idle, total)
        if last is None or total <= last[1]:
            return None
        return 100.0 * (1.0 - (idle - last[0]) / (total - last[1]))


def gpu_load_path(patterns) -> str | None:
    """Return the readable GPU load file of a Jetson, or None (no Jetson GPU)."""
    for pattern in patterns:
        for path in sorted(glob.glob(pattern)):
            if os.access(path, os.R_OK):
                return path
    return None


def gpu_percent(path: str) -> float | None:
    """Return the GPU load in percent from a Jetson load file (per mille), or None."""
    try:
        return min(int(_read(path) or '') / 10.0, 100.0)
    except ValueError:
        return None
