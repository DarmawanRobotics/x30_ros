"""Optional NTP offset of this computer, parsed from `chronyc tracking` (off by default)."""

from __future__ import annotations

import re
import shlex
import subprocess

from x30_health.core.checks.base import Check, Issue, Level, fmt
from x30_health.core.config import ClockConfig

_SYSTEM_TIME = re.compile(r'^System time\s*:\s*([-+0-9.eE]+)\s+seconds\s+(fast|slow)', re.M)
_LEAP = re.compile(r'^Leap status\s*:\s*(.+)$', re.M)


def parse_chrony_tracking(text: str) -> tuple[float | None, bool]:
    """Return (offset s, positive when fast, or None; synchronised) of `chronyc tracking`."""
    leap = _LEAP.search(text)
    synced = leap is not None and 'not synchronised' not in leap.group(1).lower()
    match = _SYSTEM_TIME.search(text)
    if match is None:
        return None, synced
    try:
        value = float(match.group(1))
    except ValueError:
        return None, synced
    return (value if match.group(2) == 'fast' else -value), synced


def run_command(command: str, timeout_sec: float) -> str | None:
    """Return the standard output of command, or None when it fails or times out."""
    try:
        args = shlex.split(command)
        if not args:
            return None
        done = subprocess.run(
            args, capture_output=True, text=True, timeout=timeout_sec, check=True
        )
    except (OSError, ValueError, subprocess.SubprocessError):
        return None
    return done.stdout


class ClockCheck(Check):
    """One finding: the offset to NTP; the command runs at most every period_sec."""

    def __init__(self, config: ClockConfig, runner=run_command):
        self.config = config
        self.runner = runner
        self.last_run: float | None = None
        self.issue = Issue(Level.OK, 'clock:ntp', 'not measured yet')

    def configure(self, config: ClockConfig):
        """Apply live changes."""
        self.config = config

    def run(self, now: float) -> list[Issue]:
        """Return the cached finding, refreshed when period_sec passed."""
        if self.last_run is None or now - self.last_run >= self.config.period_sec:
            self.last_run = now
            self.issue = self._measure()
        return [self.issue]

    def _measure(self) -> Issue:
        c = self.config
        text = self.runner(c.command, c.timeout_sec)
        if text is None:
            return Issue(Level.WARN, 'clock:ntp', f'"{c.command}" failed (chrony installed?)')
        offset, synced = parse_chrony_tracking(text)
        if offset is None:
            return Issue(Level.WARN, 'clock:ntp', 'NTP offset unreadable')
        values = {'offset_sec': offset, 'synchronised': synced}
        if not synced:
            return Issue(Level.WARN, 'clock:ntp', 'clock not synchronised (NTP)', values)
        if abs(offset) > c.max_offset_sec:
            text = f'clock {fmt(abs(offset), 3)} s {"fast" if offset > 0 else "slow"} of NTP'
            return Issue(Level.WARN, 'clock:ntp', text, values)
        return Issue(Level.OK, 'clock:ntp', f'NTP offset {fmt(offset, 4)} s', values)
