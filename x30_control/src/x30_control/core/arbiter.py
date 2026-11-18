"""Who drives: mode and source arbitration, watchdog, e-stop latch and re-arm (pure, no ROS)."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from x30_control.core.command import ZERO, Command
from x30_control.core.config import MuxConfig

NAV = 'navigation'
HARDWARE_REASON = 'hardware e-stop'
HW_SILENT_REASON = 'hardware e-stop signal lost'


class Mode(Enum):
    """Who drives (ControlStatus.MODE_*)."""

    AUTO = 'auto'
    REMOTE = 'remote'


@dataclass(frozen=True)
class Reply:
    """Outcome of a request: accepted, a message for the caller, and whether state changed."""

    success: bool
    message: str
    changed: bool = False


@dataclass(frozen=True)
class ArbiterState:
    """Everything control/status reports about arbitration."""

    mode: Mode
    remote_source: str
    estop: bool
    estop_reasons: tuple[str, ...]
    armed: bool
    hw_pressed: bool


class Arbiter:
    """Mode, source, watchdog and e-stop state machine of the control mux."""

    def __init__(self, config: MuxConfig, now: float):
        self.config = config
        self.mode = Mode(config.mode)
        self.remote_source = config.remote_sources[0] if config.remote_sources else ''
        self.reasons: list[str] = []
        self.hw_pressed = False
        self._disarmed = {NAV, *config.remote_sources}
        self._neutral = 0
        self._last: float | None = None  # last command of the active source
        self._quiet_since = now  # start of the current navigation silence
        self._zero_sent = True  # the watchdog already zeroed the robot for this silence
        self._hw_heard = now  # last control/hw_estop message (start: grace of one timeout)

    @property
    def engaged(self) -> bool:
        """True while the e-stop is latched."""
        return bool(self.reasons)

    @property
    def active_source(self) -> str:
        """NAV in AUTO, the remote source name in REMOTE."""
        return NAV if self.mode is Mode.AUTO else self.remote_source

    @property
    def armed(self) -> bool:
        """True when the active source may drive (no e-stop, re-armed)."""
        return not self.engaged and self.active_source not in self._disarmed

    def state(self) -> ArbiterState:
        """Return a snapshot for control/status."""
        source = self.remote_source if self.mode is Mode.REMOTE else ''
        return ArbiterState(
            self.mode, source, self.engaged, tuple(self.reasons), self.armed, self.hw_pressed
        )

    def set_mode(self, mode: str, source: str, now: float) -> Reply:
        """Switch mode; the same mode and source is a no-op, unknown ones are rejected."""
        try:
            new_mode = Mode(mode)
        except ValueError:
            return Reply(False, f'unknown mode {mode!r}, use {[m.value for m in Mode]}')
        new_source = self.remote_source
        if new_mode is Mode.REMOTE:
            sources = self.config.remote_sources
            new_source = source or (sources[0] if sources else '')
            if new_source not in sources:
                return Reply(False, f'unknown remote source {new_source!r}, use {list(sources)}')
        if new_mode is self.mode and (new_mode is Mode.AUTO or new_source == self.remote_source):
            return Reply(True, f'already {self._describe()}')
        self.mode, self.remote_source = new_mode, new_source
        self._last, self._zero_sent, self._neutral = None, True, 0  # the caller sends the zero
        self._quiet_since = now
        if new_mode is Mode.REMOTE:
            self._disarmed.add(new_source)
        return Reply(True, f'mode {self._describe()}', changed=True)

    def on_command(self, source: str, cmd: Command, now: float) -> Command | None:
        """Return what to send for a command of source: the command, zero, or None (ignored)."""
        if source != self.active_source:
            return None
        self._last, self._zero_sent = now, False
        if self.engaged:
            return None  # the hold timer streams zero
        if self.armed:
            return cmd
        if cmd.is_neutral(self.config.neutral_eps):
            self._neutral += 1
            if self._neutral >= self.config.rearm_neutral_count:
                self._rearm()
        else:
            self._neutral, self._quiet_since = 0, now
        return ZERO

    def tick(self, now: float) -> bool:
        """Run the watchdogs; return True when one zero must be sent (silence or e-stop)."""
        timeout = self.config.hw_estop_timeout_sec
        if timeout > 0.0 and now - self._hw_heard > timeout and self.engage(HW_SILENT_REASON):
            return True
        silent = self._last is None or now - self._last > self._timeout()
        if self.mode is Mode.AUTO and not self.engaged and NAV in self._disarmed:
            last = self._quiet_since if self._last is None else max(self._quiet_since, self._last)
            if now - last > self.config.nav_timeout_sec:
                self._rearm()
        if not silent or self._zero_sent:
            return False
        self._zero_sent = True
        return True

    def engage(self, reason: str) -> bool:
        """Latch the e-stop with a reason (oldest first, no duplicates); True when added."""
        reason = (reason or '').strip() or 'unspecified'
        if reason in self.reasons:
            return False
        self.reasons.append(reason)
        self._disarm_all()
        return True

    def release(self, now: float) -> Reply:
        """Clear every reason unless the hardware e-stop is pressed; every source must re-arm."""
        if self.hw_pressed:
            return Reply(False, 'hardware e-stop is pressed: release it first')
        if not self.engaged:
            return Reply(True, 'e-stop was not engaged')
        released = ', '.join(self.reasons)
        self.reasons.clear()
        self._disarm_all()
        self._quiet_since = now
        return Reply(True, f'e-stop released ({released}); waiting for a neutral command', True)

    def set_hardware(self, pressed: bool, now: float) -> bool:
        """Track the hardware e-stop; pressing engages. Return True when the state changed."""
        self._hw_heard = now
        changed = pressed != self.hw_pressed
        self.hw_pressed = pressed
        if pressed:
            changed = self.engage(HARDWARE_REASON) or changed
        return changed

    def _rearm(self):
        self._disarmed.discard(self.active_source)
        self._neutral = 0

    def _disarm_all(self):
        self._disarmed = {NAV, *self.config.remote_sources}
        self._neutral = 0

    def _timeout(self) -> float:
        cfg = self.config
        return cfg.nav_timeout_sec if self.mode is Mode.AUTO else cfg.remote_timeout_sec

    def _describe(self) -> str:
        return 'auto' if self.mode is Mode.AUTO else f'remote ({self.remote_source})'
