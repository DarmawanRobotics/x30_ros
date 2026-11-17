"""Why navigation is paused: a set of reasons, each with its own way of clearing."""

from __future__ import annotations

from enum import Enum


class PauseReason(Enum):
    """Pause reasons as reported in NavigationStatus.pause_reasons."""

    OPERATOR = 'operator'
    ESTOP = 'estop'
    REMOTE = 'remote'
    LOCALIZATION = 'localization'
    BATTERY = 'battery'
    DISARMED = 'disarmed'


R = PauseReason


class PauseReasons:
    """The active pause reasons: operator, estop, remote, localization, battery, disarmed."""

    def __init__(self):
        self.reasons: set[PauseReason] = set()
        self.estop_engaged = False

    @property
    def active(self) -> bool:
        """True while any reason holds."""
        return bool(self.reasons)

    def names(self) -> tuple[str, ...]:
        """Return the reason names, sorted."""
        return tuple(sorted(r.value for r in self.reasons))

    def pause(self):
        """Add the operator reason."""
        self.reasons.add(R.OPERATOR)

    def resume(self) -> tuple[bool, str]:
        """Clear operator and a released e-stop; refuse while the e-stop is still engaged."""
        self.reasons.discard(R.OPERATOR)
        if self.estop_engaged:
            return False, 'the e-stop is engaged: release it (control/estop/release) first'
        self.reasons.discard(R.ESTOP)
        return True, 'resumed' if not self.reasons else f'still paused: {", ".join(self.names())}'

    def blocking_start(self) -> tuple[str, ...]:
        """Return the reasons a start cannot clear (everything but the operator)."""
        return tuple(sorted(r.value for r in self.reasons if r is not R.OPERATOR))

    def clear_operator(self):
        """Drop the operator reason (an explicit start)."""
        self.reasons.discard(R.OPERATOR)

    def on_control(self, estop: bool, remote: bool, armed: bool = True):
        """Apply control/status: the e-stop latches the sticky reason, the others are transient."""
        self.estop_engaged = estop
        if estop:
            self.reasons.add(R.ESTOP)
        self._set(R.REMOTE, remote)
        self._set(R.DISARMED, not armed and not estop and not remote)

    def on_localization(self, localized: bool, required: bool):
        """Pause while not localized (when required); clear when localized again."""
        self._set(R.LOCALIZATION, required and not localized)

    def on_battery(self, critical: bool):
        """Pause while the battery is critical."""
        self._set(R.BATTERY, critical)

    def _set(self, reason: PauseReason, on: bool):
        if on:
            self.reasons.add(reason)
        else:
            self.reasons.discard(reason)
