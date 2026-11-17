"""The hook phases of docking: sit down, start charging, wait for the battery; undock again."""

from __future__ import annotations

from x30_navigation.core.docking.state import BUSY, D, DockInputs, HookRequest

HOOK_GROUPS = ('sit', 'charge', 'stop_charge', 'stand')


class SeatedPhases:
    """Mixin of Docking: the states driven by the robot's Trigger hooks."""

    answer: tuple[bool, str] | None = None

    def undock(self) -> tuple[bool, str]:
        """Stop charging, stand up and back away from the dock (also after a failure)."""
        if self.state in BUSY:
            return False, f'busy ({self.state.name}): cancel first'
        self.attempt = 0
        self._go(D.UNDOCKING, 'undocking: stopping the charge')
        self._request('stop_charge')
        return True, 'undocking started'

    def on_hooks(self, ticket: int, ok: bool, message: str) -> bool:
        """Take the answer of the hooks handed out with ticket; False when it is too late."""
        if ticket != self.ticket or self.step not in HOOK_GROUPS:
            return False
        self.answer = (bool(ok), message)
        return True

    def _start_seated(self):
        """Stop on the dock (one zero), then call the sit hooks."""
        self.zero_once = True
        self._go(D.SITTING, 'on the dock: sitting down')
        self._request('sit')

    def _request(self, group: str):
        self.ticket += 1
        self.step, self.answer = group, None
        self.pending = HookRequest(self.ticket, group)

    def _next_step(self, step: str):
        self.step, self.answer = step, None
        self.clock.reset()

    def _answered(self, what: str) -> bool | None:
        """Return True / False once the hooks of the current step answered (None: waiting)."""
        if self.answer is None:
            return None
        ok, message = self.answer
        self.answer = None
        if not ok:
            self._fail(f'{what} failed: {message or "no reason given"}')
        return ok

    def _seated(self, inputs: DockInputs, dt: float):
        """Run one tick of SITTING, CHARGING, DOCKED or the hook steps of UNDOCKING."""
        self.clock.run(dt)
        handler = {
            D.SITTING: self._sitting,
            D.CHARGING: self._charging,
            D.DOCKED: self._docked,
            D.UNDOCKING: self._undocking,
        }.get(self.state)
        if handler is not None:
            handler(inputs)

    def _sitting(self, inputs: DockInputs):
        hooks = self.config.hooks
        if self.step == 'sit' and self._answered('sitting down'):
            self.seated = True
            self._next_step('sit_settle')
        elif self.step == 'sit_settle' and self.clock.elapsed >= hooks.sit_settle_sec:
            self._go(D.CHARGING, 'sitting: starting the charge')
            self._request('charge')

    def _charging(self, inputs: DockInputs):
        timeout = self.config.charge.confirm_timeout_sec
        if self.step == 'charge' and self._answered('starting the charge'):
            if timeout <= 0.0:
                self._docked_now('docked (charging not confirmed: confirm_timeout_sec 0)')
            else:
                self._next_step('confirm')
                self.message = 'charge started: waiting for the battery to report charging'
        elif self.step == 'confirm':
            if inputs.charging:
                self._docked_now('docked: charging')
            elif self.clock.elapsed > timeout:
                self._fail(f'the battery does not report charging after {timeout:.0f} s')

    def _docked_now(self, message: str):
        self.step = ''
        self._go(D.DOCKED, message)

    def _docked(self, inputs: DockInputs):
        message = 'docked: charging' if inputs.charging else 'docked: battery not charging'
        if inputs.charging is None:
            message = 'docked: no battery state'
        if message != self.message:
            self.message = message
            self.notes.append(message)

    def _undocking(self, inputs: DockInputs):
        hooks = self.config.hooks
        if self.step == 'stop_charge' and self._answered('stopping the charge'):
            self.message = 'charge stopped: standing up'
            self._request('stand')
        elif self.step == 'stand' and self._answered('standing up'):
            self.seated = False
            self._next_step('stand_settle')
        elif self.step == 'stand_settle' and self.clock.elapsed >= hooks.stand_settle_sec:
            if self.config.undock.distance <= 0.0:
                self.step = ''
                self._go(D.IDLE, 'undocked')
                return
            self._next_step('reverse')
            self.anchor = None
            self.stall.reset()
            self.message = 'standing: backing away from the dock'
