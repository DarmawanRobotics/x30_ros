"""control_mux_node: the only publisher of cmd_vel (arbitration, envelope, field, e-stop)."""

from __future__ import annotations

import functools
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from x30_control.core.arbiter import NAV, Arbiter, ArbiterState, Mode
from x30_control.core.command import ZERO, Command
from x30_control.core.config import FieldConfig, MuxConfig
from x30_control.core.field import FieldState
from x30_control.core.gate import Gate
from x30_control.utils.convert import status_msg, to_command, to_scan, to_twist
from x30_msgs.msg import ControlStatus, HealthStatus, SectorScan
from x30_msgs.srv import SetMode
from x30_utils.ros import COMMAND, EVENTS, SENSOR, X30Node, run

NAME = 'control_mux_node'

INTERNAL_ERROR = 'internal error'
WATCHDOG_PERIOD_SEC = 0.02
SHUTDOWN_ZEROS = 3
SHUTDOWN_GAP_SEC = 0.01  # each zero leaves (the publisher queue holds one message)


class ControlMuxNode(X30Node):
    """Forward navigation (AUTO) or one operator source (REMOTE) to cmd_vel, safely."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self._robot = self.declare_robot().value
        self._mux = self.declare_config(MuxConfig).value
        self._field_cfg = self.declare_config(FieldConfig, 'field.').value
        self._arb = Arbiter(self._mux, time.monotonic())
        self._gate = Gate(self._robot, self._mux, self._field_cfg)
        self._field = (FieldState.STALE, 0.0, 0.0)
        self._field_time: float | None = None
        self._reported = None
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', COMMAND)
        self._status_pub = self.latched(ControlStatus, 'control/status')
        self._wire()
        self.add_shutdown_hook(self._stop_on_shutdown)
        self._publish_zero()
        self._publish_status(force=True)
        self.log.info(
            'control mux ready',
            mode=self._mux.mode,
            sources=','.join(self._mux.remote_sources),
            robot=self._robot.name,
            field_in_remote=self._field_cfg.in_remote,
        )

    def _wire(self):
        """Create subscriptions, services and steady-clock timers."""
        guard = self._guarded
        self.create_subscription(Twist, 'control/nav_cmd_vel', guard(self._on_nav), COMMAND)
        for source in self._mux.remote_sources:
            callback = guard(functools.partial(self._on_remote, source))
            self.create_subscription(Twist, f'control/remote/{source}/cmd_vel', callback, COMMAND)
        self.create_subscription(
            SectorScan, 'perception/safety_scan', guard(self._on_scan), SENSOR
        )
        self.create_subscription(String, 'control/estop_request', guard(self._on_request), EVENTS)
        self.create_subscription(Bool, 'control/hw_estop', guard(self._on_hardware), EVENTS)
        if self._mux.use_health_limit:
            self.create_subscription(
                HealthStatus, 'health/status', guard(self._on_health), COMMAND
            )
        self.create_service(SetMode, 'control/set_mode', self._service(self._on_set_mode))
        self.create_service(Trigger, 'control/estop/engage', self._service(self._on_engage))
        self.create_service(Trigger, 'control/estop/release', self._service(self._on_release))
        self.every(WATCHDOG_PERIOD_SEC, self._watchdog, steady=True, on_error=self._fail)
        hold = 1.0 / self._mux.estop_hold_hz
        self.every(hold, self._hold, steady=True, on_error=self._fail)
        self.every(1.0 / self._mux.status_hz, self._status_tick, steady=True, on_error=self._fail)

    # inputs -------------------------------------------------------------------------------

    def _on_nav(self, msg: Twist):
        self._forward(NAV, to_command(msg))

    def _on_remote(self, source: str, msg: Twist):
        self._forward(source, to_command(msg))

    def _forward(self, source: str, cmd: Command):
        """Arbitrate, gate and publish one command of a source."""
        mono = time.monotonic()
        out = self._arb.on_command(source, cmd, mono)
        if out is None:
            return
        remote = self._arb.mode is Mode.REMOTE
        result = self._gate.apply(out, remote, mono, self._now())
        if not result.finite:
            self.log.error('non-finite command dropped', source=source, throttle=1.0)
        elif result.clipped:
            self.log.warning(
                'command above the speed envelope, scaled', source=source, throttle=5.0
            )
        self._cmd_pub.publish(to_twist(result.command))
        self._set_field(result.field, result.clearance, result.speed_scale, mono)
        self._publish_status()

    def _on_scan(self, msg: SectorScan):
        problem = self._gate.update_scan(to_scan(msg), time.monotonic())
        if problem:
            self.log.warning('safety scan rejected', reason=problem, throttle=2.0)

    def _on_request(self, msg: String):
        self._engage(msg.data or 'topic')

    def _on_hardware(self, msg: Bool):
        pressed = bool(msg.data)
        if pressed:
            self._publish_zero()
        if self._arb.set_hardware(pressed, time.monotonic()):
            self.log.event('hw_estop', pressed=pressed)
            self._publish_status()

    def _on_health(self, msg: HealthStatus):
        self._gate.health.update(float(msg.speed_scale), time.monotonic())

    # services -----------------------------------------------------------------------------

    def _on_set_mode(self, request, response):
        reply = self._arb.set_mode(request.mode, request.source, time.monotonic())
        if reply.changed:
            self._publish_zero()
            self.log.event('mode', mode=request.mode, source=self._arb.remote_source)
            self._publish_status()
        response.success, response.message = reply.success, reply.message
        return response

    def _on_engage(self, _request, response):
        self._engage('service')
        response.success = True
        response.message = f'e-stop engaged: {", ".join(self._arb.reasons)}'
        return response

    def _on_release(self, _request, response):
        reply = self._arb.release(time.monotonic())
        if reply.changed:
            self._publish_zero()
            self.log.event('estop_released', detail=reply.message)
            self._publish_status()
        elif not reply.success:
            self.log.warning('e-stop release refused', reason=reply.message)
        response.success, response.message = reply.success, reply.message
        return response

    # timers -------------------------------------------------------------------------------

    def _watchdog(self):
        if self._arb.tick(time.monotonic()):
            self._publish_zero()
            self._publish_status()

    def _hold(self):
        if self._arb.engaged:
            self._publish_zero()

    def _status_tick(self):
        self._publish_status(force=True)

    # helpers ------------------------------------------------------------------------------

    def _engage(self, reason: str):
        """Latch the e-stop with a reason and stop at once."""
        self._publish_zero()
        if self._arb.engage(reason):
            self.log.error('E-STOP engaged', reason=reason, reasons=','.join(self._arb.reasons))
            self._publish_status()

    def _set_field(self, state: FieldState, clearance: float, scale: float, mono: float):
        if state is not self._field[0]:
            log = (
                self.log.warning
                if state in (FieldState.STOP, FieldState.STALE, FieldState.BLIND)
                else self.log.info
            )
            log('protective field', state=state.value)
        self._field, self._field_time = (state, clearance, scale), mono

    def _current_field(self, mono: float) -> tuple:
        """Field of the newest command, or of a robot standing still when nothing drives."""
        timeout = (
            self._mux.nav_timeout_sec
            if self._arb.mode is Mode.AUTO
            else self._mux.remote_timeout_sec
        )
        if self._field_time is None or mono - self._field_time > timeout:
            idle = self._gate.idle_state(self._arb.mode is Mode.REMOTE, mono, self._now())
            self._set_field(idle.state, idle.clearance, idle.scale, mono)
            self._field_time = None
        return self._field

    def _publish_status(self, force: bool = False):
        state: ArbiterState = self._arb.state()
        field, clearance, scale = self._current_field(time.monotonic())
        key = (state, field)
        if not force and key == self._reported:
            return
        self._reported = key
        self._status_pub.publish(
            status_msg(state, field, clearance, scale, self.stamp(), self._robot.frames.base)
        )

    def _publish_zero(self):
        self._cmd_pub.publish(to_twist(ZERO))

    def _now(self) -> float:
        return self.now()

    def _fail(self, error: Exception):
        """Fail safe: any callback error engages the e-stop and stops the robot."""
        self.log.exception('callback failed, engaging the e-stop', error=repr(error))
        try:
            self._engage(INTERNAL_ERROR)
        except Exception:
            self.log.exception('could not engage the e-stop after an error')
            self._publish_zero()

    def _guarded(self, callback):
        def guarded(*args):
            try:
                callback(*args)
            except Exception as error:
                self._fail(error)

        return guarded

    def _service(self, callback):
        def guarded(request, response):
            try:
                return callback(request, response)
            except Exception as error:
                self._fail(error)
                response.success, response.message = False, f'{INTERNAL_ERROR}: {error}'
                return response

        return guarded

    def _stop_on_shutdown(self):
        for i in range(SHUTDOWN_ZEROS):
            if i:
                time.sleep(SHUTDOWN_GAP_SEC)
            self._publish_zero()


def main():
    """Run the control mux."""
    run(ControlMuxNode, NAME)


if __name__ == '__main__':
    main()
