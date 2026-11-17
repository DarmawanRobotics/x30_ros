"""docking_node: dock onto a charger marked by a fiducial tag, sit down and start charging."""

from __future__ import annotations

import threading
import time

from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from x30_msgs.msg import ControlStatus, DockingStatus
from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.controllers.base import Command
from x30_navigation.core.docking.control import DockLimits
from x30_navigation.core.docking.fsm import Docking
from x30_navigation.core.docking.geometry import field_problems
from x30_navigation.core.docking.state import ATTEMPT, D, DockInputs, PoseInput
from x30_navigation.utils import conversions as conv
from x30_navigation.utils import docking as dk
from x30_navigation.utils.control_check import ControlFieldCheck
from x30_utils.ros import COMMAND, LATCHED, SENSOR, X30Node, run
from x30_utils.ros.time import monotonic

SHUTDOWN_ZERO_GAP_SEC = 0.01


NAME = 'docking_node'


class DockingNode(X30Node):
    """Docking onto a tag-marked charger; publishes control/nav_cmd_vel while it moves."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(DockingConfig)
        self.robot = self.declare_robot()
        self.lock = threading.RLock()
        self.docking = Docking(self.config.value, self.robot.can_strafe, self.limits())
        self.control = (False, False, True)  # estop, remote, armed (no mux heard: armed)
        self.battery: tuple[bool, float] | None = None  # (charging, monotonic time)
        self.hooks = dk.DockHooks(self, self.config.hooks)
        self.cmd_pub = self.create_publisher(Twist, 'control/nav_cmd_vel', COMMAND)
        self.status_pub = self.latched(DockingStatus, 'docking/status')
        self.goal_pub = self.create_publisher(PoseStamped, 'docking/goal', 1)
        sub = self.create_subscription
        sub(ControlStatus, 'control/status', self.locked(self.on_control), LATCHED)
        sub(BatteryState, 'battery', self.locked(self.on_battery), SENSOR)
        for name, handler in (('dock', self.on_dock), ('undock', self.on_undock),
                              ('cancel', self.on_cancel)):  # fmt: skip
            self.create_service(Trigger, f'docking/{name}', self.locked(handler))
        self.config.on_change(lambda old, new: self.locked(self.docking.configure)(new))
        self.robot.on_change(lambda old, new: self.locked(self.on_robot)())
        self.field_check = ControlFieldCheck(
            self, self.config.control_node, self.check_field,
            'docking goal inside the control field', 'docking goal clear of the control field',
        )  # fmt: skip
        self.add_shutdown_hook(self.on_shutdown)
        period = 1.0 / self.config.control_rate
        self.every(period, self.locked(self.on_tick), on_error=self.fail)
        self.log.info('docking ready', tag=self.config.tag.frame, goal=self.config.goal.distance)

    def locked(self, fn):
        """Return fn running under the state lock."""

        def run_locked(*args):
            with self.lock:
                return fn(*args)

        return run_locked

    def limits(self) -> DockLimits:
        """Return the speed caps of the robot profile."""
        limits, strafe = self.robot.limits, self.robot.can_strafe
        return DockLimits(limits.max_vx, limits.max_vy if strafe else 0.0, limits.max_wz)

    def check_field(self, lateral_margin: float, human_speed: float, margin: float) -> list:
        """Return why the mux field (its values) would hold the robot short of the dock."""
        cfg = self.config
        speed = cfg.approach.min_speed
        return field_problems(cfg.goal, self.robot.value, speed, margin, human_speed)

    # ------------------------------------------------------------------ inputs

    def pose_input(self) -> PoseInput | None:
        """Return base_link in odom from TF with its age."""
        frames = self.robot.frames
        result = self.tf.lookup(frames.odom, frames.base)
        return None if result is None else PoseInput(result.pose2d, result.age)

    def inputs(self) -> DockInputs:
        """Collect one tick's inputs."""
        pose = self.pose_input()
        t = self.config.tag
        tag = dk.tag_input(self.tf, self.robot.frames.odom, t.frame, pose, t.normal_axis)
        charging = None
        if self.battery is not None:
            fresh = monotonic() - self.battery[1] <= self.config.charge.battery_timeout_sec
            charging = self.battery[0] if fresh else None
        estop, remote, armed = self.control
        return DockInputs(monotonic(), pose, tag, estop, remote, armed, charging)

    def on_control(self, msg: ControlStatus):
        """Keep the e-stop, REMOTE mode and whether navigation is armed."""
        self.control = (bool(msg.estop), msg.mode == ControlStatus.MODE_REMOTE, bool(msg.armed))

    def on_battery(self, msg: BatteryState):
        """Keep whether the battery charges."""
        self.battery = (dk.is_charging(msg), monotonic())

    # ------------------------------------------------------------------ services

    def on_dock(self, request, response):
        """Service: stop the follower and start docking from where the robot stands."""
        response.success, response.message = self.docking.dock()
        if response.success:
            self.hooks.stop_navigation()
        self.log.info('dock requested', ok=response.success, detail=response.message)
        return response

    def on_undock(self, request, response):
        """Service: stop charging, stand up, back away from the dock."""
        response.success, response.message = self.docking.undock()
        self.log.info('undock requested', ok=response.success, detail=response.message)
        return response

    def on_cancel(self, request, response):
        """Service: stop docking or undocking now (one zero if the robot was moving)."""
        response.success, response.message = self.docking.cancel('cancelled by docking/cancel')
        return response

    # ------------------------------------------------------------------ the loop

    def on_tick(self):
        """Run one tick and publish what it decided."""
        decision = self.docking.tick(self.inputs())
        if decision.command is not None:
            self.cmd_pub.publish(conv.twist(decision.command))
        header = self.header()
        self.status_pub.publish(dk.status_msg(decision.report, header.frame_id, header.stamp))
        if decision.goal is not None and decision.report.state in ATTEMPT:
            self.goal_pub.publish(dk.goal_msg(decision.goal, header.frame_id, header.stamp))
        for note in decision.notes:
            self.log.event('docking', state=decision.report.state.name, detail=note)
        if decision.report.state is D.FAILED and decision.changed:
            self.log.error('docking failed', reason=decision.report.message)
        if decision.hooks is not None:
            self.run_hooks(decision.hooks)

    def run_hooks(self, request):
        """Call a hook group; its answer goes back to the state machine under the lock."""

        def then(ok, message):
            with self.lock:
                taken = self.docking.on_hooks(request.ticket, ok, message)
            self.log.info('docking hooks', group=request.group, ok=ok, detail=message,
                          taken=taken)  # fmt: skip

        self.hooks.call(request.group, then)

    def header(self) -> Header:
        """Return a header in the odom frame, stamped now."""
        return Header(frame_id=self.robot.frames.odom, stamp=self.stamp())

    def on_robot(self):
        """Stop and rebuild the state machine for a new robot profile."""
        moving = self.docking.state in ATTEMPT
        self.docking.cancel('robot profile changed')
        self.docking = Docking(self.config.value, self.robot.can_strafe, self.limits())
        if moving:
            self.cmd_pub.publish(conv.twist(Command()))
        self.log.warning('robot profile changed: docking stopped')

    def fail(self, error: Exception):
        """Stop (zero) and log it when a tick raised."""
        self.log.exception('docking tick failed, docking stopped', error=str(error))
        with self.lock:
            self.docking.cancel(f'internal error: {error}')
            self.cmd_pub.publish(conv.twist(Command()))

    def on_shutdown(self):
        """Leave a moving robot with a zero command (run() stopped the tick before)."""
        if self.docking.state in ATTEMPT or self.docking.step == 'reverse':
            for _ in range(3):
                self.cmd_pub.publish(conv.twist(Command()))
                time.sleep(SHUTDOWN_ZERO_GAP_SEC)  # depth-1 COMMAND QoS: Foxy drops bursts


def main():
    """Run docking_node."""
    run(DockingNode, NAME)


if __name__ == '__main__':
    main()
