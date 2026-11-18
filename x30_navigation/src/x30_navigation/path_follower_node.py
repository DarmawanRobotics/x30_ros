"""path_follower_node: the ROS side of core.follower.Follower."""

from __future__ import annotations

import threading

from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger

from x30_msgs.msg import (
    AvoidanceCommand,
    ControlStatus,
    LocalizationStatus,
    SectorScan,
    WaypointPath,
)
from x30_msgs.srv import SetPath
from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers import CONTROLLERS, Command, ControllerLimits
from x30_navigation.core.follower.fsm import Follower
from x30_navigation.core.follower.state import Inputs, PoseInput, ScanInput
from x30_navigation.core.path.processing import process
from x30_navigation.core.path.route import Route
from x30_navigation.utils import conversions as conv
from x30_navigation.utils.control_check import ControlFieldCheck
from x30_navigation.utils.hooks import FollowerHooks
from x30_navigation.utils.outputs import FollowerOutputs
from x30_navigation.utils.pose_wait import wait_for_pose
from x30_utils.ros import COMMAND, LATCHED, SENSOR, X30Node, run
from x30_utils.ros.time import monotonic

NAME = 'path_follower_node'


class PathFollowerNode(X30Node):
    """Follow routes and planned paths; publishes control/nav_cmd_vel."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(FollowerConfig)
        self.robot = self.declare_robot()
        self.controller_configs = {
            name: self.declare_config(CONTROLLERS.get(name).Config, f'{name}.')
            for name in CONTROLLERS.names()
        }
        self.lock = threading.RLock()
        self.follower = Follower(self.config.value, self.robot.value, self.make_controller())
        self.scan, self.avoidance, self.twist = None, None, None
        self.waypoints_stamp, self.last_tick = None, None
        self.hooks = FollowerHooks(self, self.config.hooks, self.lock)
        self.out = FollowerOutputs(self)
        safety = self.config.safety  # the margins are read-only
        self.field_check = ControlFieldCheck(self, self.config.control_node, safety.check_control)
        self.subscriptions_and_services()
        self.config.on_change(self.on_config)
        self.robot.on_change(lambda old, new: self.locked(self.on_robot)())
        for name, cfg in self.controller_configs.items():
            cfg.on_change(lambda old, new, name=name: self.locked(self.on_controller_config)(name))
        self.add_shutdown_hook(self.on_shutdown)
        period = 1.0 / self.config.control_rate
        self.every(period, self.locked(self.on_tick), on_error=self.fail)

    def subscriptions_and_services(self):
        """Wire every input and service: one at a time, as the node's callbacks."""
        sub = self.create_subscription
        sub(WaypointPath, 'navigation/waypoints', self.locked(self.on_waypoints), LATCHED)
        sub(SectorScan, 'perception/safety_scan', self.on_scan, SENSOR)
        sub(AvoidanceCommand, 'navigation/avoidance', self.on_avoidance, COMMAND)
        sub(ControlStatus, 'control/status', self.locked(self.on_control), LATCHED)
        sub(LocalizationStatus, 'localization/status', self.locked(self.on_localization), LATCHED)
        sub(BatteryState, 'battery', self.locked(self.on_battery), SENSOR)
        sub(Odometry, 'odom', self.on_odom, SENSOR)
        srv = self.create_service
        srv(SetPath, 'navigation/set_path', self.locked(self.on_set_path))
        for name, handler in (('stop', self.on_stop), ('pause', self.on_pause),
                              ('resume', self.on_resume)):  # fmt: skip
            srv(Trigger, f'navigation/{name}', self.locked(handler))
        # start waits for the pose beside the other callbacks: the tick and the inputs keep running
        srv(Trigger, 'navigation/start', self.on_start, concurrent=True)

    def locked(self, fn):
        """Return fn running under the state lock."""

        def run_locked(*args):
            with self.lock:
                return fn(*args)

        return run_locked

    def make_controller(self, name: str | None = None):
        """Build the configured path controller with the robot's limits."""
        name = name or self.config.controller
        limits = self.robot.limits
        period = 1.0 / self.config.control_rate
        lim = ControllerLimits(limits.max_vx, limits.max_vy if self.robot.can_strafe else 0.0,
                               limits.max_wz, period)  # fmt: skip
        return CONTROLLERS.create(name, self.controller_configs[name].value, lim)

    def pose_input(self) -> PoseInput | None:
        """Return the robot pose from TF with its age (frames from robot.frames)."""
        frames = self.robot.frames
        result = self.tf.lookup(frames.map, frames.base)
        return None if result is None else PoseInput(result.pose2d, result.age)

    def on_tick(self):
        """Run one control tick and publish what it decided."""
        now = monotonic()
        dt = 1.0 / self.config.control_rate if self.last_tick is None else now - self.last_tick
        self.last_tick = now
        scan, avoidance, age = None, None, float('inf')
        if self.scan is not None:
            points, centres, blocked, at = self.scan
            scan = ScanInput(points, blocked, now - at, centres)
        if self.avoidance is not None:
            avoidance, at = self.avoidance
            age = now - at
        twist = self.follower.shaper.last
        if self.twist is not None and now - self.twist[1] < 0.5:
            twist = self.twist[0]
        dt = min(max(dt, 0.0), 0.5)
        inputs = Inputs(now, dt, self.pose_input(), scan, avoidance, age, twist)
        self.publish(self.follower.tick(inputs))

    def publish(self, decision):
        """Publish a decision and start the inspections it asks for."""
        self.out.decision(decision, self.follower, self.config.markers)
        if decision.inspect is not None:
            self.hooks.run_inspection(self.follower, decision.inspect[1], self.out.events)

    def take_route(self, points, source: str) -> tuple[bool, str]:
        """Give the follower a route; a running follower starts over on it."""
        try:
            route = Route.from_points(points)
        except ValueError as error:
            return False, f'route from {source} refused: {error}'
        pose = self.pose_input()
        ok, message = self.follower.set_route(route, pose.pose if pose else None)
        self.out.route_key = None
        self.log.info('route', source=source, points=len(route), ok=ok, detail=message)
        if not ok:
            self.zero()
        return ok, message

    def on_waypoints(self, msg: WaypointPath):
        """Take a route published by path_loader_node; a redelivery (same stamp) is ignored."""
        stamp = (msg.header.stamp.secs, msg.header.stamp.nsecs)
        if stamp == self.waypoints_stamp:
            return
        self.waypoints_stamp = stamp
        try:
            self.take_route(conv.route_points(msg.waypoints), 'navigation/waypoints')
        except ValueError as error:
            self.log.error('route refused', error=str(error))

    def on_set_path(self, request, response):
        """Service: take a path (the planner); process it like a route when asked."""
        try:
            points = conv.route_points(request.waypoints)
        except ValueError as error:
            response.success, response.message = False, str(error)
            return response
        if request.process and len(points) >= 3:
            points, _ = process(points, self.config.processing)
        source = request.source.strip() or 'navigation/set_path'
        response.success, response.message = self.take_route(points, source)
        return response

    def on_start(self, request, response):
        """Service: start (after the start hooks, asynchronously) from where the robot is."""
        # outside the state lock: the tick, TF and the other services run meanwhile
        pose = wait_for_pose(self, self.pose_input, self.config.pose_wait_sec)
        with self.lock:
            return self.start_locked(pose, response)

    def start_locked(self, pose, response):
        """Start with the pose found (under the state lock)."""
        if not self.hooks.has_start:
            response.success, response.message = self.follower.start(pose.pose if pose else None)
            return response
        response.success, response.message = True, 'starting: waiting for the start hooks'

        def then(ok, message):
            if not ok:
                self.log.error('start hooks failed, not starting', detail=message)
                return
            now = self.pose_input()
            started, note = self.follower.start(now.pose if now else None)
            self.log.info('started' if started else 'start refused', detail=note)

        self.hooks.run_start(then)
        return response

    def on_stop(self, request, response):
        """Service: stop navigation (one zero now)."""
        self.hooks.cancel_start()
        self.out.command(self.follower.stop('stopped by navigation/stop'))
        self.hooks.run_stop()
        response.success, response.message = True, 'stopped'
        return response

    def on_pause(self, request, response):
        """Service: hold position, keep the progress."""
        self.follower.pause()
        self.zero()
        response.success, response.message = True, 'paused'
        return response

    def on_resume(self, request, response):
        """Service: clear the operator pause and a released e-stop."""
        response.success, response.message = self.follower.resume()
        return response

    def on_scan(self, msg: SectorScan):
        """Keep the safety scan as points with blocked sectors (in robot.frames.base)."""
        data = conv.scan_data(msg)
        sectors, centres = data.points(), data.points(edges=False)
        blocked = sectors.any_blocked or msg.header.frame_id not in ('', self.robot.frames.base)
        self.scan = (sectors.points, centres.points, blocked, monotonic())

    def on_avoidance(self, msg: AvoidanceCommand):
        """Keep the latest avoidance output."""
        self.avoidance = (conv.avoidance_out(msg), monotonic())

    def on_odom(self, msg: Odometry):
        """Keep the measured body twist (used to predict a slightly old pose)."""
        t = msg.twist.twist
        self.twist = (Command(t.linear.x, t.linear.y, t.angular.z), monotonic())

    def on_control(self, msg: ControlStatus):
        """Pause reasons from the control mux: estop (sticky), remote mode, not armed."""
        remote = msg.mode == ControlStatus.MODE_REMOTE
        self.follower.on_control(bool(msg.estop), remote, bool(msg.armed))

    def on_localization(self, msg: LocalizationStatus):
        """Pause while not localized."""
        self.follower.on_localization(msg.state != LocalizationStatus.NONE)

    def on_battery(self, msg: BatteryState):
        """Low battery: no new lap; critical: pause."""
        self.follower.on_battery(float(msg.percentage))

    def on_config(self, old, new):
        """Apply new parameters (under the state lock)."""
        with self.lock:
            self.follower.configure(new)
            self.hooks.config = new.hooks
            if new.controller != old.controller:
                self.follower.set_controller(new.controller, self.make_controller(new.controller))
            self.log.info('parameters applied', controller=new.controller)

    def on_controller_config(self, name: str):
        """Rebuild the controller whose parameters changed."""
        if name == self.config.controller:
            self.follower.set_controller(name, self.make_controller(name))

    def on_robot(self):
        """Stop and rebuild the follower on the same route for a new robot profile."""
        route = self.follower.tracker.route if self.follower.tracker else None
        self.zero()
        self.follower.stop('robot profile changed')
        self.follower = Follower(self.config.value, self.robot.value, self.make_controller())
        if route is not None:
            self.follower.set_route(route)
        self.log.warning('robot profile changed: navigation stopped')

    def zero(self):
        """Publish one zero command."""
        self.out.command(self.follower.shaper.stop())

    def fail(self, error: Exception):
        """Stop (zero) and log it when a control tick raised."""
        self.log.exception('control tick failed, navigation stopped', error=str(error))
        with self.lock:
            self.out.command(self.follower.stop(f'internal error: {error}'))

    def on_shutdown(self):
        """Leave the robot with a zero command."""
        for _ in range(3):
            self.out.command(Command())


def main():
    """Run path_follower_node."""
    run(PathFollowerNode, NAME)


if __name__ == '__main__':
    main()
