"""planner_node: plan on the site map to a pose, hand the path to the follower, replan."""

from __future__ import annotations

import math
import threading

from nav_msgs.msg import OccupancyGrid, Path
from std_srvs.srv import Trigger, TriggerRequest

from x30_msgs.msg import NavigationStatus, PlannerStatus, SectorScan
from x30_msgs.srv import GoTo, PlanPath, SetPath, SetPathRequest
from x30_navigation.core.config.planner import PlannerConfig
from x30_navigation.core.follower.state import FollowerState
from x30_navigation.core.planning.costmap import build_costmap
from x30_navigation.core.planning.footprint import FootprintChecker
from x30_navigation.core.planning.planner import Plan, Planner
from x30_navigation.core.planning.session import Action, GoalSession, PlannerState
from x30_navigation.utils import conversions as conv
from x30_navigation.utils.clients import AsyncCall, ServiceClient, call_and_wait
from x30_navigation.utils.pose_wait import wait_for_pose
from x30_utils.math import Pose2D
from x30_utils.ros import LATCHED, SENSOR, X30Node, run
from x30_utils.ros.time import monotonic

NAME = 'planner_node'


class PlannerNode(X30Node):
    """planner/go_to, planner/plan and planner/cancel on /map."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(PlannerConfig)
        self.robot = self.declare_robot()
        self.grid, self.costmap, self.scan, self.nav = None, None, None, None
        self.session = GoalSession(self.config.monitor)
        self.config.on_change(self.on_config)
        self.robot.on_change(lambda old, new: self.invalidate())
        # plan and go_to wait for the follower beside the other callbacks, one at a time with
        # the monitor (work); cancel never waits for them
        self.work = threading.RLock()
        self.path_pub = self.latched(Path, 'planner/path')
        self.status_pub = self.latched(PlannerStatus, 'planner/status')
        self.costmap_pub = self.latched(OccupancyGrid, 'planner/costmap')
        self.set_path = ServiceClient(SetPath, 'navigation/set_path')
        self.start = ServiceClient(Trigger, 'navigation/start')
        self.stop = ServiceClient(Trigger, 'navigation/stop')
        sub = self.create_subscription
        sub(OccupancyGrid, '/map', self.on_map, LATCHED)
        sub(NavigationStatus, 'navigation/status', self.on_nav, LATCHED)
        sub(SectorScan, 'perception/safety_scan', self.on_scan, SENSOR)
        self.create_service(PlanPath, 'planner/plan', self.working(self.on_plan), concurrent=True)
        self.create_service(GoTo, 'planner/go_to', self.working(self.on_go_to), concurrent=True)
        self.create_service(Trigger, 'planner/cancel', self.on_cancel, concurrent=True)
        self.every(0.1, self.monitor)
        self.every(1.0 / self.config.status_hz, self.publish_status)
        self.publish_status()

    def on_map(self, msg: OccupancyGrid):
        """Keep the map (empty: the site has none); the costmap is rebuilt on the next plan."""
        if not msg.data and msg.info.width == msg.info.height == 0:
            self.grid = None
            self.log.info('the site has no map: planner/go_to refuses until one is loaded')
        else:
            try:
                self.grid = conv.map_grid(msg)
            except ValueError as error:
                self.grid = None
                self.log.error('map not usable', error=str(error))
        self.invalidate()

    def on_nav(self, msg: NavigationStatus):
        """Keep the follower state."""
        try:
            self.nav = (FollowerState(msg.state), monotonic())
        except ValueError:
            self.nav = None

    def on_scan(self, msg: SectorScan):
        """Keep the safety scan obstacles; blocked sectors make it blind (never clear)."""
        sectors = conv.scan_data(msg).points()
        blind = sectors.any_blocked or msg.header.frame_id not in ('', self.robot.frames.base)
        self.scan = (sectors.points, blind, monotonic())

    def on_config(self, old, new):
        """Rebuild the costmap with the new parameters."""
        self.session.config = new.monitor
        self.invalidate()

    def invalidate(self):
        """Forget the cached costmap."""
        self.costmap = None

    def robot_pose(self) -> Pose2D | None:
        """Return the robot pose in the map, None when unknown or older than tf_timeout_sec."""
        frames = self.robot.frames
        result = self.tf.fresh(frames.map, frames.base, self.config.tf_timeout_sec)
        return None if result is None else result.pose2d

    def live_obstacles(self, pose: Pose2D):
        """Return (obstacle points in the map, error): fresh safety scan obstacles or a problem."""
        if not self.config.use_live_obstacles:
            return None, ''
        if self.scan is None or monotonic() - self.scan[2] > self.config.scan_timeout_sec:
            return None, 'no fresh safety scan to replan with'
        points, blind, _ = self.scan
        if blind:
            return None, 'the safety scan is blind (blocked sectors)'
        return pose.apply(points), ''

    def plan(self, goal: Pose2D, live: bool, wait: bool = False, give_up=None) -> Plan:
        """Plan from the robot to goal (wait: up to pose_wait_sec for the robot pose)."""
        pose = (
            wait_for_pose(self, self.robot_pose, self.config.pose_wait_sec, give_up)
            if wait
            else self.robot_pose()
        )
        if pose is None:
            frames = self.robot.frames
            within = f' within {self.config.pose_wait_sec:g} s' if wait else ''
            return Plan(
                False, f'robot pose unknown (no fresh {frames.map} -> {frames.base}{within})'
            )
        if self.grid is None:
            return Plan(False, 'no map: load a site with a map first')
        extra, error = self.live_obstacles(pose) if live else (None, '')
        if error:
            return Plan(False, error)
        cfg, footprint = self.config.value, self.robot.footprint
        costmap = self.costmap
        if costmap is None or extra is not None:
            costmap = build_costmap(self.grid, footprint, cfg.costmap, extra)
            if extra is None:
                self.costmap = costmap
            if cfg.publish_costmap:
                stamp = self.stamp()
                self.costmap_pub.publish(conv.costmap_msg(costmap, self.robot.frames.map, stamp))
        checker = FootprintChecker(costmap, footprint, cfg.costmap.margin)
        return Planner(costmap, checker, cfg.search).plan(pose, goal)

    def send(self, plan: Plan) -> tuple[bool, str]:
        """Load the plan into the follower and start it; checks the cancel flag in between."""
        timeout = self.config.service_timeout_sec
        request = SetPathRequest(waypoints=conv.waypoints(plan.waypoints), source='planner')
        for client, req in ((self.set_path, request), (self.start, TriggerRequest())):
            if self.session.cancel_requested:
                return False, 'cancelled'
            result, error = call_and_wait(client, req, timeout)
            if result is None or not result.success:
                return False, f'{client.name}: {error or result.message}'
        stamp = self.stamp()
        self.path_pub.publish(conv.nav_path(plan.waypoints, self.robot.frames.map, stamp))
        return True, ''

    def stop_navigation(self):
        """Stop the follower (asynchronously, the outcome logged)."""

        def done(ok, message):
            if not ok:
                self.log.warning('could not stop navigation', detail=message)

        AsyncCall(self, self.stop, TriggerRequest(), self.config.service_timeout_sec, done)

    def session_cancelled(self) -> bool:
        """Return True when planner/cancel was called for the current goal."""
        return self.session.cancel_requested

    def walk(self, live: bool, reason: str = '', wait: bool = False) -> bool:
        """Plan to the session goal and send it; on failure end the goal and stop."""
        plan = self.plan(self.session.goal, live, wait, self.session_cancelled)
        if self.session.cancel_requested:
            return False
        if not plan.ok:
            self.fail(plan.message if not reason else f'{reason}, {plan.message}')
            return False
        ok, error = self.send(plan)
        if self.session.cancel_requested:
            self.stop_navigation()
            return False
        if not ok:
            self.fail(error)
            return False
        self.session.following(plan, monotonic(), f'replanned, {plan.message}' if reason else '')
        self.publish_status()
        return True

    def fail(self, reason: str):
        """End the goal as failed; the follower is stopped whatever it got."""
        if self.session.cancel_requested:
            return
        self.session.end(PlannerState.FAILED, reason)
        self.log.error('goal failed', detail=self.session.message)
        self.stop_navigation()
        self.publish_status()

    def on_plan(self, request, response):
        """Service: plan to a pose without moving."""
        goal = Pose2D(request.x, request.y, request.yaw)
        if not goal.is_finite():
            response.success, response.message = False, 'goal x, y and yaw must be finite'
            return response
        plan = self.plan(goal, live=False, wait=True)
        response.success, response.message = plan.ok, plan.message
        response.waypoints, response.length = conv.waypoints(plan.waypoints), float(plan.length)
        return response

    def on_go_to(self, request, response):
        """Service: plan to a pose, load the path into the follower and start it."""
        goal = Pose2D(request.x, request.y, request.yaw)
        if not goal.is_finite():
            response.success, response.message = False, 'goal x, y and yaw must be finite'
            return response
        self.session.begin(goal)
        self.publish_status()
        response.success = self.walk(live=False, wait=True)
        response.message = self.session.message
        return response

    def on_cancel(self, request, response):
        """Service: stop walking (never waits for a plan in progress)."""
        was = self.session.state
        self.session.cancel_requested = True
        self.stop_navigation()
        if self.session.active:
            self.session.end(PlannerState.CANCELLED, 'cancelled')
            self.publish_status()
        response.success, response.message = True, f'stopped ({was.name.lower()})'
        return response

    def working(self, handler):
        """Return a service handler that runs one at a time with the monitor."""

        def run(request, response):
            with self.work:
                return handler(request, response)

        return run

    def monitor(self):
        """Follow the follower (skipped while plan or go_to runs: it waits for them)."""
        if not self.work.acquire(blocking=False):
            return
        try:
            self.follow()
        finally:
            self.work.release()

    def follow(self):
        """Arrival, blockage, failure, being stopped."""
        if self.session.state is not PlannerState.FOLLOWING:
            return
        nav, received = self.nav or (None, -math.inf)
        step = self.session.monitor(nav, received, monotonic(), self.robot_pose())
        if step.action is Action.STOP:
            self.stop_navigation()
        elif step.action is Action.REPLAN:
            self.session.replanning(step.reason)
            self.publish_status()
            self.walk(live=True, reason=step.reason)
        if self.session.state is not PlannerState.FOLLOWING:
            self.log.info('goal ended', state=self.session.state.name, detail=self.session.message)
            self.publish_status()

    def publish_status(self):
        """Publish planner/status (latched)."""
        session = self.session
        msg = PlannerStatus(state=session.state.value, message=session.message)
        msg.header.stamp = self.stamp()
        msg.header.frame_id = self.robot.frames.map
        if session.goal is not None:
            msg.goal_x, msg.goal_y, msg.goal_yaw = session.goal.x, session.goal.y, session.goal.yaw
        msg.path_length = float(session.plan.length) if session.plan else 0.0
        msg.remaining = float(session.remaining(self.robot_pose()))
        msg.replans = session.replans
        self.status_pub.publish(msg)


def main():
    """Run planner_node."""
    run(PlannerNode, NAME)


if __name__ == '__main__':
    main()
