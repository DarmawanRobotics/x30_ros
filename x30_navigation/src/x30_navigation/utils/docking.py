"""The ROS side of docking: tag and battery inputs, status and goal messages, the hook calls."""

from __future__ import annotations

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from std_srvs.srv import Trigger, TriggerRequest

from x30_msgs.msg import DockingStatus
from x30_navigation.core.docking.geometry import tag_pose
from x30_navigation.core.docking.seated import HOOK_GROUPS
from x30_navigation.core.docking.state import DockReport, PoseInput
from x30_navigation.utils.clients import AsyncCall, ServiceClient, TriggerHooks
from x30_utils.math import Pose2D
from x30_utils.ros.convert import pose2d_to_pose

CHARGING = (
    BatteryState.POWER_SUPPLY_STATUS_CHARGING,
    BatteryState.POWER_SUPPLY_STATUS_FULL,
)


def tag_input(tf, odom: str, frame: str, robot: PoseInput | None, axis: str) -> PoseInput | None:
    """Return the dock tag in odom (yaw: its normal toward the robot) with its age, or None."""
    if robot is None:
        return None
    result = tf.lookup(odom, frame)  # through the camera: odom -> base -> camera -> tag
    if result is None:
        return None
    pose = tag_pose(result.matrix, (robot.pose.x, robot.pose.y), axis)
    return None if pose is None else PoseInput(pose, result.age)


def is_charging(msg: BatteryState) -> bool:
    """Return True when the battery reports charging or full."""
    return int(msg.power_supply_status) in CHARGING


def status_msg(report: DockReport, frame: str, stamp) -> DockingStatus:
    """Return a DockingStatus."""
    e = report.errors
    msg = DockingStatus(
        state=report.state.value,
        pause_reasons=list(report.pause_reasons),
        tag_visible=bool(report.tag_visible),
        tag_age=float(report.tag_age),
        remaining=float(e.remaining),
        lateral=float(e.lateral),
        yaw_error=float(e.yaw),
        attempt=int(report.attempt),
        charging=bool(report.charging),
        message=report.message,
    )
    msg.header.frame_id, msg.header.stamp = frame, stamp
    return msg


def goal_msg(goal: Pose2D, frame: str, stamp) -> PoseStamped:
    """Return the docked pose as a PoseStamped (RViz)."""
    msg = PoseStamped(pose=pose2d_to_pose(goal))
    msg.header.frame_id, msg.header.stamp = frame, stamp
    return msg


class DockHooks:
    """The robot's Trigger hooks per group, and the navigation stop when docking starts."""

    def __init__(self, node, config):
        self.node, self.config = node, config
        self.groups = {n: TriggerHooks(node, getattr(config, n)) for n in HOOK_GROUPS}
        name = config.navigation_stop
        self.nav_stop = None
        if name:
            self.nav_stop = ServiceClient(Trigger, name)

    def call(self, group: str, then):
        """Call every hook of group; then(ok, message) once all answered (or timed out)."""

        def done(ok, results):
            failed = [f'{name}: {message}' for name, good, message in results if not good]
            names = ', '.join(name for name, _, _ in results) or 'none configured'
            then(ok, '; '.join(failed) if failed else f'{group} hooks done ({names})')

        self.groups[group].call_all(self.config.timeout_sec, done)

    def stop_navigation(self):
        """Stop the path follower so that only docking drives (best effort, logged)."""
        if self.nav_stop is None:
            return

        def done(ok, message):
            level = self.node.log.info if ok else self.node.log.warning
            level('navigation stop before docking', ok=ok, detail=message)

        AsyncCall(self.node, self.nav_stop, TriggerRequest(), self.config.timeout_sec, done)
