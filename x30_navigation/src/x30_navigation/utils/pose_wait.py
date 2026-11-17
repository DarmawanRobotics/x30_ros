"""Wait (bounded) for the robot pose: TF can lag right after LOCALIZED on a loaded machine."""

from __future__ import annotations

import time

import rospy

from x30_utils.ros.time import monotonic

POLL_SEC = 0.05


def wait_for_pose(node, get_pose, wait_sec: float, give_up=None):
    """Return get_pose(), polling for up to wait_sec while it is None (blocks the thread)."""
    began = monotonic()
    deadline = began + wait_sec
    pose = get_pose()
    while (
        pose is None and monotonic() < deadline and not rospy.is_shutdown() and not node.destroyed
    ):
        if give_up is not None and give_up():
            return None
        time.sleep(POLL_SEC)
        pose = get_pose()
    waited = monotonic() - began
    if pose is not None and waited > POLL_SEC:
        node.log.info('robot pose known after waiting', waited_sec=round(waited, 2))
    return pose
