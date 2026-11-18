"""One action goal at a time: sent, waited for within a deadline, cancelled when asked."""

from __future__ import annotations

import time

import rospy

SERVER_WAIT_SEC = 2.0  # the driver's action server must answer this soon
POLL_SEC = 0.2


def run_goal(client, goal, timeout_sec: float, cancelled=lambda: False) -> tuple[object, str]:
    """Send goal and wait for its result: (result, '') or (None, why); never raises."""
    try:
        return _run(client, goal, timeout_sec, cancelled)
    except rospy.ROSException as error:  # the client was closed: the node shuts down
        return None, f'failed: {error}'


def _run(client, goal, timeout_sec: float, cancelled) -> tuple[object, str]:
    if not client.wait_for_server(rospy.Duration(SERVER_WAIT_SEC)):
        return None, 'not available'
    client.send_goal(goal)
    deadline = time.monotonic() + timeout_sec
    while not client.wait_for_result(rospy.Duration(POLL_SEC)):
        stop = cancelled() or rospy.is_shutdown()
        if stop or time.monotonic() > deadline:
            client.cancel_goal()
            return None, 'cancelled' if stop else f'no result within {timeout_sec:.0f} s'
    result = client.get_result()
    return (result, '') if result is not None else (None, 'ended without a result')
