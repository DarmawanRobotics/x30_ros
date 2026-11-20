"""Check once that the follower guard is at least as cautious as the control mux field."""

from __future__ import annotations

import rospy

FIELD_PARAMETERS = (
    'field/lateral_margin',
    'field/human_speed',
    'field/margin',
    'field/yaw_margin',
)
POLL_SEC = 5.0


class ControlFieldCheck:
    """Read the control mux field from the parameter server; log every mismatch with the guard."""

    def __init__(
        self,
        node,
        control_node: str,
        check,
        problem: str = 'guard less cautious than the control field',
        fine: str = 'guard covers the control field',
    ):
        self.node = node
        self.check = check
        self.problem, self.fine = problem, fine
        self.timer = None
        if control_node:
            # X30Node writes every node's effective parameters back to the server
            self.names = [f'{control_node.rstrip("/")}/{name}' for name in FIELD_PARAMETERS]
            self.timer = node.every(POLL_SEC, self.poll, steady=True)

    def poll(self):
        """Compare once the mux has put its field on the server; then stop polling."""
        values = [_number(_get(name)) for name in self.names]
        if values[0] is None:
            return
        self.timer.shutdown()
        problems = self.check(values[0], *(value or 0.0 for value in values[1:]))
        for problem in problems:
            self.node.log.error(self.problem, detail=problem)
        if not problems:
            self.node.log.info(self.fine, lateral_margin=values[0])


def _get(name: str):
    try:
        return rospy.get_param(name, None)
    except (rospy.ROSException, OSError):
        return None


def _number(value) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return float(value)
