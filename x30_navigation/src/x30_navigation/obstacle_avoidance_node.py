"""obstacle_avoidance_node: a reactive avoidance policy on perception/sector_scan."""

from __future__ import annotations

from dataclasses import fields

from geometry_msgs.msg import Twist
from visualization_msgs.msg import MarkerArray

from x30_msgs.msg import AvoidanceCommand, SectorScan
from x30_navigation.core.avoidance import POLICIES
from x30_navigation.core.avoidance.base import travel_direction
from x30_navigation.core.config.avoidance import AvoidanceConfig
from x30_navigation.utils.conversions import avoidance_msg, scan_data
from x30_navigation.utils.markers import avoidance_markers
from x30_utils.ros import COMMAND, SENSOR, X30Node, run
from x30_utils.ros.time import monotonic

COMMAND_TIMEOUT_SEC = 0.5  # an older control/nav_cmd_vel: forward travel


NAME = 'obstacle_avoidance_node'


class ObstacleAvoidanceNode(X30Node):
    """Turn every sector scan into an AvoidanceCommand (navigation/avoidance)."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.config = self.declare_config(AvoidanceConfig)
        self.robot = self.declare_robot()  # the body width of the heading choice
        builtin = {f.name for f in fields(AvoidanceConfig)}
        self.plugin_configs = {
            name: self.declare_config(POLICIES.get(name).Config, f'{name}.')
            for name in POLICIES.names()
            if name not in builtin
        }
        self.policy = self.build()
        self.config.on_change(lambda old, new: self.rebuild())
        self.robot.on_change(lambda old, new: self.rebuild())
        for cfg in self.plugin_configs.values():
            cfg.on_change(lambda old, new: self.rebuild())
        self.avoidance_pub = self.create_publisher(
            AvoidanceCommand, 'navigation/avoidance', COMMAND
        )
        self.markers_pub = self.create_publisher(MarkerArray, 'navigation/avoidance_markers', 1)
        self.create_subscription(SectorScan, 'perception/sector_scan', self.on_scan, SENSOR)
        self.create_subscription(Twist, 'control/nav_cmd_vel', self.on_command, COMMAND)
        self.last_scan = monotonic()
        self.command = (0.0, -float('inf'))  # (vx of navigation's command, monotonic time)
        self.every(1.0, self.watchdog, steady=True)

    def watchdog(self):
        """Warn while no sector scan arrives (the follower stops on stale avoidance)."""
        silent = monotonic() - self.last_scan
        if silent > 1.0:
            self.log.warning('no perception/sector_scan', silent_sec=silent, throttle=10.0)

    def build(self):
        """Return the configured policy."""
        cfg = self.config.value
        section = cfg.policy_config()
        if cfg.policy in self.plugin_configs:
            section = self.plugin_configs[cfg.policy].value
        return POLICIES.create(cfg.policy, cfg.shared, section, robot=self.robot.value)

    def rebuild(self):
        """Apply new parameters."""
        self.policy = self.build()
        self.log.info('avoidance configured', policy=self.config.policy)

    def on_command(self, msg: Twist):
        """Keep navigation's forward speed: backward means the robot reverses."""
        self.command = (float(msg.linear.x), monotonic())

    def direction(self, full_circle: bool) -> float:
        """Return the direction of travel: pi while navigation reverses (360 deg scan only)."""
        vx, at = self.command
        if monotonic() - at > COMMAND_TIMEOUT_SEC:
            vx = 0.0
        return travel_direction(vx, full_circle)

    def on_scan(self, msg: SectorScan):
        """Run one avoidance update around the direction of travel and publish it."""
        self.last_scan = monotonic()
        scan = scan_data(msg)
        out = self.policy.update(scan, self.direction(scan.points(edges=False).full_circle))
        self.avoidance_pub.publish(avoidance_msg(out, msg.header))
        if self.config.markers:
            shared = self.config.shared
            markers = avoidance_markers(
                msg.header, out, shared.emergency_distance, shared.emergency_cone_deg
            )
            self.markers_pub.publish(markers)


def main():
    """Run obstacle_avoidance_node."""
    run(ObstacleAvoidanceNode, NAME)


if __name__ == '__main__':
    main()
