"""Everything path_follower_node publishes."""

from __future__ import annotations

from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray

from x30_msgs.msg import NavigationStatus, StopPointEvent
from x30_navigation.core.controllers.base import Command
from x30_navigation.utils import conversions as conv
from x30_navigation.utils.markers import follower_markers, route_markers
from x30_utils.ros import COMMAND, EVENTS


class FollowerOutputs:
    """Publishers of the follower: command, status, stop point events and markers."""

    def __init__(self, node):
        self.node = node
        self.cmd = node.create_publisher(Twist, 'control/nav_cmd_vel', COMMAND)
        self.status = node.latched(NavigationStatus, 'navigation/status')
        self.stop_points = node.create_publisher(StopPointEvent, 'navigation/stop_point', EVENTS)
        self.markers = node.create_publisher(MarkerArray, 'navigation/markers', 1)
        self.route = node.latched(MarkerArray, 'navigation/route_markers')
        self.route_key = None

    def header(self) -> Header:
        """Return a header in the map frame, stamped now."""
        stamp = self.node.stamp()
        return Header(frame_id=self.node.robot.frames.map, stamp=stamp)

    def command(self, command: Command):
        """Publish a velocity command."""
        self.cmd.publish(conv.twist(command))

    def decision(self, decision, follower, markers: bool):
        """Publish a decision: command, status, events, markers."""
        if decision.command is not None:
            self.command(decision.command)
        warning = follower.controller.warning
        if warning:  # e.g. NeuPAN's solves longer than its step
            self.node.log.warning(warning, throttle=30.0, key='controller_warning')
        header = self.header()
        self.status.publish(conv.status_msg(decision.report, header.frame_id, header.stamp))
        self.events(decision.events)
        if markers:
            report = decision.report
            drawing = follower_markers(header, report.state, report.message, decision.debug)
            self.markers.publish(drawing)
            self.route_markers(header, follower)

    def events(self, events):
        """Publish stop point events."""
        header = self.header()
        for event in events:
            self.stop_points.publish(conv.stop_event_msg(event, header.frame_id, header.stamp))
            self.node.log.event(
                'stop_point', kind=event.kind.name, index=event.index, detail=event.message
            )

    def route_markers(self, header: Header, follower):
        """Publish the latched route drawing when the route or the next stop changed."""
        stops = follower.stops
        if stops is None:
            return
        key = (id(stops.route), stops.position)
        if key != self.route_key:
            self.route_key = key
            self.route.publish(route_markers(header, stops.route, stops.colors()))
