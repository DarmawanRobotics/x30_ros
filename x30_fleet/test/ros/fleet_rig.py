"""A fleet_uplink_node in this process, a fake NETRA, and fakes of the stack around it."""

from __future__ import annotations

import itertools
import json
import os
import time

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse
import yaml

from x30_fleet.fleet_uplink_node import FleetUplinkNode
from x30_fleet.tools.fake_netra import FakeNetra
from x30_msgs.msg import SiteStatus
from x30_msgs.srv import LoadPath, LoadPathResponse
from x30_utils.math import make_transform, quat_from_yaw
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

TOKEN = 'secret'
SITE = 'yard'
NAMES = itertools.count()
TRIGGERS = ('navigation/start', 'recorder/start', 'recorder/stop', 'control/estop/engage')


class Rig:
    """The node against a fake NETRA, with the stack's services and status topics faked."""

    def __init__(self, tmp_path, clock_offset: float = 0.0, **params):
        self.netra = FakeNetra(clock_offset=clock_offset).start()
        self.ns = f'/fleet_test_{next(NAMES)}'
        self.sites = tmp_path / 'map'
        (self.sites / SITE).mkdir(parents=True)
        self.calls: list[str] = []
        self.inspected: list[list[str]] = []  # the NETRA plan's points at each inspection
        self.services: list = []
        self.pubs: list = []
        for name in TRIGGERS:
            self.serve(name, Trigger, lambda _req, n=name: self.trigger(n))
        self.serve('inspection/inspect', Trigger, self.inspect)
        self.serve('navigation/load_path', LoadPath, self.load)
        self.estops = Collector(String, 'control/estop_request')
        values = {
            'server_url': self.netra.url,
            'robot_id': 'x30-test',
            'spool_dir': str(tmp_path / 'spool'),
            'sites_dir': str(self.sites),
            'telemetry_hz': 5.0,
            'upload_period_sec': 0.5,
            'clock_sync_period_sec': 1.0,
            **params,
        }
        os.environ['X30_NETRA_TOKEN'] = TOKEN
        self.node = FleetUplinkNode(param_ns=f'{self.ns}/fleet_uplink_node', overrides=values)
        assert connected(self.estops)
        assert wait_for(lambda: self.netra.of_type('hello'), 10.0), 'no hello from the robot'

    # ------------------------------------------------------------ the stack
    def serve(self, name: str, srv_type, handler):
        """Offer a fake service of the stack."""
        self.services.append(rospy.Service(name, srv_type, handler))

    def trigger(self, name: str) -> TriggerResponse:
        self.calls.append(name)
        return TriggerResponse(success=True, message=f'{name} ok')

    def load(self, request) -> LoadPathResponse:
        self.calls.append(f'load {request.path}')
        return LoadPathResponse(success=True, message='loaded')

    def inspect(self, _request) -> TriggerResponse:
        with open(self.plan_file(), encoding='utf-8') as f:
            plan = yaml.safe_load(f)
        self.inspected.append([p['name'] for p in plan['points']])
        return TriggerResponse(success=True, message='inspected')

    def plan_file(self) -> str:
        return str(self.sites / SITE / 'netra' / 'inspection.yaml')

    def publisher(self, topic: str, msg_type):
        """Return a latched publisher of a status topic."""
        pub = rospy.Publisher(topic, msg_type, queue_size=10, latch=True)
        self.pubs.append(pub)
        return pub

    def site(self, name: str = SITE):
        """Load a site (site/status)."""
        self.publisher('site/status', SiteStatus).publish(SiteStatus(site=name))
        assert wait_for(lambda: self.node.telemetry.site == name, 5.0)

    def pose(self, x: float, y: float, yaw: float):
        """Put the robot at x, y, yaw in the map frame."""
        matrix = make_transform((x, y, 0.0), quat_from_yaw(yaw))
        self.node.tf_out.send_static([(matrix, 'map', 'base_link')])
        assert wait_for(lambda: self.node.pose() is not None, 5.0)

    # ------------------------------------------------------------ NETRA
    def command(self, cid: int, name: str, timeout: float = 10.0, **args) -> dict:
        """Send a command from NETRA and return its acknowledgement."""
        assert self.netra.command(cid, name, **args)
        assert wait_for(lambda: self.ack(cid) is not None, timeout), f'no ack of {name}'
        return self.ack(cid)

    def ack(self, cid: int):
        return next((m for m in self.netra.of_type('ack') if m.get('id') == cid), None)

    def uploads(self, path: str) -> list[bytes]:
        with self.netra.lock:
            return [body for p, body in self.netra.uploads if p == path]

    def telemetry(self, check, timeout: float = 5.0) -> dict | None:
        """Return the first telemetry received from now on that passes check."""
        start = len(self.netra.of_type('telemetry'))
        found: list = []

        def seen() -> bool:
            found.extend(m for m in self.netra.of_type('telemetry')[start:] if check(m))
            return bool(found)

        return found[0] if wait_for(seen, timeout) else None

    def close(self):
        """Tear everything down."""
        self.node.run_shutdown_hooks()
        self.node.destroy_node()
        for service in self.services:
            service.shutdown()
        for pub in self.pubs:
            pub.unregister()
        self.estops.close()
        self.netra.close()
        delete_params(self.ns)
        os.environ.pop('X30_NETRA_TOKEN', None)
        time.sleep(0.2)  # the services go away


def meta_of(body: bytes) -> dict:
    """Return the meta JSON of a multipart upload."""
    start = body.index(b'name="meta"')
    start = body.index(b'\r\n\r\n', start) + 4
    return json.loads(body[start : body.index(b'\r\n--', start)])
