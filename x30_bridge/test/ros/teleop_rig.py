"""A teleop_udp_node in this process with UDP clients and fakes of the stack's services."""

from __future__ import annotations

import itertools
import json
import socket
import time

from geometry_msgs.msg import Twist
import rospy
from std_msgs.msg import String

from x30_bridge.teleop_udp_node import TeleopUdpNode
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

TOKEN = 'secret'
NAMES = itertools.count()


def free_port() -> int:
    """Return a UDP port nobody listens on."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


class Rig:
    """The node, listeners on the stick and the e-stop request, UDP clients."""

    def __init__(self, sites_dir, **params):
        self.ns = f'/teleop_test_{next(NAMES)}'
        self.port = free_port()
        values = {
            'port': self.port,
            'sites_dir': str(sites_dir),
            'telemetry_hz': 20.0,
            'client_timeout_sec': 10.0,  # the tests send no keep-alive
            'navigation_node': f'{self.ns}/path_follower_node',
            **params,
        }
        self.node = TeleopUdpNode(param_ns=f'{self.ns}/teleop_udp_node', overrides=values)
        self.twists = Collector(Twist, 'control/remote/handheld/cmd_vel')
        self.estops = Collector(String, 'control/estop_request')
        assert connected(self.twists, self.estops)
        self.clients: list[socket.socket] = []
        self.services: list = []
        self.pubs: list = []

    def client(self, ip: str = '127.0.0.1') -> socket.socket:
        """Open a client socket bound to ip (127.0.0.2 counts as another device)."""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((ip, 0))
        sock.setblocking(False)  # read what is there: the states come at 20 Hz
        self.clients.append(sock)
        return sock

    def send(self, sock, message, raw: bool = False, settle: float = 0.15):
        """Send one datagram to the node and give it time to answer."""
        data = message if raw else json.dumps({'k': TOKEN, **message}).encode()
        sock.sendto(data, ('127.0.0.1', self.port))
        time.sleep(settle)

    def received(self, sock) -> list:
        """Return the JSON messages waiting on a client socket."""
        return [json.loads(d) for d in self.datagrams(sock) if d[:1] == b'{']

    def binary(self, sock) -> list:
        """Return the binary stream datagrams waiting on a client socket."""
        return [d for d in self.datagrams(sock) if d[:1] != b'{']

    @staticmethod
    def datagrams(sock) -> list:
        """Return every datagram waiting on a client socket."""
        out = []
        while True:
            try:
                data, _ = sock.recvfrom(65535)
            except BlockingIOError:
                return out
            out.append(data)

    def acks(self, sock, timeout: float = 3.0, until=None) -> dict:
        """Collect acknowledgements by command id (until(acks) holds or timeout)."""
        acks: dict = {}
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            for m in self.received(sock):
                if m['t'] == 'ack':
                    acks.setdefault(m['id'], []).append(m)
            if until is not None and until(acks):
                break
            time.sleep(0.02)
        return acks

    def serve(self, name: str, srv_type, handler):
        """Offer a fake service of the stack."""
        self.services.append(rospy.Service(name, srv_type, handler))
        assert wait_for(lambda: _available(name), 5.0)

    def publisher(self, topic: str, msg_type):
        """Return a latched publisher of a status topic."""
        pub = rospy.Publisher(topic, msg_type, queue_size=1, latch=True)
        self.pubs.append(pub)
        return pub

    def close(self):
        """Tear everything down."""
        for sock in self.clients:
            sock.close()
        for service in self.services:
            service.shutdown()
        for pub in self.pubs:
            pub.unregister()
        self.twists.close()
        self.estops.close()
        self.node.run_shutdown_hooks()
        self.node.destroy_node()
        delete_params(self.ns)


def _available(name: str) -> bool:
    try:
        rospy.wait_for_service(name, 0.1)
    except rospy.ROSException:
        return False
    return True
