#!/usr/bin/env python3
"""x30_check: the ROS rows of tools/x30_host.sh check (standard library only, no ROS needed).

The master and every node's address from here; the other host's clock against this one's, from
the stamps of a topic it publishes; on the perception host the inspection cameras' ports. Exit 1
when the master, a node or a camera cannot be reached.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import socket
import struct
import sys
import time
from urllib.parse import urlparse
import xml.etree.ElementTree as ET
import xmlrpc.client

CALLER = '/x30_host_check'
TIMEOUT = 2.0
IN_STEP = 0.1  # s between a stamp there and its arrival here: the clocks agree, transport included
FIRST = (
    '/x30/odom',
    '/imu/data',
    '/hikmicro/ptz/state',
    '/hikmicro/joint_states',
    '/hertzino/status',
)
LARGE = (
    'sensor_msgs/Image', 'sensor_msgs/CompressedImage', 'sensor_msgs/PointCloud2',
    'sensor_msgs/PointCloud', 'sensor_msgs/LaserScan', 'livox_ros_driver/CustomMsg',
    'livox_ros_driver2/CustomMsg', 'hertzino_msgs/RawFrame', 'hertzino_msgs/Spectrum',
    'nav_msgs/OccupancyGrid', 'nav_msgs/Path', 'visualization_msgs/MarkerArray',
)  # fmt: skip
# the inspection cameras (perception_host.launch's arguments) and their drivers' ports
CAMERAS = {'hikmicro': (('ISAPI', 80), ('RTSP', 554)), 'hertzino': (('HTTP', 80), ('RTSP', 443))}
LAUNCH = Path(__file__).resolve().parents[1] / 'x30_bringup' / 'launch' / 'perception_host.launch'


def row(name: str, status: str, text: str):
    """Print one row as x30_host.sh does."""
    print(f'{name:<8} {status:<5} {text}', flush=True)


def proxy(uri: str) -> xmlrpc.client.ServerProxy:
    """Return an XML-RPC proxy (the socket timeout is set for the whole run)."""
    return xmlrpc.client.ServerProxy(uri)


def tcpros_header(fields: dict) -> bytes:
    """Return a TCPROS connection header."""
    items = [f'{key}={value}'.encode() for key, value in fields.items()]
    body = b''.join(struct.pack('<I', len(item)) + item for item in items)
    return struct.pack('<I', len(body)) + body


def read_exact(sock: socket.socket, size: int) -> bytes:
    """Return size bytes from the socket (OSError when it closes first)."""
    data = b''
    while len(data) < size:
        chunk = sock.recv(size - len(data))
        if not chunk:
            raise OSError('connection closed')
        data += chunk
    return data


def read_header(sock: socket.socket) -> dict:
    """Return the fields of a TCPROS header."""
    (size,) = struct.unpack('<I', read_exact(sock, 4))
    body, fields, at = read_exact(sock, size), {}, 0
    while at + 4 <= len(body):
        (length,) = struct.unpack_from('<I', body, at)
        key, _, value = body[at + 4 : at + 4 + length].decode(errors='replace').partition('=')
        fields[key] = value
        at += 4 + length
    return fields


def stamped(definition: str) -> bool:
    """Return True when a message definition starts with a std_msgs/Header."""
    for line in definition.splitlines():
        words = line.split('#', 1)[0].split()
        if words:
            return words[0] in ('Header', 'std_msgs/Header')
    return False


def sample(uri: str, topic: str, kind: str, seconds: float) -> list:
    """Return arrival here minus stamp there (s) of a publisher's messages, [] when unusable."""
    code, _, protocol = proxy(uri).requestTopic(CALLER, topic, [['TCPROS']])
    if code != 1 or len(protocol) < 3 or protocol[0] != 'TCPROS':
        return []
    with socket.create_connection((protocol[1], int(protocol[2])), TIMEOUT) as sock:
        fields = {'callerid': CALLER, 'topic': topic, 'md5sum': '*', 'type': kind}
        sock.sendall(tcpros_header({**fields, 'tcp_nodelay': '1'}))
        reply = read_header(sock)
        if 'error' in reply or reply.get('latching') == '1':  # a latched one may be old
            return []
        if not stamped(reply.get('message_definition', '')):
            return []
        offsets, end = [], time.monotonic() + seconds
        while time.monotonic() < end and len(offsets) < 50:
            sock.settimeout(max(end - time.monotonic(), 0.01))
            try:
                (size,) = struct.unpack('<I', read_exact(sock, 4))
                payload = read_exact(sock, size)
            except OSError:  # the time is up, or the publisher went
                break
            arrived = time.time()
            if size >= 12:
                _, secs, nsecs = struct.unpack_from('<3I', payload)
                if secs:
                    offsets.append(arrived - (secs + nsecs * 1e-9))
        return offsets


def other_publishers(master, state, other: str) -> list:
    """Return (topic, node uri, type) of what the other host publishes, small and likely first."""
    types = dict(master.getTopicTypes(CALLER)[2])
    uris, out = {}, []
    for topic, nodes in state[0]:
        if types.get(topic, '') in LARGE:
            continue
        for node in nodes:
            if node not in uris:
                code, _, uri = master.lookupNode(CALLER, node)
                uris[node] = uri if code == 1 else ''
            host = urlparse(uris[node]).hostname or ''
            try:
                address = socket.gethostbyname(host) if host else ''
            except OSError:
                address = ''
            if other in (host, address):
                out.append((topic, uris[node], types.get(topic, '*')))
                break
    out.sort(key=lambda item: (item[0] not in FIRST, item[0]))
    return out


def clock(master, state, other: str):
    """Print how the other host's stamps arrive here."""
    for topic, uri, kind in other_publishers(master, state, other)[:6]:
        try:
            offsets = sample(uri, topic, kind, 1.5)
        except (OSError, xmlrpc.client.Error):
            continue
        if len(offsets) < 3:
            continue
        offset = min(offsets)  # the least delayed: the clocks' difference, transport included
        if -IN_STEP <= offset <= IN_STEP:
            row('clock', 'ok', f'in step with {other} (its {topic}: {offset * 1000:+.0f} ms)')
        elif offset < 0.0:
            text = f'{other} is {-offset:.2f} s ahead of this host (its {topic})'
            row('clock', 'WARN', f'{text}: one clock for both (tools/README.md)')
        else:
            text = f'{other} is {offset:.2f} s behind this host, or stamps its {topic} late'
            row('clock', 'WARN', f'{text}: one clock for both (tools/README.md)')
        return
    row('clock', '-', f'nothing stamped from {other} to compare yet (its stack running?)')


def nodes(master, state) -> int:
    """Print the nodes this host cannot reach; return how many."""
    names = sorted({node for kind in state for _, members in kind for node in members})
    unreachable = 0
    for node in names:
        code, _, uri = master.lookupNode(CALLER, node)
        if code != 1:
            continue
        address = urlparse(uri)
        try:
            socket.create_connection((address.hostname, address.port), TIMEOUT).close()
        except OSError as error:
            unreachable += 1
            row('node', 'MISS', f'{node} at {address.hostname}:{address.port}: {error}')
    if unreachable:
        print('         a name that does not resolve: "<ip> <name>" in /etc/hosts, or ROS_IP for')
        print('         that node where it starts')
    status = 'ok' if not unreachable else 'MISS'
    row('nodes', status, f'{len(names) - unreachable} of {len(names)} reachable from here')
    return unreachable


def closed(host: str, port: int) -> str:
    """Return why a TCP port does not answer ('' when it does)."""
    try:
        socket.create_connection((host, port), TIMEOUT).close()
    except OSError as error:
        return str(error) or type(error).__name__
    return ''


def cameras(launch_args: str) -> int:
    """Print whether each started camera's ports answer; return how many do not."""
    try:
        values = {a.get('name'): a.get('default', '') for a in ET.parse(LAUNCH).iter('arg')}
    except (OSError, ET.ParseError):
        values = {}
    for word in shlex.split(launch_args):
        key, given, value = word.partition(':=')
        if given:
            values[key] = value
    missing = 0
    for name, ports in CAMERAS.items():
        host = values.get(f'{name}_host', '')
        if values.get(name, 'true').lower() in ('false', '0') or not host:
            row(name, '-', 'not started here (launch arguments)')
            continue
        shut = [(label, port, closed(host, port)) for label, port in ports]
        if not any(why for _, _, why in shut):
            row(
                name,
                'ok',
                f'{host}: {", ".join(f"{label} {port}" for label, port, _ in shut)} open',
            )
            continue
        missing += 1
        why = '; '.join(f'{label} {port}: {reason}' for label, port, reason in shut if reason)
        net = host.rsplit('.', 1)[0]
        row(name, 'MISS', f'{host}: {why} (its cable and address, this host on {net}.x)')
    return missing


def main(argv: list | None = None) -> int:
    """Print the rows; 1 when something cannot be reached."""
    parser = argparse.ArgumentParser(prog='x30_check', description=__doc__.splitlines()[0])
    parser.add_argument('--master', required=True)
    parser.add_argument('--other', default='', help="the other host's address")
    parser.add_argument('--cameras', action='store_true', help='check the inspection cameras')
    parser.add_argument('--launch-args', default='', help="the host's launch arguments")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    socket.setdefaulttimeout(TIMEOUT)
    master, problems = proxy(args.master), 0
    try:
        _, _, state = master.getSystemState(CALLER)
    except (OSError, xmlrpc.client.Error) as error:
        row('master', 'MISS', f'{args.master}: {error}')
        state, problems = None, 1
    if state is not None:
        row('master', 'ok', args.master)
        problems += nodes(master, state)
        if args.other:
            clock(master, state, args.other)
    if args.cameras:  # with or without the master
        problems += cameras(args.launch_args)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
