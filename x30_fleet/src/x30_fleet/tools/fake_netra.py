"""A stand-in for the NETRA backend (API.md), for the simulator, tests and bench checks."""

# rosrun x30_fleet fake_netra --port 8765 --log /tmp/netra.jsonl
# roslaunch x30_bringup bringup.launch sim:=true fleet:=true netra_url:=http://127.0.0.1:8765
# It accepts the robot's WebSocket (/api/v1/robot/ws), records every message, answers the
# uploads (routes, substation inspections), serves route files and inspection plans to
# start_patrol. Standard library only, one robot at a time; not a production server.

from __future__ import annotations

import argparse
import base64
import contextlib
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import struct
import threading
import time

from x30_fleet.core.clock import answer

GUID = '258EAFA5-E914-47DA-95CA-C5AB0DC85B11'
WS_PATH = '/api/v1/robot/ws'
ROUTE_PREFIX = '/api/v1/robot/routes/'


def accept_key(key: str) -> str:
    """Return Sec-WebSocket-Accept of a client key (RFC 6455)."""
    return base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()


def encode_frame(text: str) -> bytes:
    """Return one unmasked text frame (server to client)."""
    data = text.encode()
    if len(data) < 126:
        header = struct.pack('!BB', 0x81, len(data))
    elif len(data) < 1 << 16:
        header = struct.pack('!BBH', 0x81, 126, len(data))
    else:
        header = struct.pack('!BBQ', 0x81, 127, len(data))
    return header + data


def read_frame(stream) -> tuple[int, bytes] | None:
    """Read one (masked) client frame; return (opcode, payload), None when closed."""
    head = stream.read(2)
    if len(head) < 2:
        return None
    opcode, length = head[0] & 0x0F, head[1] & 0x7F
    if length == 126:
        length = struct.unpack('!H', stream.read(2))[0]
    elif length == 127:
        length = struct.unpack('!Q', stream.read(8))[0]
    mask = stream.read(4) if head[1] & 0x80 else b'\0\0\0\0'
    payload = bytearray(stream.read(length))
    for i in range(len(payload)):
        payload[i] ^= mask[i % 4]
    return opcode, bytes(payload)


class FakeNetra:
    """The server: received messages, uploads, routes to serve, commands to send."""

    def __init__(self, port: int = 0, log_path: str = '', clock_offset: float = 0.0):
        self.clock_offset = clock_offset  # this server's clock minus the computer's (s)
        self.messages: list[dict] = []
        self.uploads: list[tuple[str, bytes]] = []
        self.routes: dict[str, bytes] = {}
        self.plans: dict[str, dict] = {}  # route id -> its substation inspection plan
        self.headers: dict = {}
        self.log_path = log_path
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()  # one frame at a time to the robot
        self.client = None  # the robot's socket file while connected
        self.server = ThreadingHTTPServer(('127.0.0.1', port), self.handler())
        self.server.daemon_threads = True
        self.port = self.server.server_port
        self.url = f'http://127.0.0.1:{self.port}'
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def start(self) -> FakeNetra:
        """Serve in the background."""
        self.thread.start()
        return self

    def close(self):
        """Stop serving."""
        self.server.shutdown()
        self.server.server_close()

    def record(self, message: dict):
        """Keep (and log) a message from the robot."""
        with self.lock:
            self.messages.append(message)
        if self.log_path:
            with open(self.log_path, 'a', encoding='utf-8') as f:
                f.write(json.dumps(message) + '\n')

    def of_type(self, kind: str) -> list[dict]:
        """Return the received messages of one type."""
        with self.lock:
            return [m for m in self.messages if m.get('type') == kind]

    def command(self, cid, name: str, **args) -> bool:
        """Send a command to the connected robot; False when none is connected."""
        message = json.dumps({'type': 'command', 'id': cid, 'name': name, 'args': args})
        with self.lock:
            client = self.client
        if client is None:
            return False
        return self.write(client, message)

    def write(self, client, text: str) -> bool:
        """Send one text frame to the robot."""
        with self.write_lock, contextlib.suppress(OSError):
            client.write(encode_frame(text))
            client.flush()
            return True
        return False

    def now(self) -> float:
        """Return this server's clock."""
        return time.time() + self.clock_offset

    def handler(self):
        """Return the request handler class bound to this server."""
        return type('Handler', (NetraHandler,), {'fake': self})


class NetraHandler(BaseHTTPRequestHandler):
    """WebSocket upgrade, uploads and route files of one FakeNetra (`fake`)."""

    fake: FakeNetra

    def log_message(self, *args):
        """Keep the console quiet."""

    def reply(self, code: int, body: bytes = b'{}', kind: str = 'application/json'):
        """Answer with a body."""
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        """Upgrade to the WebSocket, or serve a route file."""
        if self.path == WS_PATH and 'Sec-WebSocket-Key' in self.headers:
            self.websocket()
        elif self.path.startswith(ROUTE_PREFIX) and self.path.endswith('/file'):
            route_id = self.path[len(ROUTE_PREFIX) : -len('/file')]
            data = self.fake.routes.get(route_id)
            self.reply(200, data, 'text/csv') if data else self.reply(404)
        elif self.path.startswith(ROUTE_PREFIX) and self.path.endswith('/inspection'):
            route_id = self.path[len(ROUTE_PREFIX) : -len('/inspection')]
            plan = self.fake.plans.get(route_id, {'route_id': route_id, 'points': []})
            self.reply(200, json.dumps(plan).encode())
        else:
            self.reply(404)

    def do_POST(self):
        """Accept an upload."""
        body = self.rfile.read(int(self.headers.get('Content-Length', 0)))
        with self.fake.lock:
            self.fake.uploads.append((self.path, body))
            number = len(self.fake.uploads)
        self.reply(201, json.dumps({'id': number, 'created': True}).encode())

    def websocket(self):
        """Hand-shake and serve the robot's WebSocket until it closes."""
        self.fake.headers = dict(self.headers)
        self.send_response(101)
        self.send_header('Upgrade', 'websocket')
        self.send_header('Connection', 'Upgrade')
        self.send_header('Sec-WebSocket-Accept', accept_key(self.headers['Sec-WebSocket-Key']))
        self.end_headers()
        self.wfile.flush()
        with self.fake.lock:
            self.fake.client = self.wfile
        try:
            self.frames()
        finally:
            with self.fake.lock:
                self.fake.client = None
            self.close_connection = True

    def frames(self):
        """Record the robot's text frames, answer pings."""
        while True:
            frame = read_frame(self.rfile)
            if frame is None or frame[0] == 0x8:  # closed
                return
            if frame[0] == 0x9:  # ping: pong
                self.wfile.write(b'\x8a\x00')
                self.wfile.flush()
            elif frame[0] == 0x1:
                received = self.fake.now()
                with contextlib.suppress(ValueError):
                    message = json.loads(frame[1].decode())
                    self.fake.record(message)
                    if isinstance(message, dict) and message.get('type') == 'time_sync':
                        reply = answer(message, received, self.fake.now())
                        self.fake.write(self.wfile, json.dumps(reply))


def main(args=None):
    """Run the server until Ctrl-C (--log keeps what the robot sent)."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--log', default='', help='append every message as a JSON line')
    parser.add_argument('--clock-offset', type=float, default=0.0, help='server clock skew (s)')
    options, _ = parser.parse_known_args(args)
    server = FakeNetra(options.port, options.log, options.clock_offset)
    with contextlib.suppress(KeyboardInterrupt):
        server.server.serve_forever()
    server.server.server_close()


if __name__ == '__main__':
    main()
