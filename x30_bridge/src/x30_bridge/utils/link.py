"""The UDP socket of the gateway: a reader thread, a bounded inbox, chunked streams."""

from __future__ import annotations

from collections import deque
import contextlib
import select
import socket
import threading

from x30_bridge.core import protocol

INBOX = 256  # datagrams waiting at most (a flood cannot starve the node)
BURST = 64  # datagrams read per wake-up
SEND_BUFFER = 1 << 20  # room for a camera frame sent as one burst (capped by wmem_max)


class UdpLink:
    """Receives on a thread (calling wake per burst) and sends to one client."""

    def __init__(self, port: int, wake, host: str = '0.0.0.0'):
        self.wake = wake  # called on the reader thread after datagrams arrived
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        with contextlib.suppress(OSError):
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, SEND_BUFFER)
        try:
            self.sock.bind((host, port))
        except OSError:
            self.sock.close()
            raise
        self.sock.setblocking(False)
        self.inbox: deque = deque(maxlen=INBOX)
        self.frame_ids: dict = {}
        self.send_lock = threading.Lock()  # acks come from the command threads too
        self.running = True
        self.reader = threading.Thread(target=self.read, name='teleop udp', daemon=True)
        self.reader.start()

    def close(self):
        """Stop the reader and free the port at once."""
        self.running = False
        self.reader.join(timeout=1.0)
        self.sock.close()

    def read(self):
        """Receive datagrams into the inbox until closed."""
        while self.running:
            try:
                ready, _, _ = select.select([self.sock], [], [], 0.5)
            except (OSError, ValueError):  # socket closed under us
                return
            if not ready:
                continue
            for _ in range(BURST):
                try:
                    self.inbox.append(self.sock.recvfrom(65535))
                except (BlockingIOError, InterruptedError):
                    break
                except OSError:
                    if not self.running:
                        return
                    break
            if self.running:
                self.wake()

    def take(self) -> list:
        """Return the received (data, addr) pairs and empty the inbox."""
        return [self.inbox.popleft() for _ in range(len(self.inbox))]

    def send(self, data: bytes, addr) -> bool:
        """Send one datagram; False when it was dropped (no address, socket busy)."""
        if addr is None:
            return False
        try:
            with self.send_lock:
                self.sock.sendto(data, addr)
        except OSError:
            return False
        return True

    def send_json(self, message: dict, addr):
        """Send a JSON message."""
        self.send(protocol.encode_json(message), addr)

    def send_stream(self, stream: int, payload: bytes, addr):
        """Send a chunked payload; stop at a dropped chunk (the frame is lost anyway)."""
        if addr is None:
            return
        self.frame_ids[stream] = self.frame_ids.get(stream, 0) + 1
        for datagram in protocol.chunk(stream, self.frame_ids[stream], payload):
            if not self.send(datagram, addr):
                return
