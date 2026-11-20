"""The link to NETRA: a WebSocket and the HTTP uploader of the spool, each on its own thread."""

# The WebSocket reconnects and keeps an outbox while offline; NETRA's commands are
# handed to the ROS thread.

from __future__ import annotations

from collections import deque
import contextlib
import json
import os
import queue
import re
import threading
import time

from x30_fleet.core.clock import ClockOffset
from x30_fleet.core.spool import Spool

# spooled kind -> NETRA's endpoint; the files go as fields of their name (route.csv; thermal,
# visual and acoustic of an inspection)
UPLOADS = {'route': '/api/v1/robot/routes', 'inspection': '/api/v1/robot/inspections'}
TYPES = {'.jpg': 'image/jpeg', '.png': 'image/png', '.csv': 'text/csv'}
OUTBOX = 500  # durable messages kept while offline
BACKOFF_MAX_SEC = 30.0


class NetraLink:
    """WebSocket and uploads to the command center (websocket-client and requests)."""

    def __init__(self, url: str, token: str, robot_id: str, verify: bool, spool: Spool, log):
        self.base_url = url.rstrip('/')
        self.token, self.robot_id, self.verify = token, robot_id, verify
        self.spool = spool
        self.log = log
        self.incoming: queue.Queue = queue.Queue()  # commands, for the ROS thread
        self.outbox: deque = deque(maxlen=OUTBOX)
        self.ws = None
        self.lock = threading.Lock()
        self.running = False
        self.clock = ClockOffset()  # NETRA's clock minus ours (time_sync answers)

    def ws_url(self) -> str:
        """Return the WebSocket URL of server_url."""
        return re.sub(r'^http', 'ws', self.base_url) + '/api/v1/robot/ws'

    def headers(self) -> dict:
        """Return the auth headers for HTTP and WebSocket."""
        return {'Authorization': f'Bearer {self.token}', 'X-Robot-Id': self.robot_id}

    def start(self, upload_period: float):
        """Start the WebSocket and upload threads."""
        # Import their libraries here, before the threads: importing requests and websocket on
        # two threads at once can deadlock in the import system.
        import requests  # noqa: F401
        import websocket  # noqa: F401

        self.running = True
        threading.Thread(target=self.ws_loop, daemon=True).start()
        threading.Thread(target=self.upload_loop, args=(upload_period,), daemon=True).start()

    def stop(self):
        """Stop the threads (they finish their current wait)."""
        self.running = False
        with self.lock:
            ws, self.ws = self.ws, None
        if ws is not None:
            with contextlib.suppress(Exception):
                ws.close()

    # ---------------------------------------------------------------- WebSocket
    def ws_loop(self):
        """Keep a WebSocket open, reconnecting with backoff."""
        import ssl

        import websocket

        backoff = 1.0
        sslopt = None if self.verify else {'cert_reqs': ssl.CERT_NONE}
        while self.running:
            try:
                ws = websocket.create_connection(
                    self.ws_url(),
                    header=[f'{k}: {v}' for k, v in self.headers().items()],
                    timeout=10,
                    sslopt=sslopt,
                )
            except Exception as error:
                self.log.warning('command center unreachable', error=str(error), throttle=30.0)
                time.sleep(backoff)
                backoff = min(backoff * 2.0, BACKOFF_MAX_SEC)
                continue
            backoff = 1.0
            ws.settimeout(30)
            with self.lock:
                self.ws = ws
            self.log.info('command center connected', url=self.base_url)
            self.send({'type': 'hello', 'robot_id': self.robot_id, 'version': 1})
            self.sync_clock()
            while self.outbox and self.running:
                self.send(self.outbox.popleft(), durable=True)
            self.receive(ws, websocket.WebSocketTimeoutException)
            with self.lock:
                self.ws = None
            with contextlib.suppress(Exception):
                ws.close()
            if self.running:
                self.log.warning('command center disconnected')

    def receive(self, ws, timeout_error):
        """Queue the server's commands until the socket closes."""
        while self.running:
            try:
                text = ws.recv()
            except timeout_error:
                continue
            except Exception:
                return
            if not text:
                return
            try:
                message = json.loads(text)
            except ValueError:
                continue
            if not isinstance(message, dict):
                continue
            if message.get('type') == 'command':
                self.incoming.put(message)
            elif message.get('type') == 'time_sync':
                self.on_time(message, time.time())

    def sync_clock(self) -> bool:
        """Ask NETRA for its time (answered with time_sync {t0, t1, t2}); dropped offline."""
        return self.send({'type': 'time_sync', 't0': time.time()})

    def on_time(self, message: dict, t3: float):
        """Add NETRA's time_sync answer to the clock offset."""
        try:
            t0, t1, t2 = (float(message[k]) for k in ('t0', 't1', 't2'))
        except (KeyError, TypeError, ValueError):
            return
        self.clock.add(t0, t1, t2, t3)

    def send(self, message: dict, durable: bool = False) -> bool:
        """Send now; durable messages wait in the outbox while offline (telemetry does not)."""
        with self.lock:
            ws = self.ws
            if ws is not None:
                try:
                    ws.send(json.dumps(message, separators=(',', ':')))
                    return True
                except Exception:
                    self.ws = None
        if durable:
            self.outbox.append(message)
        return False

    # ---------------------------------------------------------------- uploads
    def upload_loop(self, period: float):
        """Upload spooled items; they stay on disk until the server accepts them."""
        import requests

        while self.running:
            try:
                self.upload_pending(requests)
            except Exception as error:  # keep the uploader alive whatever happens
                self.log.error('upload loop', error=repr(error), throttle=30.0)
            time.sleep(period)

    def upload_pending(self, requests):
        """Upload every item once, oldest first; stop at the first transient failure."""
        for item in self.spool.items():
            try:
                kind, meta, files = Spool.load(item)
                url = self.base_url + UPLOADS[kind]
            except (OSError, KeyError, ValueError) as error:
                self.log.error('unreadable spool item, quarantined', item=item, error=str(error))
                self.spool.quarantine(item)
                continue
            # always multipart (NETRA refuses anything else), even without a file
            multipart = {'meta': (None, json.dumps(meta), 'application/json')}
            for name, data in files.items():
                kind_of = TYPES.get(os.path.splitext(name)[1], 'application/octet-stream')
                multipart[name] = (name, data, kind_of)
            try:
                reply = requests.post(url, headers=self.headers(), files=multipart, timeout=20,
                                      verify=self.verify)  # fmt: skip
            except Exception as error:
                self.log.warning('upload failed', error=str(error), throttle=30.0)
                return
            if reply.status_code < 300 or reply.status_code == 409:
                Spool.remove(item)
            elif reply.status_code in (400, 413, 422):
                self.log.error('upload rejected', kind=kind, uid=meta.get('uid'), reply=reply.text)
                Spool.remove(item)
            else:
                return

    def get(self, path: str):
        """GET a robot endpoint of NETRA; return the reply (raises on failure)."""
        import requests

        reply = requests.get(self.base_url + path, headers=self.headers(), timeout=20,
                             verify=self.verify)  # fmt: skip
        reply.raise_for_status()
        return reply

    def download_route(self, route_id: str, path: str) -> bytes:
        """Fetch a patrol route CSV into path; return it (raises on failure)."""
        data = self.get(f'/api/v1/robot/routes/{route_id}/file').content
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'wb') as f:
            f.write(data)
        return data

    def inspection_plan(self, route_id: str) -> dict:
        """Return the substation inspection plan of a route (empty without the module)."""
        plan = self.get(f'/api/v1/robot/routes/{route_id}/inspection').json()
        return plan if isinstance(plan, dict) else {}
