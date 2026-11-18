"""The Hertzino HZ-FA-371 through hertzino_driver: the Acoustic of core.discharge on ROS."""

from __future__ import annotations

import os
import threading
import time

import actionlib
from hertzino_msgs.msg import AcousticStatus, DetectAction, DetectGoal

from x30_inspection.core.config import AcousticConfig
from x30_inspection.core.discharge import Acoustic, Detection
from x30_inspection.utils.actions import run_goal
from x30_utils.ros import EVENTS

ACTION_MARGIN_SEC = 5.0  # past the driver's own detection timeout
PHOTOS = (('soundmap_photo', 'soundmap'), ('prpd_photo', 'prpd'))


class HertzinoAcoustic(Acoustic):
    """One detection at a time, with its sound map and PRPD photos."""

    def __init__(self, node, config: AcousticConfig):
        self.node, self.config = node, config
        ns = config.namespace.rstrip('/')
        self.lock = threading.Lock()
        self.status: tuple[AcousticStatus | None, float] = (None, 0.0)
        self.client = actionlib.SimpleActionClient(f'{ns}/detect', DetectAction)
        node.create_subscription(AcousticStatus, f'{ns}/status', self.on_status, EVENTS)

    def on_status(self, msg: AcousticStatus):
        """Keep the camera's state."""
        with self.lock:
            self.status = (msg, time.monotonic())

    def fresh_status(self) -> AcousticStatus | None:
        """Return the latest status, None when stale."""
        with self.lock:
            msg, at = self.status
        fresh = msg is not None and time.monotonic() - at <= self.config.status_max_age_sec
        return msg if fresh else None

    def connected(self) -> bool:
        """Return True while the driver reports a connected camera."""
        msg = self.fresh_status()
        return msg is not None and bool(msg.connected)

    def mode(self) -> int:
        """Return the camera's work mode (0: unknown)."""
        msg = self.fresh_status()
        return 0 if msg is None else int(msg.config.work_mode)

    def detect(self, duration: float, prefix: str, cancelled) -> tuple:
        """Detect once (measured after now); save the photos; (Detection, files, error)."""
        timeout = duration + self.config.margin_sec
        goal = DetectGoal(duration=float(duration), timeout=float(timeout), keep_running=False,
                          fetch_photos=True)  # fmt: skip
        result, error = run_goal(self.client, goal, timeout + ACTION_MARGIN_SEC, cancelled)
        if result is None:
            return None, [], f'detect {error}'
        files = [path for path in (self.save(result, key, prefix, suffix)
                                   for key, suffix in PHOTOS) if path]  # fmt: skip
        if not result.success:
            return None, files, f'detect {result.state}: {result.message}'
        r = result.result
        probabilities = {'noise': r.noise_prob, 'corona': r.corona_prob,
                         'surface': r.surface_prob, 'floating': r.floating_prob}  # fmt: skip
        detection = Detection(int(r.detect_mode), int(r.status), float(r.intensity),
                              {k: float(v) for k, v in probabilities.items()})  # fmt: skip
        return detection, files, ''

    def save(self, result, key: str, prefix: str, suffix: str) -> str:
        """Write a photo of the result as <prefix>_<suffix>.jpg; '' when there is none."""
        photo = getattr(result, key)
        if not photo.data:
            return ''
        path = f'{prefix}_{suffix}.jpg'
        try:
            with open(path, 'wb') as f:
                f.write(bytes(photo.data))
        except OSError as failure:
            self.node.log.warning('photo not saved', path=path, detail=str(failure))
            return ''
        return os.path.abspath(path)

    def close(self):
        """Unregister the action client (status is the node's subscription)."""
        self.client.action_client.stop()
