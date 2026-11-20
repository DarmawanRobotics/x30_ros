"""The HIKMICRO PTZ camera through hikmicro_driver: the Camera of core.runner on ROS."""

from __future__ import annotations

import math
import os
import threading
import time

import actionlib
from hikmicro_msgs.msg import GotoPresetAction, GotoPresetGoal, MoveToAction, MoveToGoal, PtzState
from hikmicro_msgs.srv import GetTemperature, GetTemperatureRequest, Snapshot, SnapshotRequest
from sensor_msgs.msg import Image

from x30_inspection.core.config import CameraConfig
from x30_inspection.core.runner import Camera
from x30_inspection.core.thermal import Reading
from x30_inspection.utils.actions import run_goal
from x30_utils.io import image_temperatures, write_thermal
from x30_utils.ros import EVENTS, SENSOR
from x30_utils.ros.services import ServiceClient

ACTION_MARGIN_SEC = 5.0  # past the driver's own move timeout
SNAPSHOTS = ((SnapshotRequest.THERMAL, 'thermal'), (SnapshotRequest.VISIBLE, 'visible'))
UNKNOWN = (math.nan, math.nan, math.nan)


class HikmicroCamera(Camera):
    """Pan, tilt, zoom with arrival, temperatures and snapshots of the camera."""

    # calls block the inspection's thread (a concurrent service), never a node callback
    def __init__(self, node, config: CameraConfig, max_age_sec: float):
        self.node, self.config, self.max_age = node, config, max_age_sec
        ns = config.namespace.rstrip('/')
        self.lock = threading.Lock()
        self.state: tuple[PtzState | None, float] = (None, 0.0)
        self.frame: Image | None = None  # the newest thermometry frame
        self.settled = 0.0  # ROS time the camera settled at the last target
        self.move = actionlib.SimpleActionClient(f'{ns}/ptz/move_to', MoveToAction)
        self.preset = actionlib.SimpleActionClient(f'{ns}/ptz/goto_preset', GotoPresetAction)
        self.temperature = ServiceClient(GetTemperature, f'{ns}/thermal/get_temperature')
        self.snapshots = ServiceClient(Snapshot, f'{ns}/snapshot')
        node.create_subscription(PtzState, f'{ns}/ptz/state', self.on_state, EVENTS)
        if config.raw_thermal:
            node.create_subscription(Image, f'{ns}/thermal/temperature', self.on_frame, SENSOR)

    def on_state(self, msg: PtzState):
        """Keep the camera's position."""
        with self.lock:
            self.state = (msg, time.monotonic())

    def on_frame(self, msg: Image):
        """Keep the newest thermometry frame (32FC1, degC per pixel)."""
        with self.lock:
            self.frame = msg

    def fresh_state(self) -> PtzState | None:
        """Return the latest connected state, None when stale or disconnected."""
        with self.lock:
            msg, at = self.state
        if msg is None or not msg.connected:
            return None
        return msg if time.monotonic() - at <= self.config.state_max_age_sec else None

    def connected(self) -> bool:
        """Return True while the driver reports a connected camera."""
        return self.fresh_state() is not None

    def pose(self) -> tuple[float, float, float] | None:
        """Return the current (pan, tilt, zoom), None when unknown or stale."""
        msg = self.fresh_state()
        return None if msg is None else (float(msg.pan), float(msg.tilt), float(msg.zoom))

    def point(self, pan_deg: float, tilt_deg: float, zoom: float, cancelled) -> str:
        """Move there; '' once the camera reports it has arrived, else the error."""
        timeout = self.config.move_timeout_sec
        goal = MoveToGoal(pan=float(pan_deg), tilt=float(tilt_deg), zoom=float(zoom),
                          timeout=timeout)  # fmt: skip
        result, error = run_goal(self.move, goal, timeout + ACTION_MARGIN_SEC, cancelled)
        if result is None:
            return f'ptz/move_to {error}'
        self.settled = self.node.now() + self.config.settle_sec
        return '' if result.reached else f'ptz/move_to {result.state}: {result.message}'

    def measure(self, box) -> tuple[Reading | None, str]:
        """Return the temperatures of a box (fractions of the frame) or of the frame (None)."""
        request = GetTemperatureRequest(max_age=self.max_age)
        if box is None:
            request.mode = GetTemperatureRequest.FRAME
        else:
            request.mode, request.normalized = GetTemperatureRequest.REGION, True
            request.x, request.y, request.x2, request.y2 = (float(v) for v in box)
        response, error = self.temperature.call(request, self.config.call_timeout_sec)
        if response is None:
            return None, error
        if not response.success:
            return None, response.message or 'get_temperature refused'
        s = response.stats
        if not (math.isfinite(s.max) and math.isfinite(s.min)):
            return None, 'temperatures not finite'
        return Reading('', float(s.max), float(s.min), int(s.max_x), int(s.max_y)), ''

    def snapshot(self, prefix: str) -> list[str]:
        """Save full-resolution JPEGs as <prefix>_thermal.jpg and _visible.jpg; return them."""
        files = []
        for channel, name in SNAPSHOTS:
            response, error = self.snapshots.call(SnapshotRequest(channel=channel),
                                                  self.config.call_timeout_sec)  # fmt: skip
            if response is None or not response.success or not response.image.data:
                why = error or (response.message if response else '') or 'empty image'
                self.node.log.warning('no snapshot for the evidence', image=name, detail=why)
                continue
            path = f'{prefix}_{name}.jpg'
            try:
                with open(path, 'wb') as f:
                    f.write(bytes(response.image.data))
            except OSError as failure:
                self.node.log.warning('snapshot not saved', path=path, detail=str(failure))
                continue
            files.append(os.path.abspath(path))
        raw = self.raw_thermal(prefix)
        return files + ([raw] if raw else [])

    def raw_thermal(self, prefix: str) -> str:
        """Save the newest frame since the camera settled as <prefix>_thermal.npz ('' if none)."""
        if not self.config.raw_thermal:
            return ''
        path = f'{prefix}_thermal.npz'
        deadline = time.monotonic() + self.config.raw_wait_sec
        while True:
            with self.lock:
                msg = self.frame
            if msg is None:  # no thermal/temperature: hikmicro_driver's radiometric_rate 0
                return ''
            if msg.header.stamp.to_sec() >= self.settled:
                break
            if time.monotonic() >= deadline:
                self.node.log.warning('no raw thermal frame since the camera settled', path=path)
                return ''
            time.sleep(0.05)
        meta = {'frame_id': msg.header.frame_id, 'target': os.path.basename(prefix)}
        big = bool(msg.is_bigendian)
        try:
            if msg.encoding != '32FC1':
                raise ValueError(f'{msg.encoding} is not 32FC1')
            data = image_temperatures(msg.height, msg.width, msg.step, msg.data, big)
            stamp, ptz = msg.header.stamp.to_sec(), self.pose() or UNKNOWN
            write_thermal(path, data, stamp, ptz=ptz, meta=meta)
        except (OSError, ValueError) as failure:
            self.node.log.warning('raw thermal frame not saved', path=path, detail=str(failure))
            return ''
        return os.path.abspath(path)

    def park(self) -> str:
        """Call the park preset; '' once the camera has arrived there."""
        timeout = self.config.move_timeout_sec
        goal = GotoPresetGoal(id=int(self.config.park_preset), timeout=timeout)
        result, error = run_goal(self.preset, goal, timeout + ACTION_MARGIN_SEC)
        if result is None:
            return f'ptz/goto_preset {error}'
        return '' if result.reached else f'ptz/goto_preset {result.state}: {result.message}'

    def close(self):
        """Unregister the action clients (ptz/state is the node's subscription)."""
        for client in (self.move, self.preset):
            client.action_client.stop()
