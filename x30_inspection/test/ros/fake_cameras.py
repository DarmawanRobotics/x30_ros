"""hikmicro_driver's and hertzino_driver's ROS interfaces, faked for the node tests."""

from __future__ import annotations

import threading
import time

import actionlib
from hertzino_msgs.msg import AcousticStatus, DetectAction, DetectResult
from hikmicro_msgs.msg import (
    GotoPresetAction,
    GotoPresetResult,
    MoveToAction,
    MoveToResult,
    PtzState,
)
from hikmicro_msgs.srv import (
    GetTemperature,
    GetTemperatureRequest,
    GetTemperatureResponse,
    Snapshot,
    SnapshotRequest,
    SnapshotResponse,
)
import rospy

JPEG = b'\xff\xd8\xff\xe0fake'


def key(box) -> tuple:
    """Return a box rounded for lookups (None: the frame)."""
    return 'frame' if box is None else tuple(round(v, 3) for v in box)


def stop_server(server: actionlib.SimpleActionServer):
    """Stop an action server and its execute thread (not a daemon: it would outlive the tests)."""
    with server.terminate_mutex:
        server.need_to_terminate = True
    server.execute_thread.join(5.0)
    server.action_server.stop()


class FakeHikmicro:
    """PTZ moves that arrive (or stall), temperatures per box, JPEG snapshots, ptz/state."""

    def __init__(self, ns: str, temps: dict | None = None):
        self.ns = ns
        self.temps = {key(box): t for box, t in (temps or {}).items()}
        self.lock = threading.Lock()
        self.pose = [0.0, 0.0, 1.0]
        self.moves, self.presets, self.measured = [], [], []
        self.stall = False
        self.state_pub = rospy.Publisher(f'{ns}/ptz/state', PtzState, queue_size=1)
        self.move = actionlib.SimpleActionServer(
            f'{ns}/ptz/move_to', MoveToAction, self.on_move, auto_start=False
        )
        self.preset = actionlib.SimpleActionServer(
            f'{ns}/ptz/goto_preset', GotoPresetAction, self.on_preset, auto_start=False
        )
        self.move.start()
        self.preset.start()
        self.services = [
            rospy.Service(f'{ns}/thermal/get_temperature', GetTemperature, self.on_temperature),
            rospy.Service(f'{ns}/snapshot', Snapshot, self.on_snapshot),
        ]
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_state)

    def publish_state(self, _event=None):
        with self.lock:
            pan, tilt, zoom = self.pose
        msg = PtzState(connected=True, pan=pan, tilt=tilt, zoom=zoom)
        msg.header.stamp = rospy.Time.now()
        self.state_pub.publish(msg)

    def on_move(self, goal):
        with self.lock:
            self.moves.append((goal.pan, goal.tilt, goal.zoom))
            if not self.stall:
                self.pose = [goal.pan, goal.tilt, goal.zoom]
            pan, tilt, zoom = self.pose
        result = MoveToResult(pan=pan, tilt=tilt, zoom=zoom)
        if self.stall:
            result.state, result.message = 'stalled', 'the PTZ stopped short'
            self.move.set_aborted(result, result.message)
        else:
            result.reached, result.state = True, 'reached'
            self.move.set_succeeded(result)

    def on_preset(self, goal):
        self.presets.append(goal.id)
        self.preset.set_succeeded(GotoPresetResult(reached=True, state='reached'))

    def on_temperature(self, request):
        box = None
        if request.mode == GetTemperatureRequest.REGION:
            box = (request.x, request.y, request.x2, request.y2)
        self.measured.append((key(box), request.normalized))
        response = GetTemperatureResponse(success=True)
        response.stats.max = self.temps.get(key(box), 35.0)
        response.stats.min, response.stats.max_x, response.stats.max_y = 25.0, 320, 256
        return response

    def on_snapshot(self, request):
        response = SnapshotResponse(success=True)
        name = b'thermal' if request.channel == SnapshotRequest.THERMAL else b'visible'
        response.image.format, response.image.data = 'jpeg', JPEG + name
        return response

    def close(self):
        self.timer.shutdown()
        for service in self.services:
            service.shutdown()
        stop_server(self.move)
        stop_server(self.preset)
        self.state_pub.unregister()


class FakeHertzino:
    """One detection per goal after its duration, with photos; status in partial discharge."""

    def __init__(self, ns: str, status: int = 1, intensity: int = 12, mode: int = 1):
        self.ns = ns
        self.status, self.intensity, self.mode = status, intensity, mode
        self.goals = []
        self.status_pub = rospy.Publisher(f'{ns}/status', AcousticStatus, queue_size=1)
        self.server = actionlib.SimpleActionServer(
            f'{ns}/detect', DetectAction, self.on_detect, auto_start=False
        )
        self.server.start()
        self.timer = rospy.Timer(rospy.Duration(0.2), self.publish_status)

    def publish_status(self, _event=None):
        msg = AcousticStatus(connected=True)
        msg.config.work_mode = self.mode
        msg.header.stamp = rospy.Time.now()
        self.status_pub.publish(msg)

    def on_detect(self, goal):
        self.goals.append(goal)
        end = time.monotonic() + min(goal.duration, 2.0)
        while time.monotonic() < end:
            if self.server.is_preempt_requested():
                self.server.set_preempted(DetectResult(state='preempted'))
                return
            time.sleep(0.05)
        result = DetectResult(success=True, state='detected')
        r = result.result
        r.detect_mode, r.status, r.intensity = self.mode, self.status, self.intensity
        r.noise_prob, r.surface_prob = 0.1, 0.8
        result.soundmap_photo.format, result.soundmap_photo.data = 'jpeg', JPEG + b'soundmap'
        result.prpd_photo.format, result.prpd_photo.data = 'jpeg', JPEG + b'prpd'
        self.server.set_succeeded(result)

    def close(self):
        self.timer.shutdown()
        stop_server(self.server)
        self.status_pub.unregister()
