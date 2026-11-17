"""TF lookups as numpy transforms, with the age of the data and per-edge diagnostics."""

from __future__ import annotations

import contextlib
from dataclasses import dataclass

import numpy as np
import rospy
import tf2_ros
import yaml

from x30_utils.math import Pose2D
from x30_utils.ros.convert import transform_stamped, transform_to_matrix
from x30_utils.ros.time import to_sec, to_stamp


@dataclass(frozen=True)
class TfResult:
    """A looked-up transform, the time it is valid at and how old that is."""

    matrix: np.ndarray
    stamp: float
    age: float

    @property
    def pose2d(self) -> Pose2D:
        """Planar part of the transform."""
        return Pose2D.from_matrix(self.matrix)


def _when(stamp) -> rospy.Time:
    """Return a lookup time: None is the latest common time, floats are seconds."""
    if stamp is None:
        return rospy.Time(0)
    if isinstance(stamp, (int, float)):
        return to_stamp(stamp)
    return stamp


class TfCache:
    """TF buffer and listener (/tf and /tf_static, also in a namespace); statics cached."""

    def __init__(self, node, cache_sec: float = 10.0):
        self.node = node
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration.from_sec(cache_sec))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self._static: dict[tuple[str, str], np.ndarray] = {}
        self.error = ''

    def lookup(self, target: str, source: str, stamp=None) -> TfResult | None:
        """Return target <- source at stamp (None: the latest common time), or None."""
        try:
            t = self.buffer.lookup_transform(target, source, _when(stamp))
        except tf2_ros.TransformException as error:
            self.error = str(error)
            return None
        at = to_sec(t.header.stamp)
        return TfResult(transform_to_matrix(t.transform), at, self.node.now() - at)

    def fresh(self, target: str, source: str, max_age: float) -> TfResult | None:
        """Return the latest transform when it is at most max_age old."""
        result = self.lookup(target, source)
        if result is not None and result.age > max_age:
            self.error = f'{target} <- {source} is {result.age:.2f} s old'
            return None
        return result

    def static(self, target: str, source: str) -> np.ndarray | None:
        """Return a transform that never changes (URDF, extrinsics), cached."""
        key = (target, source)
        if key not in self._static:
            result = self.lookup(target, source)
            if result is None:
                return None
            self._static[key] = result.matrix
        return self._static[key]

    def edge_ages(self) -> dict[str, tuple[str, float]]:
        """Return child -> (parent, age of its newest transform) for every dynamic edge."""
        try:
            frames = yaml.safe_load(self.buffer.all_frames_as_yaml()) or {}
        except yaml.YAMLError:
            return {}
        now = self.node.now()
        out = {}
        for child, info in frames.items():
            newest = float(info.get('most_recent_transform', 0.0) or 0.0)
            if newest > 0.0:
                out[child] = (info.get('parent', ''), now - newest)
        return out

    def close(self):
        """Stop listening."""
        with contextlib.suppress(Exception):
            self.listener.unregister()


class TfPublisher:
    """Broadcast dynamic and static transforms from numpy matrices."""

    def __init__(self, node):
        self.node = node
        self._dynamic = tf2_ros.TransformBroadcaster()
        self._static = tf2_ros.StaticTransformBroadcaster()

    def send(self, matrix: np.ndarray, parent: str, child: str, stamp=None):
        """Broadcast a dynamic transform (stamp: now by default; floats are seconds)."""
        when = self.node.stamp() if stamp is None else _when(stamp)
        self._dynamic.sendTransform(transform_stamped(matrix, parent, child, when))

    def send_static(self, transforms: list[tuple[np.ndarray, str, str]]):
        """Latch static transforms (all of them in one message: a new call replaces them)."""
        stamp = self.node.stamp()
        self._static.sendTransform([transform_stamped(m, p, c, stamp) for m, p, c in transforms])
