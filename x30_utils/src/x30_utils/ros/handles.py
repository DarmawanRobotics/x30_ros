"""The pieces X30Node hands out: guarded publishers, steady timers, raw subscriptions."""

from __future__ import annotations

import threading
import time

import rospy

STEADY_MIN_PERIOD = 1e-4
POLL_SEC = 0.005


def wait_until(predicate, timeout: float) -> bool:
    """Poll predicate() on monotonic time; return its last value."""
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            return False
        time.sleep(POLL_SEC)
    return True


class GuardedPublisher:
    """A rospy publisher that drops messages once its node is destroyed or ROS is shut down."""

    def __init__(self, publisher: rospy.Publisher, destroyed):
        self.publisher = publisher
        self._destroyed = destroyed

    @property
    def name(self) -> str:
        """The resolved topic name."""
        return self.publisher.resolved_name

    def get_num_connections(self) -> int:
        """Return the number of subscribers connected."""
        return self.publisher.get_num_connections()

    def publish(self, msg):
        """Publish msg; silently dropped after shutdown (other threads may still publish)."""
        if self._destroyed() or rospy.is_shutdown():
            return
        try:
            self.publisher.publish(msg)
        except rospy.ROSException:
            if self._destroyed() or rospy.is_shutdown():
                return
            raise

    def flushed(self) -> bool:
        """Return True when every connection wrote what was published (rospy queues)."""
        impl = getattr(self.publisher, 'impl', None)
        for connection in list(getattr(impl, 'connections', None) or ()):
            # rospy's QueuedConnection: a writer thread per subscriber, idle when _waiting
            if getattr(connection, '_queue', None) or not getattr(connection, '_waiting', True):
                return False
        return True

    def unregister(self):
        """Unregister the topic."""
        self.publisher.unregister()


class SteadyTimer:
    """Call a function every period on monotonic time, in its own thread (sim time aside)."""

    def __init__(self, period: float, callback, name: str = 'timer', oneshot: bool = False):
        if not period > 0.0:
            raise ValueError(f'timer period must be > 0, got {period}')
        self.period = max(STEADY_MIN_PERIOD, float(period))
        self.callback = callback
        self.oneshot = oneshot
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    def _run(self):
        deadline = time.monotonic() + self.period
        while not self._stop.wait(max(0.0, deadline - time.monotonic())):
            if rospy.is_shutdown():
                return
            self.callback()
            if self.oneshot:
                return
            deadline += self.period
            now = time.monotonic()
            if deadline < now:  # late by more than a period: skip the missed ones
                deadline = now + self.period

    def shutdown(self):
        """Stop the timer (a running call finishes)."""
        self._stop.set()

    def is_alive(self) -> bool:
        """Return True while the timer thread runs."""
        return self._thread.is_alive()


def type_names(msg_type) -> tuple[str, ...]:
    """Return the accepted ROS type names of a raw subscription (empty: any type)."""
    if msg_type is None:
        return ()
    if isinstance(msg_type, str):
        return (msg_type,)
    if isinstance(msg_type, (tuple, list)):
        return tuple(str(t) for t in msg_type)
    return (msg_type._type,)


def raw_message(handler):
    """Wrap handler(bytes, connection header) as a rospy.AnyMsg callback (any type)."""

    def receive(msg: rospy.AnyMsg):
        handler(msg._buff, getattr(msg, '_connection_header', None) or {})

    return receive


def raw_bytes(handler, accepted: tuple[str, ...] = (), on_wrong_type=None):
    """Wrap handler(bytes) as a rospy.AnyMsg callback; other types go to on_wrong_type(type)."""

    def receive(msg: rospy.AnyMsg):
        header = getattr(msg, '_connection_header', None) or {}
        kind = header.get('type', '')
        if accepted and kind not in accepted:
            if on_wrong_type is not None:
                on_wrong_type(kind)
            return
        handler(msg._buff)

    return receive
