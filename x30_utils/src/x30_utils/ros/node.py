"""X30Node: rospy with typed configuration, structured logging, TF, safe timers and callbacks."""

from __future__ import annotations

import contextlib
import threading
from typing import TypeVar

import rospy

from x30_utils.log import Logger, ros_logger
from x30_utils.robot import RobotProfile
from x30_utils.ros.handles import (
    GuardedPublisher,
    SteadyTimer,
    raw_bytes,
    type_names,
    wait_until,
)
from x30_utils.ros.params import ROBOT, LiveConfig, ParamBinder
from x30_utils.ros.qos import COMMAND, LATCHED, SENSOR, profile
from x30_utils.ros.tf import TfCache, TfPublisher

T = TypeVar('T')

PARAM_POLL_SEC = 1.0
UNKNOWN_CHECK_SEC = 0.5
NODE_INTERNALS = (
    'log', '_name', '_lock', '_binder', '_handles', '_timers', '_hooks', '_destroyed',
    '_halted', '_tf_cache', '_tf_publisher', '_polling',
)  # fmt: skip


class RospyLogger:
    """The rospy log functions under the names the log sinks call."""

    debug = staticmethod(rospy.logdebug)
    info = staticmethod(rospy.loginfo)
    warning = staticmethod(rospy.logwarn)
    error = staticmethod(rospy.logerr)
    fatal = staticmethod(rospy.logfatal)


class X30Node:
    """Base class of every node: callbacks run one at a time, as on a single-threaded executor."""

    def __init__(self, name: str, *, param_ns: str | None = None, overrides: dict | None = None):
        if not rospy.core.is_initialized():
            raise RuntimeError('rospy.init_node() first (x30_utils.ros.run does it)')
        self._name = name
        self.log: Logger = ros_logger(name, RospyLogger())
        self._lock = threading.RLock()
        self._binder = ParamBinder(param_ns or rospy.get_name(), overrides)
        self._handles: list = []
        self._timers: list = []
        self._hooks: list = []
        self._destroyed = self._halted = self._polling = False
        self._tf_cache: TfCache | None = None
        self._tf_publisher: TfPublisher | None = None
        self.after(UNKNOWN_CHECK_SEC, self._warn_unknown_params)

    @property
    def name(self) -> str:
        """The node name given by the subclass (logs)."""
        return self._name

    @property
    def param_namespace(self) -> str:
        """Where this node's parameters are on the server (the private namespace)."""
        return self._binder.namespace

    def declare_config(self, cls: type[T], prefix: str = '', *, frozen=False) -> LiveConfig[T]:
        """Read a configuration dataclass from the parameters; live fields follow changes."""
        config = self._binder.declare(cls, prefix, frozen)
        if not self._polling:
            self._polling = True
            self.every(PARAM_POLL_SEC, self._poll_params, steady=True)
        return config

    def declare_robot(self) -> LiveConfig[RobotProfile]:
        """Read the shared robot profile (robot/ searched up from the node), read-only."""
        if self._binder.find_robot() is None:
            self.log.warning('no robot/ parameters found: robot profile defaults in use')
        return self.declare_config(RobotProfile, f'{ROBOT}.', frozen=True)

    @property
    def tf(self) -> TfCache:
        """TF lookups (listener created on first use)."""
        if self._tf_cache is None:
            self._tf_cache = TfCache(self)
        return self._tf_cache

    @property
    def tf_out(self) -> TfPublisher:
        """TF broadcasting (created on first use)."""
        if self._tf_publisher is None:
            self._tf_publisher = TfPublisher(self)
        return self._tf_publisher

    @property
    def destroyed(self) -> bool:
        """True once destroy_node() ran: publications are dropped, callbacks do nothing."""
        return self._destroyed

    def now(self) -> float:
        """Return ROS time in seconds (simulated time with /use_sim_time)."""
        return rospy.get_time()

    def stamp(self) -> rospy.Time:
        """Return ROS time as a header stamp."""
        return rospy.Time.now()

    def create_publisher(self, msg_type, topic: str, qos=COMMAND) -> GuardedPublisher:
        """Create a publisher whose publish() is a no-op once the node is destroyed."""
        p = profile(qos)
        pub = rospy.Publisher(
            topic, msg_type, queue_size=p.queue_size, latch=p.latch, tcp_nodelay=p.tcp_nodelay
        )
        guarded = GuardedPublisher(pub, lambda: self._destroyed)
        self._handles.append(guarded)
        return guarded

    def latched(self, msg_type, topic: str) -> GuardedPublisher:
        """Create a publisher whose last message reaches late subscribers."""
        return self.create_publisher(msg_type, topic, LATCHED)

    def create_subscription(self, msg_type, topic: str, callback, qos=SENSOR, *, raw=False):
        """Subscribe; raw=True passes the serialized bytes (msg_type: accepted type names)."""
        p = profile(qos)
        handler = self._callback(callback, f'{topic} callback failed')
        if raw:
            accepted = type_names(msg_type)

            def wrong_type(kind: str):
                expected = ' or '.join(accepted)
                self.log.error(f'{topic}: {kind} is not {expected}, dropped', throttle=10.0)

            handler = raw_bytes(handler, accepted, wrong_type)
        sub = rospy.Subscriber(
            topic,
            rospy.AnyMsg if raw else msg_type,
            handler,
            queue_size=p.queue_size or None,
            buff_size=p.buff_size,
            tcp_nodelay=p.tcp_nodelay,
        )
        self._handles.append(sub)
        return sub

    def destroy_subscription(self, subscription):
        """Unregister a subscription of this node: no callbacks from it once this returns."""
        with self._lock:  # a delivery in flight finishes first
            with contextlib.suppress(ValueError):
                self._handles.remove(subscription)
            subscription.unregister()

    def create_service(self, srv_type, name: str, callback, *, concurrent: bool = False):
        """Serve callback(request, response) -> response; an exception fails the call."""
        # concurrent: the callback runs beside the other callbacks instead of after them, for a
        # service that waits for them (a recording that the timers finish); it guards its state
        response_class = srv_type._response_class

        def serve(request):
            with contextlib.nullcontext() if concurrent else self._lock:
                if self._halted:
                    raise rospy.ServiceException('node is shutting down')
                try:
                    return callback(request, response_class())
                except Exception as error:
                    self.log.exception(f'service {name} failed')
                    raise rospy.ServiceException(str(error)) from error

        service = rospy.Service(name, srv_type, serve)
        self._handles.append(service)
        return service

    def every(self, period: float, callback, *, steady: bool = False, on_error=None):
        """Call callback every period s (steady: monotonic time); errors logged or on_error."""
        guarded = self._callback(callback, f'{callback.__name__} failed', on_error)
        if steady:
            timer = SteadyTimer(period, guarded, f'{self._name}.{callback.__name__}')
        else:
            timer = rospy.Timer(rospy.Duration.from_sec(period), lambda _event: guarded())
        self._timers.append(timer)
        return timer

    def after(self, delay: float, callback):
        """Call callback once after delay s (monotonic time)."""
        guarded = self._callback(callback, f'{callback.__name__} failed')
        timer = SteadyTimer(delay, guarded, f'{self._name}.{callback.__name__}', oneshot=True)
        self._timers.append(timer)
        return timer

    def guard(self, callback, what: str = ''):
        """Return callback run like the node's own: one at a time with them, errors logged."""
        name = getattr(callback, '__name__', 'callback')
        return self._callback(callback, what or f'{name} failed')

    def add_shutdown_hook(self, hook):
        """Run hook() when the node stops (e.g. publish a zero command)."""
        self._hooks.append(hook)

    def run_shutdown_hooks(self):
        """Run every shutdown hook once, before destroy_node, ignoring their failures."""
        hooks, self._hooks = self._hooks, []
        if self._destroyed:
            return
        for hook in hooks:
            try:
                hook()
            except Exception:
                self.log.exception('shutdown hook failed')

    def flush(self, timeout: float = 0.5) -> bool:
        """Wait until every publisher wrote its queued messages (before unregistering)."""
        publishers = [h for h in self._handles if isinstance(h, GuardedPublisher)]
        return wait_until(lambda: all(p.flushed() for p in publishers), timeout)

    def halt(self, timeout: float = 2.0) -> bool:
        """Stop timers and callbacks; return False if a running callback outlasted timeout."""
        self._halted = True
        for timer in self._timers:
            timer.shutdown()
        if self._lock.acquire(timeout=timeout):
            self._lock.release()
            return True
        return False

    def destroy_node(self):
        """Unregister everything; later publications from other threads are dropped."""
        self.halt(timeout=0.0)
        self._destroyed = True
        for handle in self._handles:
            with contextlib.suppress(Exception):
                handle.shutdown() if isinstance(handle, rospy.Service) else handle.unregister()
        if self._tf_cache is not None:
            self._tf_cache.close()
        if self._tf_publisher is not None:
            self._tf_publisher.close()
        self._handles.clear()

    def _callback(self, callback, what: str, on_error=None):
        def guarded(*args):
            with self._lock:
                if self._halted:
                    return
                try:
                    callback(*args)
                except Exception as error:
                    if on_error is not None:
                        on_error(error)
                    else:
                        self.log.exception(what, throttle=5.0)

        return guarded

    def _poll_params(self):
        for note in self._binder.poll():
            log = self.log.info if note.startswith('applied') else self.log.warning
            log(f'parameters {note}')

    def _warn_unknown_params(self):
        unknown = self._binder.unknown()
        if unknown:
            self.log.warning('unknown parameters ignored (typo?)', names=','.join(unknown))
