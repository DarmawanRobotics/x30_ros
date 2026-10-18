"""
A small stand-in for rospy, for test_node.py: the driver node runs without a ROS master.

Publishers record what they publish, services and subscribers are kept by name, timers run on
threads, parameters come from PARAMS. reset() starts over for the next node.
"""

import threading
import time
import traceback

PARAMS = {}
published = []     # (topic, message)
services = {}      # name -> handler
subscribers = {}   # topic -> callback
timers = []        # Timer
errors = []        # exceptions raised in timer callbacks
warnings = []      # throttled warnings (format, args)
_shutdown_hooks = []
_shutting = threading.Event()


def reset():
    """Forget the last node (call it after signal_shutdown)."""
    PARAMS.clear()
    del published[:]
    services.clear()
    subscribers.clear()
    del timers[:]
    del errors[:]
    del warnings[:]
    del _shutdown_hooks[:]
    _shutting.clear()


class Time(object):
    def __init__(self, t=None):
        self.t = time.time() if t is None else t

    @staticmethod
    def now():
        return Time()


class Duration(object):
    def __init__(self, d):
        self.d = d


class Publisher(object):
    def __init__(self, name, kind, queue_size=0, latch=False):
        self.name = name

    def publish(self, msg):
        published.append((self.name, msg))


def Subscriber(name, kind, callback, queue_size=0):  # noqa: N802 (rospy's name)
    subscribers[name] = callback


def Service(name, kind, handler):  # noqa: N802 (rospy's name)
    services[name] = handler
    return name


class Timer(object):
    def __init__(self, period, callback):
        self.period = period.d
        self.callback = callback
        self.alive = True
        timers.append(self)
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        while self.alive and not _shutting.is_set():
            time.sleep(self.period)
            if not self.alive:
                return
            try:
                self.callback(None)
            except Exception:  # noqa: B902 rospy prints it and the timer thread ends
                errors.append(traceback.format_exc())
                return

    def shutdown(self):
        self.alive = False


def get_param(name, default=None):
    return PARAMS.get(name.lstrip('~'), default)


def _log(*args):
    pass


loginfo = logwarn = logerr = logfatal = logdebug = _log


def logwarn_throttle(period, msg, *args):
    warnings.append((msg, args))


def init_node(name):
    pass


def on_shutdown(callback):
    _shutdown_hooks.append(callback)


def is_shutdown():
    return _shutting.is_set()


def signal_shutdown(reason=''):
    _shutting.set()
    for callback in _shutdown_hooks:
        callback()


def spin():
    while not _shutting.is_set():
        time.sleep(0.1)
