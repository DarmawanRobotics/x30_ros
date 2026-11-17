"""Entry point boilerplate: init, spin, stop the callbacks, run the shutdown hooks, clean up."""

from __future__ import annotations

import faulthandler
import signal
import sys
import threading

import rospy

from x30_utils.config import ConfigError

WAKE_UP_SEC = 0.1
STOP_TIMEOUT_SEC = 2.0
FLUSH_TIMEOUT_SEC = 0.5  # the hooks' messages leave before the topics close
# After a stop signal the process must be gone this soon (roslaunch escalates to SIGTERM after
# 15 s); otherwise every thread's stack goes to stderr and it exits with 1.
EXIT_DEADLINE_SEC = 8.0
SIGNALS = (signal.SIGINT, signal.SIGTERM)
EXIT_CONFIG = 2


class _StopRequest:
    """SIGINT / SIGTERM handler: raises while the node is built, then only records."""

    def __init__(self, exit_deadline_sec: float | None):
        self.requested = False
        self.interrupt = True
        self.exit_deadline_sec = exit_deadline_sec
        self.event = threading.Event()

    def __call__(self, signum, frame):
        if not self.requested and self.exit_deadline_sec:
            # a C watchdog thread: it fires even when the main thread is stuck holding the GIL
            faulthandler.dump_traceback_later(self.exit_deadline_sec, exit=True)
        self.requested = True
        self.event.set()
        if self.interrupt:
            raise KeyboardInterrupt


class _Finisher:
    """Stop the callbacks, run the hooks while topics are open, then destroy: exactly once."""

    def __init__(self):
        self.node = None
        self._done = False
        self._lock = threading.Lock()

    def __call__(self, reason: str = ''):
        with self._lock:
            if self._done or self.node is None:
                return
            self._done = True
            node = self.node
        if not node.halt(STOP_TIMEOUT_SEC):
            node.log.warning('callbacks still running at shutdown', timeout=STOP_TIMEOUT_SEC)
        node.run_shutdown_hooks()
        if not node.flush(FLUSH_TIMEOUT_SEC):
            node.log.warning('messages still queued at shutdown', timeout=FLUSH_TIMEOUT_SEC)
        node.destroy_node()


def run(node_factory, name: str, *, argv=None, exit_deadline_sec=EXIT_DEADLINE_SEC):
    """Init rospy as name, create the node, spin and stop cleanly on Ctrl-C, SIGTERM or kill."""
    stop = _StopRequest(exit_deadline_sec)
    for signum in SIGNALS:
        signal.signal(signum, stop)
    finish = _Finisher()
    try:
        rospy.init_node(name, argv=argv if argv is not None else sys.argv, disable_signals=True)
        # rosnode kill and a master shutdown: the hooks run before rospy closes the topics
        rospy.core.add_preshutdown_hook(finish)
        finish.node = node_factory()
        stop.interrupt = False
        while not stop.requested and not rospy.is_shutdown():
            stop.event.wait(WAKE_UP_SEC)
    except KeyboardInterrupt:
        pass
    except ConfigError as error:
        rospy.logfatal(f'{name}: invalid parameters, not starting: {error}')
        sys.exit(EXIT_CONFIG)
    finally:
        stop.interrupt = False
        try:
            finish('stopped')
        finally:
            rospy.signal_shutdown('stopped')
            # A signal arriving while the interpreter exits would end the process with -SIGINT.
            for signum in SIGNALS:
                signal.signal(signum, signal.SIG_IGN)
