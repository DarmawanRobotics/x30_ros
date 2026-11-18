"""Launch prefix: wait until the IMU is still and level, then exec the estimator."""

from __future__ import annotations

import math
import os
import signal
import sys
import threading
import time

import rospy
from sensor_msgs.msg import Imu

from x30_odometry.core.config import StillConfig
from x30_odometry.core.still import GateDecision, StillGate
from x30_utils.config import ConfigError
from x30_utils.ros import SENSOR, X30Node

NAME = 'odometry_gate'
LOG_PERIOD_SEC = 5.0
INTERRUPTED = 130  # the shell's code for a command ended by SIGINT
INVALID = 2  # invalid parameters, as x30_utils.ros.run


class StillGateNode(X30Node):
    """Collects IMU samples until the core gate releases the estimator start."""

    def __init__(self, **kwargs):
        super().__init__(NAME, **kwargs)
        self.settings = self.declare_config(StillConfig).value
        self.gate = StillGate(self.settings, time.monotonic())
        self.decision: GateDecision | None = None
        self.released = threading.Event()
        self.last_log = -LOG_PERIOD_SEC
        topic = self.settings.imu_topic
        self.create_subscription(Imu, topic, self.on_imu, SENSOR)
        self.every(0.2, self.evaluate, steady=True)
        self.log.info(
            'holding the odometry start until the IMU is still',
            imu=topic,
            still_sec=self.settings.still_sec,
            max_tilt_deg=self.settings.max_tilt_deg,
        )

    @property
    def done(self) -> bool:
        """True once the estimator may start."""
        return self.released.is_set()

    def on_imu(self, msg: Imu):
        """Add a sample to the window."""
        w, a = msg.angular_velocity, msg.linear_acceleration
        self.gate.add(time.monotonic(), (w.x, w.y, w.z), (a.x, a.y, a.z))

    def evaluate(self):
        """Ask the gate; log why the start is held now and then."""
        if self.done:
            return
        now = time.monotonic()
        decision = self.gate.decide(now)
        elapsed = now - self.gate.start
        if decision.release and decision.forced:
            self.log.error(
                'not still, starting odometry anyway: expect drift, restart it standing still',
                reason=decision.reason,
                waited_sec=round(elapsed, 1),
            )
        elif decision.release:
            s = decision.stillness
            self.log.info(
                'IMU still, starting odometry',
                gyro_rad_s=round(s.gyro_max, 4),
                acc_rel_std=round(s.acc_rel_std, 4),
                tilt_deg=round(math.degrees(s.tilt), 2),
                waited_sec=round(elapsed, 1),
            )
        elif now - self.last_log >= LOG_PERIOD_SEC:
            self.last_log = now
            self.log.warning('odometry start held', reason=decision.reason)
        self.decision = decision
        if decision.release:
            self.released.set()


def split_args(argv: list[str]) -> list[str]:
    """Return the command of `-- COMMAND [ARGS...]` (its remappings are the command's own)."""
    if '--' not in argv:
        raise SystemExit('usage: wait_still -- COMMAND [ARGS...]')
    i = argv.index('--')
    if argv[:i]:
        raise SystemExit(f'wait_still: unknown arguments {argv[:i]}, the parameters are in ~')
    command = argv[i + 1 :]
    if not command:
        raise SystemExit('wait_still: no command after --')
    return command


def _interrupt(*_):
    raise KeyboardInterrupt


def wait() -> int:
    """Run the gate until it releases; return 0, or the exit code when it must not exec."""
    # the command's remappings (__name, __log) are not ours: the gate is odometry_gate in the
    # namespace roslaunch gives the command (ROS_NAMESPACE)
    rospy.init_node(NAME, argv=['wait_still'], disable_signals=True)
    signal.signal(signal.SIGTERM, _interrupt)  # exec resets it for the command
    node = None
    try:
        node = StillGateNode()
        while not node.released.wait(0.1):
            if rospy.is_shutdown():
                return INTERRUPTED
        return 0
    except ConfigError as error:
        rospy.logfatal(f'{NAME}: invalid parameters, not starting: {error}')
        return INVALID
    except KeyboardInterrupt:
        return INTERRUPTED
    finally:
        if node is not None:
            node.destroy_node()
        rospy.signal_shutdown('odometry gate done')  # unregisters before the exec


def main(argv=None) -> int:
    """Wait for stillness, then replace this process with the command."""
    command = split_args(sys.argv[1:] if argv is None else argv)
    code = wait()
    if code:
        return code
    sys.stdout.flush()
    sys.stderr.flush()
    os.execvp(command[0], command)
    return 0  # not reached


if __name__ == '__main__':
    sys.exit(main())
