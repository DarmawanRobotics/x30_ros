"""wait_still: arguments, the gate on a fake IMU, the exec, signals and bad parameters."""

from __future__ import annotations

import itertools
import math
import os
import signal
import subprocess
import sys

import pytest
import rospy
from sensor_msgs.msg import Imu

from x30_odometry.wait_still import StillGateNode, split_args
from x30_utils.testing.harness import connected, wait_for

NAMES = itertools.count()
GATE = 'import sys; from x30_odometry.wait_still import main; sys.exit(main(sys.argv[1:]))'


def test_split_args():
    assert split_args(['--', '/bin/fl', '__name:=fast_lio', '__log:=x']) == [
        '/bin/fl',
        '__name:=fast_lio',
        '__log:=x',
    ]


@pytest.mark.parametrize('argv', [['x'], ['--'], ['--imu', '/i', '--', 'x'], []])
def test_split_args_rejects_bad_usage(argv):
    with pytest.raises(SystemExit):
        split_args(argv)


def imu(tilt_deg: float = 0.0, gyro: float = 0.0) -> Imu:
    msg = Imu()
    t = math.radians(tilt_deg)
    msg.linear_acceleration.x, msg.linear_acceleration.z = math.sin(t), math.cos(t)
    msg.angular_velocity.z = gyro
    return msg


@pytest.mark.parametrize(('tilt_deg', 'released'), [(0.0, True), (5.0, False)])
def test_gate_node(ros, tilt_deg, released):
    """Level and still: released; 5 deg tilted (above the 3 deg default): held."""
    n = next(NAMES)
    topic = f'/gate_test_{n}/imu'
    gate = StillGateNode(
        param_ns=f'/gate_test_{n}/odometry_gate',
        overrides={'imu_topic': topic, 'still_sec': 0.5, 'timeout_sec': 0.0},
    )
    pub = rospy.Publisher(topic, Imu, queue_size=50)
    try:
        assert connected(pub)
        rate = rospy.Rate(200)
        for _ in range(300):  # 1.5 s
            pub.publish(imu(tilt_deg))
            rate.sleep()
        if released:
            assert wait_for(lambda: gate.done, timeout=0.5)
        else:  # judged while the samples come in: without them the window empties
            assert not gate.done
            assert gate.decision.reason.startswith('tilted')
    finally:
        pub.unregister()
        gate.destroy_node()


def gate_process(ns: str, marker, **params) -> subprocess.Popen:
    """Run wait_still in ns with params; its command writes marker."""
    for key, value in params.items():
        rospy.set_param(f'{ns}/odometry_gate/{key}', value)
    command = [sys.executable, '-c', f'open({str(marker)!r}, "w").write("ok")']
    return subprocess.Popen(
        [sys.executable, '-c', GATE, '--', *command],
        env=dict(os.environ, ROS_NAMESPACE=ns, PYTHONUNBUFFERED='1'),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def test_timeout_execs_the_command(ros, tmp_path):
    """Without IMU data the gate gives up after timeout_sec and execs the command."""
    marker = tmp_path / 'started'
    proc = gate_process('/wait_still_exec', marker, imu_topic='/no_imu', timeout_sec=0.5)
    try:
        out, _ = proc.communicate(timeout=30)
    finally:
        proc.kill()
    assert proc.returncode == 0, out
    assert marker.read_text() == 'ok'
    assert 'starting odometry anyway' in out


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_signal_while_held_exits_without_the_command(ros, tmp_path, sig):
    """Ctrl-C or SIGTERM (roslaunch stopping) while the start is held: exit 130, no command."""
    marker = tmp_path / 'started'
    ns = f'/wait_still_signal_{sig.name.lower()}'
    proc = gate_process(ns, marker, imu_topic='/no_imu', timeout_sec=60.0)
    try:
        for line in proc.stdout:  # the gate logs once it is up
            if 'holding the odometry start' in line:
                break
        proc.send_signal(sig)
        assert proc.wait(timeout=20) == 130
    finally:
        proc.kill()
        proc.stdout.close()
    assert not marker.exists()


def test_invalid_parameters_exit_without_the_command(ros, tmp_path):
    marker = tmp_path / 'started'
    proc = gate_process('/wait_still_invalid', marker, still_sec=0.0)  # below the minimum
    try:
        out, _ = proc.communicate(timeout=30)
    finally:
        proc.kill()
    assert proc.returncode == 2, out
    assert 'invalid parameters' in out
    assert not marker.exists()
