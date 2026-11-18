from __future__ import annotations

import os
from pathlib import Path
import signal
import subprocess
import sys

import pytest
from std_msgs.msg import String

from x30_utils.testing.harness import Collector, wait_for

SAMPLE = Path(__file__).with_name('sample_node.py')


def start(name: str, *args: str) -> subprocess.Popen:
    env = dict(os.environ, PYTHONUNBUFFERED='1')
    return subprocess.Popen(
        [sys.executable, str(SAMPLE), f'__name:={name}', *args],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


@pytest.mark.parametrize('sig', [signal.SIGINT, signal.SIGTERM])
def test_signal_runs_the_shutdown_hook_then_exits_cleanly(ros, sig):
    out = Collector(String, 'sample/out')
    proc = start(f'sample_{sig.name.lower()}')
    try:
        assert out.wait_until(lambda m: m.data == 'ready', 15.0)
        proc.send_signal(sig)
        assert proc.wait(15.0) == 0
        assert wait_for(lambda: any(m.data == 'bye' for m in out.messages), 3.0)
    finally:
        out.close()
        if proc.poll() is None:
            proc.kill()


def test_invalid_parameters_exit_with_a_reason(ros):
    proc = start('sample_bad', '_rate:=1000.0')
    try:
        assert proc.wait(15.0) == 2
        assert 'invalid parameters' in proc.stdout.read()
    finally:
        if proc.poll() is None:
            proc.kill()


def test_a_stuck_callback_does_not_hold_the_exit(ros):
    out = Collector(String, 'sample/out')
    proc = start('sample_stuck', '_stuck:=true')
    try:
        assert out.wait_until(lambda m: m.data == 'stuck', 15.0)
        proc.send_signal(signal.SIGINT)
        assert proc.wait(15.0) == 0
        assert 'callbacks still running at shutdown' in proc.stdout.read()
    finally:
        out.close()
        if proc.poll() is None:
            proc.kill()
