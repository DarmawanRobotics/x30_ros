"""docking_node stops cleanly on SIGINT (once, or twice as terminal + launch) while it ticks."""

from __future__ import annotations

import os
import signal
import subprocess
import sys
import time

NODE = 'from x30_navigation.docking_node import main; main()'
MARKERS = ('Traceback', 'Unable to convert', 'failed')
RUNS = 6


def _start(index: int):
    args = ['_control_rate:=200.0', '_control_node:=', f'__name:=docking_sig_{index}']
    return subprocess.Popen(
        [sys.executable, '-c', NODE, *args],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=dict(os.environ, PYTHONUNBUFFERED='1'),
    )


def _stop(proc, delay: float, twice: bool):
    """Wait for 'docking ready', let it tick for delay, SIGINT it; (exit code, output)."""
    lines = []
    for line in proc.stdout:
        lines.append(line)
        if 'docking ready' in line:
            break
    time.sleep(delay)
    proc.send_signal(signal.SIGINT)
    if twice:
        time.sleep(0.05)
        proc.send_signal(signal.SIGINT)
    out, _ = proc.communicate(timeout=15.0)
    return proc.returncode, ''.join(lines) + out


def test_docking_node_stops_cleanly_on_sigint(ros):
    procs = [_start(i) for i in range(RUNS)]
    try:
        results = [_stop(proc, 0.3 + 0.1 * i, i % 2 == 1) for i, proc in enumerate(procs)]
    finally:
        for proc in procs:  # a failed run must not leave nodes behind
            if proc.poll() is None:
                proc.kill()
                proc.communicate()
    for code, out in results:
        assert code == 0, out
        assert 'docking ready' in out, out
        assert not [m for m in MARKERS if m in out], out
