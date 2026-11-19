"""tools/x30_host.sh check and x30_check.py: the master and the nodes."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess

TOOLS = Path(__file__).resolve().parents[2] / 'tools'


def test_check_on_a_laptop_with_a_loopback_ros_ip_and_no_master():
    env = {**os.environ, 'ROS_IP': '127.0.0.1', 'ROS_MASTER_URI': 'http://127.0.0.1:9'}
    out = subprocess.run(
        ['bash', str(TOOLS / 'x30_host.sh'), 'check'],
        capture_output=True,
        text=True,
        env=env,
        timeout=60,
        check=False,
    )
    assert out.returncode == 1, out.stdout + out.stderr
    assert 'ros_ip   MISS  ROS_IP 127.0.0.1: loopback' in out.stdout
    assert 'master   MISS  http://127.0.0.1:9' in out.stdout
