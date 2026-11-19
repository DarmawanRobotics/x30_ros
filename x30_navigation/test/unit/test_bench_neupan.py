"""bench_neupan: NeuPAN's solves timed on this computer, and what they mean for the follower."""

from __future__ import annotations

import sys

import pytest

from x30_navigation import bench_neupan


def test_without_neupan_it_says_how_to_install_it(monkeypatch, capsys):
    monkeypatch.setitem(sys.modules, 'neupan', None)
    assert bench_neupan.main(['--solves', '3']) == 2
    assert 'install_neupan.sh' in capsys.readouterr().out


def test_the_verdict_weighs_the_slow_solves_against_the_tick_and_the_step(capsys):
    assert bench_neupan.verdict(30.0, 50.0, 100.0) == 0
    assert "within the follower's 50 ms tick" in capsys.readouterr().out
    assert bench_neupan.verdict(80.0, 50.0, 100.0) == 0
    assert 'the next one comes late' in capsys.readouterr().out
    assert bench_neupan.verdict(150.0, 50.0, 100.0) == 1
    assert 'too slow' in capsys.readouterr().out


def test_it_times_both_routes(capsys):
    pytest.importorskip('torch')
    robot = pytest.importorskip('neupan.robot').robot
    if not hasattr(robot, 'linear_holonomic_model'):
        pytest.skip('NeuPAN without the X30 patches')
    code = bench_neupan.main(['--solves', '5'])
    out = capsys.readouterr().out
    assert code in (0, 1), out  # the verdict depends on this computer
    assert 'open route     5 solves' in out and 'corridor, box  5 solves' in out, out
