"""The offset to NETRA's clock from time_sync exchanges (NTP style)."""

from __future__ import annotations

import pytest

from x30_fleet.core.clock import ClockOffset, answer


def test_offset_and_round_trip_of_one_exchange():
    clock = ClockOffset()
    # NETRA is 2 s ahead; 0.1 s each way, 0.05 s inside NETRA
    assert clock.add(t0=100.0, t1=102.1, t2=102.15, t3=100.25)
    offset, rtt = clock.best()
    assert offset == pytest.approx(2.0) and rtt == pytest.approx(0.2)


def test_the_shortest_round_trip_wins_and_impossible_answers_are_ignored():
    clock = ClockOffset()
    clock.add(0.0, 0.2, 0.2, 0.8)  # slow, asymmetric exchange: offset -0.2, off
    clock.add(10.0, 9.55, 9.55, 10.1)  # fast: offset -0.5
    assert clock.best()[0] == pytest.approx(-0.5)
    assert not clock.add(5.0, 1.0, 2.0, 4.0)  # answered before asked
    assert not clock.add(5.0, 2.0, 1.0, 6.0)  # sent before received
    assert len(clock._samples) == 2


def test_report_only_with_a_recent_answer():
    clock = ClockOffset()
    assert clock.report(100.0, 30.0) is None
    clock.add(50.0, 50.3, 50.3, 50.2)
    assert clock.report(100.0, 30.0) is None  # 50 s old
    report = clock.report(60.0, 30.0)
    assert report == {'offset_ms': 200.0, 'rtt_ms': 200.0, 'age_sec': 9.8}


def test_answer_echoes_t0():
    assert answer({'type': 'time_sync', 't0': 1.5}, 2.0, 2.1) == {
        'type': 'time_sync', 't0': 1.5, 't1': 2.0, 't2': 2.1}  # fmt: skip
