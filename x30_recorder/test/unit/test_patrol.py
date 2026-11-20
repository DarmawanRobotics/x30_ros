"""Patrol: recording while a route runs, a grace for laps and stops, nothing on silence."""

from __future__ import annotations

from x30_recorder.core.patrol import Patrol

INACTIVE, FOLLOWING, DWELLING, PAUSED, GOAL_REACHED = 0, 2, 4, 5, 8


def test_a_route_running_paused_or_dwelling_is_a_patrol():
    p = Patrol(stop_after_sec=30.0, silence_sec=60.0)
    assert not p.wanted(0.0)
    p.update(INACTIVE, 0.0)
    assert not p.wanted(1.0)
    for t, state in ((2.0, FOLLOWING), (10.0, DWELLING), (20.0, PAUSED)):
        p.update(state, t)
        assert p.wanted(t)


def test_the_route_ended_stops_after_the_grace_and_a_lap_continues():
    p = Patrol(stop_after_sec=30.0, silence_sec=60.0)
    p.update(FOLLOWING, 0.0)
    p.update(GOAL_REACHED, 5.0)
    assert p.wanted(29.0)  # the next lap may start
    p.update(FOLLOWING, 29.0)
    assert p.wanted(40.0)
    p.update(GOAL_REACHED, 50.0)
    assert p.wanted(79.0) and not p.wanted(81.0)  # 30 s after the route ended


def test_a_silent_navigation_ends_the_patrol_a_quiet_route_does_not():
    p = Patrol(stop_after_sec=30.0, silence_sec=5.0)
    p.update(FOLLOWING, 0.0)
    assert p.wanted(4.0) and not p.wanted(6.0)
    quiet = Patrol(stop_after_sec=1.0, silence_sec=600.0)  # a status sent only on change
    quiet.update(FOLLOWING, 0.0)
    assert quiet.wanted(500.0)
