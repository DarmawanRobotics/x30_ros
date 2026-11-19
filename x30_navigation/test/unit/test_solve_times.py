"""Solve times: a warning while too many overrun."""

from __future__ import annotations

from x30_navigation.core.controllers.solve_times import SolveTimes


def test_late_only_when_many_solves_overrun_the_budget():
    times = SolveTimes(0.1, window=20, least=10)
    for _ in range(9):
        times.add(0.3)
    assert not times.late  # too few to judge
    assert times.last_ms == 300.0
    times.add(0.02)
    assert times.late and times.max_ms == 300.0
    assert times.warning('NeuPAN', 'fewer points') == (
        'NeuPAN solves take up to 300 ms, longer than 100 ms: fewer points'
    )
    for _ in range(19):
        times.add(0.02)
    assert times.max_ms == 20.0 and not times.late  # the slow ones left the window
    assert times.warning('NeuPAN', 'fewer points') == ''


def test_one_overrun_in_ten_is_no_warning():
    times = SolveTimes(0.1, window=50)
    for i in range(50):
        times.add(0.15 if i % 10 == 0 else 0.03)
    assert not times.late  # 10 %: a pause now and then

