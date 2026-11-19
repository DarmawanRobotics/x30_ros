"""Solve times: the latest in the follower's status, a warning while too many overrun."""

from __future__ import annotations

from rig import Rig
from sim import route_of, straight_route

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers import ControllerLimits
from x30_navigation.core.controllers.holonomic import HolonomicController
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


class Timed(HolonomicController):
    """A controller that reports a solve time and a standing warning."""

    def compute(self, request):
        command = super().compute(request)
        self.debug['solve_ms'] = 23.4
        self.warning = 'solves too slow'
        return command


def test_the_status_shows_the_latest_solve():
    rig = Rig(
        route_of(straight_route(0.0, 4.0)), config=FollowerConfig(require_localization=False)
    )
    controller = Timed(None, ControllerLimits(0.8, 0.4, 1.0))
    rig.follower.set_controller('timed', controller)
    assert rig.follower.start(rig.pose)[0]
    rig.run(2.0)
    reports = [d.report.message for d in rig.decisions]
    assert any('| timed 23 ms |' in m for m in reports), reports[-1]
    assert rig.follower.controller.warning == 'solves too slow'  # the node logs it, throttled
