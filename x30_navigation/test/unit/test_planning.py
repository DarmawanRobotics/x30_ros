"""Costmap, A*, line of sight, footprint checks and planning on a room with a door."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_navigation.core.config.planner import CostmapConfig, SearchConfig
from x30_navigation.core.planning import astar, costmap
from x30_navigation.core.planning.footprint import FootprintChecker
from x30_navigation.core.planning.line import line_cells, line_ok, shortcut
from x30_navigation.core.planning.planner import Planner, remaining_length
from x30_utils.grid import GridSpec
from x30_utils.math import Pose2D
from x30_utils.robot import Footprint

FOOTPRINT = Footprint(front=0.40, rear=0.40, half_width=0.28)
CONFIG = CostmapConfig(resolution=0.1, margin=0.05, inflation_radius=0.8)


def room_grid() -> costmap.MapGrid:
    """A 10 x 6 m room: a wall at x = 5 with a door at y = 0.5 .. 1.7, an unknown corner."""
    data = np.zeros((120, 200), dtype=np.int8)
    data[0, :] = data[-1, :] = 100
    data[:, 0] = data[:, -1] = 100
    data[:, 100] = 100
    data[10:34, 100] = 0
    data[100:, 150:] = -1
    return costmap.MapGrid(data, GridSpec(0.05, 0.0, 0.0, 200, 120))


def planner_for(grid, footprint=FOOTPRINT, config=CONFIG, search=None) -> Planner:
    cm = costmap.build_costmap(grid, footprint, config)
    return Planner(cm, FootprintChecker(cm, footprint, config.margin), search or SearchConfig())


def test_radii_come_from_the_footprint():
    """N1: lethal = inscribed radius + margin, the oriented check reaches the corners."""
    radii = costmap.Radii.of(FOOTPRINT, CONFIG)
    assert radii.lethal == pytest.approx(0.28 + 0.05)
    assert radii.circumscribed == pytest.approx(math.hypot(0.40, 0.28) + 0.05)
    assert radii.inflation >= radii.circumscribed


def test_inflation_is_lethal_near_walls_and_decays():
    data = np.zeros((40, 40), dtype=np.int8)
    data[:, 20] = 100
    grid = costmap.MapGrid(data, GridSpec(0.05, 0.0, 0.0, 40, 40))
    cm = costmap.build_costmap(grid, FOOTPRINT, CostmapConfig(resolution=0.05, margin=0.05))
    assert math.isinf(cm.cost_at(1.02, 1.0)) and math.isinf(cm.cost_at(0.75, 1.0))
    near, far = cm.cost_at(0.6, 1.0), cm.cost_at(0.4, 1.0)
    assert near > far > 1.0
    assert math.isinf(cm.cost_at(-1.0, 1.0))


def test_extra_obstacles_and_unknown():
    data = np.zeros((30, 30), dtype=np.int8)
    data[:, 25:] = -1
    grid = costmap.MapGrid(data, GridSpec(0.1, 0.0, 0.0, 30, 30))
    blocked = costmap.build_costmap(grid, FOOTPRINT, CONFIG, extra=np.array([[1.5, 1.5]]))
    assert math.isinf(blocked.cost_at(1.55, 1.55)) and math.isinf(blocked.cost_at(2.8, 1.0))
    open_cfg = CostmapConfig(resolution=0.1, unknown_is_obstacle=False)
    assert math.isfinite(costmap.build_costmap(grid, FOOTPRINT, open_cfg).cost_at(2.8, 1.0))


def test_oriented_footprint_against_a_wall():
    """N1: 0.45 m from a wall the long side fits, the nose (0.40 + margin) does not."""
    data = np.zeros((60, 60), dtype=np.int8)
    data[:, 40] = 100  # wall at x = 2.0
    grid = costmap.MapGrid(data, GridSpec(0.05, 0.0, 0.0, 60, 60))
    cm = costmap.build_costmap(grid, FOOTPRINT, CostmapConfig(resolution=0.05, margin=0.05))
    checker = FootprintChecker(cm, FOOTPRINT, 0.05)
    beside = Pose2D(2.0 - 0.37, 1.5, math.pi / 2)
    facing = Pose2D(2.0 - 0.37, 1.5, 0.0)
    assert math.isfinite(cm.cost_at(beside.x, beside.y))
    assert not checker.collides(beside)
    assert checker.collides(facing)


def test_astar_door_and_no_corner_cutting():
    cost = np.ones((20, 20))
    cost[:, 10] = np.inf
    cost[14, 10] = 1.0
    path = astar.astar(cost, (2, 2), (2, 18))
    assert (14, 10) in path
    for (r0, c0), (r1, c1) in zip(path, path[1:]):
        if r0 != r1 and c0 != c1:
            assert math.isfinite(cost[r0, c1]) and math.isfinite(cost[r1, c0])
    cost[14, 10] = np.inf
    assert astar.astar(cost, (2, 2), (2, 18)) is None
    with pytest.raises(astar.SearchTimeoutError):
        astar.astar(np.ones((60, 60)), (0, 0), (59, 59), max_expansions=10)


def test_nearest_free_with_acceptance():
    cost = np.ones((20, 20))
    cost[5:10, 5:10] = np.inf
    assert astar.nearest_free(cost, (3, 3), 2) == (3, 3)
    assert astar.nearest_free(cost, (7, 9), 3) == (7, 10)
    assert astar.nearest_free(cost, (7, 7), 2) is None
    assert astar.nearest_free(cost, (3, 3), 3, accept=lambda c: c[1] < 3) == (3, 2)


def test_line_of_sight_blocks_diagonal_pinches():
    """F17: a line through the corner between two diagonal obstacles is not clear."""
    cost = np.ones((5, 5))
    cost[1, 2] = cost[2, 1] = np.inf
    assert not line_ok(cost, (1, 1), (2, 2), 10.0)
    assert line_ok(np.ones((5, 5)), (1, 1), (2, 2), 10.0)
    rows, cols = line_cells((1, 1), (4, 9))
    assert (rows[0], cols[0]) == (1, 1) and len(rows) == len(cols)
    assert (4, 9) in set(zip(rows, cols))


def test_shortcut_never_crosses_costlier_cells():
    cost = np.ones((30, 30))
    cost[0:20, 15] = np.inf
    cost[0:20, 14] = 4.0
    path = astar.astar(cost, (2, 2), (2, 28))
    short = shortcut(cost, path)
    assert short[0] == path[0] and short[-1] == path[-1] and len(short) < len(path)
    worst = max(cost[c] for c in path)
    for a, b in zip(short, short[1:]):
        if max(abs(b[0] - a[0]), abs(b[1] - a[1])) <= 1:
            continue  # a step of the search itself
        rows, cols = line_cells(a, b)
        assert np.all(np.isfinite(cost[rows, cols])) and cost[rows, cols].max() <= worst


def test_plan_through_the_door_with_the_footprint():
    planner = planner_for(room_grid())
    plan = planner.plan(Pose2D(2.0, 4.0, 0.0), Pose2D(8.0, 4.0, math.pi / 2))
    assert plan.ok, plan.message
    crossing = [w for w in plan.waypoints if 4.9 < w.x < 5.1]
    assert crossing and all(0.5 < w.y < 1.7 for w in crossing)
    assert (plan.waypoints[0].x, plan.waypoints[0].y) == pytest.approx((2.0, 4.0))
    assert plan.waypoints[-1].yaw == pytest.approx(math.pi / 2)
    poses = [Pose2D(w.x, w.y, w.yaw) for w in plan.waypoints]
    assert planner.checker.first_collision(poses, 1) is None
    gaps = np.hypot(*np.diff([(w.x, w.y) for w in plan.waypoints], axis=0).T)
    assert gaps.max() < 0.15 and plan.length > 6.0


def test_door_narrower_than_the_body_is_refused():
    """N1: the old 0.35 m radius let a 0.8 m wide robot plan through a 0.6 m door."""
    grid = room_grid()
    grid.data[10:34, 100] = 100
    grid.data[20:32, 100] = 0  # 0.6 m door
    plan = planner_for(grid).plan(Pose2D(2.0, 4.0, 0.0), Pose2D(8.0, 4.0, 0.0))
    assert not plan.ok


def test_snapped_goal_is_where_the_path_ends():
    """F8: the plan reports the snapped goal so arrival is tested against it."""
    planner = planner_for(room_grid())
    snapped = planner.plan(Pose2D(2.0, 4.0, 0.0), Pose2D(4.9, 4.0, 0.0))
    assert snapped.ok and 'moved' in snapped.message
    end = snapped.waypoints[-1]
    assert (snapped.goal.x, snapped.goal.y) == pytest.approx((end.x, end.y))
    assert snapped.goal.x < 4.6
    search = SearchConfig(goal_tolerance=0.1)
    assert not planner_for(room_grid(), search=search).plan(Pose2D(2, 4, 0), Pose2D(5, 4, 0)).ok


def test_start_escape_and_outside():
    planner = planner_for(room_grid())
    assert planner.plan(Pose2D(4.8, 4.0, 0.0), Pose2D(2.0, 2.0, 0.0)).ok
    assert not planner.plan(Pose2D(-3.0, 1.0, 0.0), Pose2D(2.0, 2.0, 0.0)).ok
    outside = planner.plan(Pose2D(2.0, 4.0, 0.0), Pose2D(20.0, 4.0, 0.0))
    assert not outside.ok and 'outside the map' in outside.message


def test_short_plan_and_remaining_length():
    short = planner_for(room_grid()).plan(Pose2D(2.0, 2.0, 0.0), Pose2D(2.05, 2.0, 1.0))
    assert short.ok and len(short.waypoints) >= 2
    assert short.waypoints[-1].yaw == pytest.approx(1.0)
    path = np.array([(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)])
    assert remaining_length(path, 1.0, 0.1) == pytest.approx(1.1)
    assert remaining_length(np.zeros((0, 2)), 0.0, 0.0) == 0.0
