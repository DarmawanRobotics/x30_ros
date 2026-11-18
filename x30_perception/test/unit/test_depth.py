"""Depth camera obstacles: floor fit, range, cell density, confirmation, blind camera."""

from __future__ import annotations

import math

from depth_scene import l1w_mount, render
import numpy as np
import pytest

from x30_perception.core.config import DepthConfig, FilterConfig, PerceptionConfig
from x30_perception.core.depth import (
    CONFIRM_MAX_GAP_SEC,
    CellGrid,
    DepthFilter,
    View,
    dense,
    fit_floor,
)
from x30_utils.math import default_rng
from x30_utils.robot import RobotProfile

ROBOT = RobotProfile(base_height=0.35)
MOUNT = l1w_mount()
NOISE = 0.0075  # depth error std per m^2: a D435 at 424 x 240
BOX = (1.5, 1.8, -0.15, 0.15, 0.15)  # 15 cm tall, below the LiDAR's lowest beam
FRAME_SEC = 1.0 / 15.0


def depth_filter(**depth) -> DepthFilter:
    """A filter of the default depth settings on a level base (Pashupati: level_source none)."""
    config = PerceptionConfig(
        depth=DepthConfig(enabled=True, **depth), filter=FilterConfig(level_source='none')
    )
    return DepthFilter(config, ROBOT)


def frames(f: DepthFilter, count=2, mount=MOUNT, scene_mount=None, start=0.0, **scene):
    """Feed count clouds of a scene one frame apart; return the last result."""
    out = None
    for i in range(count):
        cloud = render(scene_mount if scene_mount is not None else mount, seed=i, **scene)
        out = f.process(cloud, mount, None, start + i * FRAME_SEC)
    return out


def nearest(out) -> float:
    """Distance from base_link to the nearest obstacle point (inf: none)."""
    xy = out.obstacles.xy
    return float(np.hypot(xy[:, 0], xy[:, 1]).min()) if len(xy) else math.inf


def test_low_box_below_the_lidar_is_an_obstacle():
    """A 15 cm box 1.5 m ahead: obstacle points on its face, the floor around it is free."""
    out = frames(depth_filter(), boxes=[BOX], noise=NOISE)
    assert not out.reason and out.floor.fitted
    assert nearest(out) == pytest.approx(1.5, abs=0.03)
    xy = out.obstacles.xy
    assert (np.abs(xy[:, 1]) <= 0.17).all() and (xy[:, 0] >= 1.47).all()
    assert out.obstacles.height.max() == pytest.approx(0.15, abs=0.02)


@pytest.mark.parametrize('error_deg', [-3.0, 2.0, 4.0])
def test_floor_fit_keeps_the_floor_free_with_a_mount_pitch_error(error_deg):
    """The camera pitched a few degrees off its TF: the fit finds it, no ghost obstacles."""
    scene = l1w_mount(10.0 + error_deg)
    out = frames(depth_filter(), scene_mount=scene, noise=NOISE)
    assert out.floor.fitted and len(out.obstacles.xy) == 0
    pitch, roll = out.floor.mount_error_deg
    assert pitch == pytest.approx(error_deg, abs=0.1) and roll == pytest.approx(0.0, abs=0.1)
    boxed = frames(depth_filter(), scene_mount=scene, boxes=[BOX], noise=NOISE)
    assert nearest(boxed) == pytest.approx(1.5, abs=0.06)  # positions keep ~2 cm of the error


def test_without_the_floor_fit_a_pitch_error_lifts_the_floor():
    """The reason for the fit: 4 deg more pitch than the TF says puts the far floor in the band."""
    out = frames(depth_filter(floor_fit=False), scene_mount=l1w_mount(14.0), noise=NOISE)
    assert not out.floor.fitted and nearest(out) < 2.5


def test_roll_error_is_reported_with_its_sign():
    """A camera rolled 2 deg (left side up) more than its TF: the fit reports +2 deg roll."""
    out = frames(depth_filter(), scene_mount=l1w_mount(roll_deg=2.0))
    assert out.floor.fitted and len(out.obstacles.xy) == 0
    assert out.floor.mount_error_deg[1] == pytest.approx(2.0, abs=0.1)


def test_no_floor_in_view_keeps_the_nominal_floor():
    """A wall 0.8 m ahead fills the view: no fit (a line of floor points), the wall blocks."""
    out = frames(depth_filter(), boxes=[(0.8, 0.9, -3.0, 3.0, 2.0)])
    assert not out.floor.fitted and 'line' in out.floor.note
    assert nearest(out) == pytest.approx(0.8, abs=0.02)


def test_floor_fit_does_not_absorb_a_low_platform():
    """A 12 cm platform across the whole view from 1.5 m on stays an obstacle."""
    out = frames(depth_filter(), boxes=[(1.5, 5.0, -3.0, 3.0, 0.12)], noise=NOISE)
    assert out.floor.fitted and abs(out.floor.c) < 0.01
    assert nearest(out) == pytest.approx(1.5, abs=0.04)


def test_fit_refused_when_tilted_or_offset():
    """A plane 8 deg steep, or 4 cm above the nominal floor under the robot: nominal floor."""
    xs, ys = np.meshgrid(np.linspace(-0.3, 0.3, 40), np.linspace(-0.5, 0.5, 40))
    xy = np.column_stack((xs.ravel(), ys.ravel()))
    ramp = math.tan(math.radians(8.0)) * xy[:, 0]  # within the first window near base_link
    steep = fit_floor(xy, ramp, math.radians(10.0), 0.05)
    assert steep.fitted and math.degrees(steep.tilt) == pytest.approx(8.0, abs=0.01)
    steep = fit_floor(xy, ramp, math.radians(5.0), 0.05)
    assert not steep.fitted and 'tilted' in steep.note
    raised = fit_floor(xy, np.full(len(xy), 0.04), math.radians(5.0), 0.02)
    assert not raised.fitted and 'off the floor' in raised.note
    assert not fit_floor(xy[:50], np.zeros(50), 1.0, 1.0).fitted  # too few points


def test_obstacles_need_the_previous_cloud():
    """A first cloud confirms nothing; the next one within the gap does; confirm_radius 0: off."""
    f = depth_filter()
    first = f.process(render(MOUNT, boxes=[BOX]), MOUNT, None, 10.0)
    assert len(first.obstacles.xy) == 0 and not first.reason
    assert nearest(f.process(render(MOUNT, boxes=[BOX]), MOUNT, None, 10.07)) < 1.6
    late = 10.07 + CONFIRM_MAX_GAP_SEC + 0.1
    assert len(f.process(render(MOUNT, boxes=[BOX]), MOUNT, None, late).obstacles.xy) == 0
    moved = (1.62, 1.92, -0.15, 0.15, 0.15)  # 12 cm on in one frame: still the same object
    assert nearest(f.process(render(MOUNT, boxes=[moved]), MOUNT, None, late + 0.07)) < 1.7
    once = depth_filter(confirm_radius=0.0).process(render(MOUNT, boxes=[BOX]), MOUNT, None)
    assert nearest(once) == pytest.approx(1.5, abs=0.03)


def test_stereo_speckles_are_not_obstacles():
    """Wrong depths on 0.6 % of the pixels, new ones every frame: never an obstacle."""
    f = depth_filter()
    rng = default_rng(7)
    for i in range(15):
        floor = render(MOUNT, noise=NOISE, seed=i)
        wrong = floor[rng.choice(len(floor), 100, replace=False)]
        cloud = np.vstack((floor, wrong * rng.uniform(0.3, 1.0, (100, 1))))
        out = f.process(cloud, MOUNT, None, i * FRAME_SEC)
        assert len(out.obstacles.xy) == 0, f'frame {i}'
    unfiltered = depth_filter(cell_points=1, confirm_radius=0.0).process(cloud, MOUNT, None)
    assert len(unfiltered.obstacles.xy) > 0  # what the filters remove


def test_points_beyond_range_max_are_dropped():
    """A box 3.6 m from the camera is beyond range_max 3.0 m; range_max 5.0 sees it."""
    far = [(3.9, 4.2, -0.3, 0.3, 0.5)]
    assert len(frames(depth_filter(), boxes=far).obstacles.xy) == 0
    assert nearest(frames(depth_filter(range_max=5.0), boxes=far)) == pytest.approx(3.9, abs=0.05)


def test_blind_camera_is_not_merged():
    """Fewer than min_points points (covered lens, sun in the window): unusable, no history."""
    f = depth_filter()
    f.process(render(MOUNT, boxes=[BOX]), MOUNT, None, 0.0)
    out = f.process(render(MOUNT)[:150], MOUNT, None, FRAME_SEC)
    assert 'blind' in out.reason and len(out.obstacles.xy) == 0
    after = f.process(render(MOUNT, boxes=[BOX]), MOUNT, None, 2 * FRAME_SEC)
    assert len(after.obstacles.xy) == 0  # the blind cloud reset the confirmation


def test_level_source_tf_needs_the_tilt():
    """filter.level_source tf without the roll/pitch of the base: unusable (as the LiDAR)."""
    config = PerceptionConfig(depth=DepthConfig(enabled=True))
    out = DepthFilter(config, ROBOT).process(render(MOUNT), MOUNT, None)
    assert 'roll/pitch' in out.reason
    level = DepthFilter(config, ROBOT).process(render(MOUNT), MOUNT, np.eye(3))
    assert not level.reason


def test_points_on_the_robot_and_invalid_points_are_ignored():
    """Points inside the footprint (legs) and zeros (holes of an ordered cloud) are not seen."""
    legs = np.tile([[0.3, 0.2, -0.2]], (500, 1))
    cloud = np.vstack(((legs - MOUNT[:3, 3]) @ MOUNT[:3, :3], np.zeros((500, 3))))
    out = depth_filter().process(cloud, MOUNT, None)
    assert out.points == 0 and 'blind' in out.reason


def test_dense_keeps_cells_with_enough_points():
    """Three points in one 5 cm cell stay, a lone point goes."""
    xy = np.array([[1.01, 0.01], [1.02, 0.02], [1.03, 0.04], [2.0, 2.0]])
    assert dense(xy, 0.05, 3).tolist() == [True, True, True, False]
    assert dense(xy, 0.05, 1).all() and len(dense(np.empty((0, 2)), 0.05, 3)) == 0


def test_cell_grid_grows_by_the_radius():
    """A cell grown by 0.1 m covers its neighbours two cells away, not three; off-grid: no."""
    grid = CellGrid(0.05, 2.0)
    grown = grid.grown(np.array([[1.01, 0.01]]), 0.1)
    near = np.array([[1.11, 0.11], [0.91, -0.09], [1.17, 0.01], [5.0, 0.0], [-3.0, 0.0]])
    assert grid.contains(grown, near).tolist() == [True, True, False, False, False]
    assert not grid.grown(np.empty((0, 2)), 0.1).any()


def test_view_of_the_l1w_camera():
    """The floor the L1W's camera sees: ahead, a little wider than 87 deg (pitched down)."""
    out = frames(depth_filter(), count=1)
    view = out.view
    assert view.x == pytest.approx(MOUNT[0, 3]) and view.heading == pytest.approx(0.0, abs=0.01)
    assert 43.5 < math.degrees(view.high) < 50.0
    assert math.degrees(view.low) == pytest.approx(-math.degrees(view.high), abs=0.5)
    assert 0.5 < view.near < 0.65 and view.far == pytest.approx(3.0, abs=0.05)
    inside = view.contains(np.array([[1.5, 0.0], [1.5, 1.6], [0.5, 0.0], [0.3, 0.6], [4.0, 0.0]]))
    assert inside.tolist() == [True, False, False, False, False]
    behind = View.of(np.array([[-1.0, 0.1], [-2.0, -0.2]]), (0.0, 0.0))
    assert abs(behind.heading) == pytest.approx(math.pi, abs=1e-6)  # wraps around +-pi
    assert behind.contains(np.array([[-1.5, 0.0], [1.5, 0.0]])).tolist() == [True, False]
    assert View.of(np.empty((0, 2)), (0.0, 0.0)) is None
