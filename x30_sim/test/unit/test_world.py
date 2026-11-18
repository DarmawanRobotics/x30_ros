"""World: ray casting against walls and circles (inside and outside), collisions, loading."""

from __future__ import annotations

import math

import numpy as np
import pytest

from x30_sim.core.world import World, WorldError, cast_rays, circle_distances, collides
from x30_utils.math import Pose2D, quat_from_rpy, quat_to_matrix

BOX = World.from_dict(
    {
        'walls': [[[-2, -2], [2, -2], [2, 2], [-2, 2], [-2, -2]]],
        'pillars': [[1.0, 0.0, 0.25]],
    }
)
LO, HI = (-0.3, -0.2), (0.3, 0.2)  # a small footprint


def test_rays_hit_walls_and_pillars():
    """From the centre: the pillar ahead at 0.75 m, walls at 2 m elsewhere."""
    d = cast_rays(BOX, 0.0, 0.0, np.array([0.0, math.pi / 2, math.pi, -math.pi / 2]), 10.0)
    assert np.allclose(d, [0.75, 2.0, 2.0, 2.0])


def test_dynamic_obstacles_and_max_range():
    """Extra circles are seen; empty space reads max range."""
    angles = np.array([0.0, math.pi])
    assert np.allclose(cast_rays(World(), 0, 0, angles, 5.0), [5.0, 5.0])
    d = cast_rays(World(), 0, 0, angles, 5.0, np.array([[-3.0, 0.0, 0.5, 1.7]]))
    assert np.allclose(d, [5.0, 2.5])


def test_ray_from_inside_a_circle_sees_its_far_side():
    """#10: an origin inside a circle used to miss it (t = -b - sqrt(disc) < 0): blind."""
    circle = np.array([[0.0, 0.0, 0.5, 1.7]])
    angles = np.linspace(-math.pi, math.pi, 16, endpoint=False)
    inside = circle_distances(circle, 0.1, 0.0, angles)[:, 0]
    assert np.all(np.isfinite(inside))
    assert inside[8] == pytest.approx(0.4)  # angle 0: towards the near edge at x = 0.5
    assert inside[0] == pytest.approx(0.6)  # angle -pi: the far edge at x = -0.5
    outside = circle_distances(circle, -2.0, 0.0, np.array([0.0, math.pi]))[:, 0]
    assert outside[0] == pytest.approx(1.5) and math.isinf(outside[1])


def test_tangent_and_behind_circles():
    """A ray grazing a circle hits it at the tangent point; circles behind are missed."""
    circle = np.array([[2.0, 1.0, 1.0, 1.0]])
    assert circle_distances(circle, 0.0, 0.0, np.array([0.0]))[0, 0] == pytest.approx(2.0)
    assert math.isinf(circle_distances(circle, 0.0, 0.0, np.array([math.pi]))[0, 0])


def test_collisions_with_walls_and_circles():
    """The footprint box touches walls and circles only when they reach it."""
    assert not collides(BOX, Pose2D(0.0, 0.0, 0.0), LO, HI)
    assert collides(BOX, Pose2D(0.5, 0.0, 0.0), LO, HI)  # pillar edge at 0.75 m, front 0.8
    assert collides(BOX, Pose2D(-1.75, 0.0, 0.0), LO, HI)  # rear at -2.05: through the wall
    assert not collides(BOX, Pose2D(-1.75, 0.0, math.pi / 2), LO, HI)  # turned: 0.2 m wide
    assert collides(BOX, Pose2D(-1.0, 1.0, 0.0), LO, HI, np.array([[-1.0, 1.4, 0.25, 1.7]]))


def test_world_file_loads(tmp_path):
    """Polylines become segments with heights, start yaw is in degrees, tags are wall plates."""
    f = tmp_path / 'w.yaml'
    f.write_text(
        'walls:\n  - [[0, 0], [1, 0], [1, 1]]\n  - {points: [[3, 3], [4, 3]], height: 0.4}\n'
        'pillars: [[2, 2, 0.3], [5, 5, 0.2, 0.5]]\nstart: [1, 2, 90]\n'
        'tags: {base_map: [1.0, 0.5, 1.2, 180.0]}\n'
    )
    world = World.from_yaml(str(f))
    assert world.segments.shape == (3, 5) and world.circles.shape == (2, 4)
    assert world.segments[2, 4] == pytest.approx(0.4) and world.circles[1, 3] == 0.5
    assert world.start.yaw == pytest.approx(math.pi / 2)
    tag = world.tags['base_map']
    assert tag[:3] == (1.0, 0.5, 1.2)
    normal = quat_to_matrix(tag[3:])[:, 2]  # the plate faces -x (180 deg)
    assert normal == pytest.approx([-1.0, 0.0, 0.0], abs=1e-9)
    assert tag[3:] == pytest.approx(
        quat_from_rpy(-math.pi / 2, 0.0, math.pi / 2)
    )  # as localization


@pytest.mark.parametrize(
    'text',
    ['walls: [[[0, 0]]]', 'pillars: [[1, 2]]', 'start: [a, 0, 0]', 'walls: [[[0, .nan], [1, 1]]]'],
)
def test_bad_world_files_raise(tmp_path, text):
    """Broken worlds raise WorldError with the path."""
    f = tmp_path / 'bad.yaml'
    f.write_text(text + '\n')
    with pytest.raises(WorldError, match=r'bad\.yaml'):
        World.from_yaml(str(f))
