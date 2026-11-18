"""The cloud -> sector scans pipeline: blind -> BLOCKED, leveling, noise, memory, depth camera."""

from __future__ import annotations

import math

from depth_scene import l1w_mount, render
import numpy as np
import pytest

from x30_perception.core.config import (
    BlindConfig,
    DepthConfig,
    FilterConfig,
    MemoryConfig,
    PerceptionConfig,
)
from x30_perception.core.depth import DepthCloud, DepthFilter, View
from x30_perception.core.filtering import Obstacles
from x30_perception.core.pipeline import Frame, SectorPipeline
from x30_perception.core.sectors import BLOCKED
from x30_utils.math import Pose2D, make_transform, quat_from_rpy
from x30_utils.robot import RobotProfile

ROBOT = RobotProfile(base_height=0.35)
MOUNT = make_transform((0.13, 0.0, 0.18))
LEVEL = np.eye(3)
ORIGIN = Pose2D()


def walls(radius: float = 3.0, n: int = 720) -> np.ndarray:
    """A round room around the sensor (sensor frame), 0.2 m above the floor."""
    a = np.linspace(-math.pi, math.pi, n, endpoint=False)
    z = 0.2 - 0.53
    return np.column_stack((radius * np.cos(a), radius * np.sin(a), np.full(n, z)))


def frame(xyz, tags=None, tilt=LEVEL, odom=ORIGIN, stamp=0.0) -> Frame:
    """A frame from the level test mount."""
    return Frame(np.asarray(xyz, float), MOUNT, stamp, tags, tilt, odom)


def test_room_gives_ranges_in_both_scans():
    """A round room 3 m from the sensor is 3 m away in every safety sector (base frame)."""
    pipeline = SectorPipeline(PerceptionConfig(), ROBOT)
    result = pipeline.process(frame(walls()))
    assert not result.blocked_reason
    assert len(result.safety) == 72 and len(result.front) == 32
    assert result.safety.min() == pytest.approx(2.87, abs=0.01)  # behind: 3 - 0.13
    assert result.front.max() <= 3.14


def test_blind_lidar_blocks_every_sector():
    """Too few points: both scans are BLOCKED in every sector."""
    pipeline = SectorPipeline(PerceptionConfig(), ROBOT)
    result = pipeline.process(frame(walls(n=50)))
    assert 'blind' in result.blocked_reason
    assert set(result.safety.tolist()) == {BLOCKED} and set(result.front.tolist()) == {BLOCKED}
    assert len(pipeline.blocked('x').safety) == 72


def test_points_on_the_robot_do_not_count_as_seeing():
    """A cloud of only the robot's own legs is blind, not a clear field."""
    legs = np.tile([[0.2, 0.2, -0.4]], (500, 1)) - MOUNT[:3, 3]
    assert 'blind' in SectorPipeline(PerceptionConfig(), ROBOT).process(frame(legs)).blocked_reason


def test_missing_leveling_blocks_unless_disabled():
    """level_source tf and no odom->base roll/pitch: BLOCKED; level_source none: processed."""
    unlevel = SectorPipeline(PerceptionConfig(), ROBOT).process(frame(walls(), tilt=None))
    assert 'roll/pitch' in unlevel.blocked_reason
    config = PerceptionConfig(filter=FilterConfig(level_source='none'))
    assert not SectorPipeline(config, ROBOT).process(frame(walls(), tilt=None)).blocked_reason


def test_pitched_body_floor_is_not_an_obstacle():
    """Body pitched 2 deg down: the floor seen by the sensor stays out of the scans."""
    pitch = math.radians(2.0)
    tilt = make_transform(quat=quat_from_rpy(0.0, pitch, 0.0))[:3, :3]
    xs, ys = np.meshgrid(np.linspace(-6, 6, 121), np.linspace(-6, 6, 121))
    floor_level = np.column_stack((xs.ravel(), ys.ravel(), np.full(xs.size, -0.35)))
    floor_level = floor_level[np.hypot(floor_level[:, 0], floor_level[:, 1]) > 1.0]
    sensor = floor_level @ tilt - MOUNT[:3, 3]  # level -> base_link -> sensor
    config = PerceptionConfig(memory=MemoryConfig(enabled=False))
    result = SectorPipeline(config, ROBOT).process(frame(sensor, tilt=tilt))
    assert not result.blocked_reason
    assert (result.front == 8.0).all() and (result.safety == 4.0).all()
    unleveled = SectorPipeline(config, ROBOT).process(frame(sensor, tilt=LEVEL))
    assert (unleveled.front < 8.0).any()  # the old behaviour: floor ahead read as obstacle


def test_tagged_noise_is_dropped():
    """Low-confidence particle points (dust) in front do not block; normal points do."""
    dust = np.tile([[1.0, 0.0, -0.2]], (5, 1))
    cloud = np.vstack((walls(), dust))
    tags = np.zeros(len(cloud), np.uint8)
    tags[-5:] = 0b1000  # bits 2-3 = 2: low confidence, rain / fog / dust
    pipeline = SectorPipeline(PerceptionConfig(), ROBOT)
    assert pipeline.process(frame(cloud, tags)).front.min() > 2.5
    kept = PerceptionConfig(filter=FilterConfig(drop_particle_noise=False))
    assert SectorPipeline(kept, ROBOT).process(frame(cloud, tags)).front.min() < 1.2


def test_memory_keeps_a_low_box_in_the_safety_scan():
    """A low box seen 1 m ahead stays in the safety scan when the next cloud misses it."""
    box = np.array([[1.0 - 0.13, y, 0.12 - 0.53] for y in (-0.05, 0.0, 0.05)])
    pipeline = SectorPipeline(PerceptionConfig(), ROBOT)
    first = pipeline.process(frame(np.vstack((walls(), box)), odom=Pose2D(), stamp=0.0))
    assert first.safety.min() == pytest.approx(1.0, abs=0.01)
    later = pipeline.process(frame(walls(), odom=Pose2D(0.3, 0.0, 0.0), stamp=0.5))
    assert later.safety.min() == pytest.approx(0.7, abs=0.06)  # remembered, robot moved 0.3 m
    assert later.front.min() > 2.5  # memory only feeds the safety scan
    gone = pipeline.process(frame(walls(), odom=Pose2D(0.3, 0.0, 0.0), stamp=11.0))
    assert gone.safety.min() > 2.5  # decayed after memory_sec


def test_memory_is_cleared_on_an_odometry_jump():
    """odom jumping 5 m in 0.1 s (restart) forgets the memory instead of misplacing it."""
    box = np.array([[1.0 - 0.13, y, 0.12 - 0.53] for y in (-0.05, 0.0, 0.05)])
    pipeline = SectorPipeline(PerceptionConfig(), ROBOT)
    pipeline.process(frame(np.vstack((walls(), box)), odom=Pose2D(), stamp=0.0))
    jumped = pipeline.process(frame(walls(), odom=Pose2D(-5.0, 0.0, 0.0), stamp=0.1))
    assert len(pipeline.memory) == 0 or jumped.safety.min() > 2.5
    no_odom = SectorPipeline(PerceptionConfig(), ROBOT)
    result = no_odom.process(frame(np.vstack((walls(), box)), odom=None))
    assert len(no_odom.memory) == 0 and result.safety.min() == pytest.approx(1.0, abs=0.01)


def test_region_blindness_from_config():
    """blind.min_region_points catches a covered half of the view."""
    cloud = walls()
    half = cloud[cloud[:, 1] > 0.0]
    config = PerceptionConfig(blind=BlindConfig(min_points=10, regions=4, min_region_points=20))
    assert 'region' in SectorPipeline(config, ROBOT).process(frame(half)).blocked_reason
    assert not SectorPipeline(config, ROBOT).process(frame(cloud)).blocked_reason


AHEAD = View(0.26, 0.0, 0.0, -0.76, 0.76, 0.55, 3.0)  # a D435 pitched down, looking ahead


def camera(xy, height=0.15, view=AHEAD) -> DepthCloud:
    """A depth cloud with obstacle points at xy (level base frame)."""
    xy = np.asarray(xy, dtype=float).reshape(-1, 2)
    return DepthCloud(Obstacles(xy, np.full(len(xy), height)), points=1000, view=view)


def test_depth_obstacles_join_both_scans():
    """A low box only the camera sees, 1.2 m ahead: in the front and the safety scan."""
    config = PerceptionConfig(
        depth=DepthConfig(enabled=True, memory_sec=0.0), memory=MemoryConfig(enabled=False)
    )
    pipeline = SectorPipeline(config, ROBOT)
    box = [[1.2, y] for y in (-0.05, 0.0, 0.05)]
    result = pipeline.process(frame(walls()), camera(box))
    assert not result.blocked_reason
    assert result.front.min() == pytest.approx(1.2, abs=0.01)
    assert result.safety.min() == pytest.approx(1.2, abs=0.01)
    assert len(result.depth) == 72 and result.depth.min() == pytest.approx(1.2, abs=0.01)
    lidar = pipeline.process(frame(walls()), None)
    assert lidar.front.min() > 2.5 and len(lidar.depth) == 0


def test_unusable_depth_falls_back_to_the_lidar_unless_required():
    """A stale or blind camera: the LiDAR alone; depth.required: every sector BLOCKED."""
    stale = DepthCloud.unusable('no cloud for 0.6 s')
    config = PerceptionConfig(depth=DepthConfig(enabled=True))
    result = SectorPipeline(config, ROBOT).process(frame(walls()), stale)
    assert not result.blocked_reason and result.front.min() > 2.5
    required = PerceptionConfig(depth=DepthConfig(enabled=True, required=True))
    blocked = SectorPipeline(required, ROBOT).process(frame(walls()), stale)
    assert 'depth camera: no cloud for 0.6 s' in blocked.blocked_reason
    assert set(blocked.safety.tolist()) == {BLOCKED}
    missing = SectorPipeline(required, ROBOT).process(frame(walls()), None)
    assert 'depth camera' in missing.blocked_reason
    ok = SectorPipeline(required, ROBOT).process(frame(walls()), camera([[1.0, 0.0]]))
    assert not ok.blocked_reason


def passed_box(pipeline):
    """The camera sees a low box ahead on the left; 3 s later the robot is beside it."""
    box = [[1.0, y] for y in (0.40, 0.45, 0.50)]
    pipeline.process(frame(walls(), odom=Pose2D(), stamp=0.0), camera(box, 0.12))
    return pipeline.process(frame(walls(), odom=Pose2D(0.9, 0.0, 0.0), stamp=3.0), camera([]))


def test_depth_memory_keeps_a_passed_box_beside_the_robot():
    """A box below the LiDAR and out of the camera's view stays in the safety scan for 6 s."""
    config = PerceptionConfig(depth=DepthConfig(enabled=True), memory=MemoryConfig(enabled=False))
    pipeline = SectorPipeline(config, ROBOT)
    beside = passed_box(pipeline)
    assert beside.safety.min() == pytest.approx(math.hypot(0.1, 0.4), abs=0.06)
    assert beside.front.min() > 2.5
    silent = DepthCloud.unusable('no cloud for 1.0 s')  # a silent camera clears nothing
    kept = pipeline.process(frame(walls(), odom=Pose2D(0.9, 0.0, 0.0), stamp=3.2), silent)
    assert kept.safety.min() < 0.5
    gone = pipeline.process(frame(walls(), odom=Pose2D(0.9, 0.0, 0.0), stamp=6.5), silent)
    assert gone.safety.min() > 2.5
    off = SectorPipeline(
        PerceptionConfig(
            depth=DepthConfig(enabled=True, memory_sec=0.0), memory=MemoryConfig(enabled=False)
        ),
        ROBOT,
    )
    assert passed_box(off).safety.min() > 2.5


def test_depth_memory_forgets_what_the_camera_sees_free():
    """The camera looks at the box's place again and sees nothing: forgotten for good."""
    config = PerceptionConfig(depth=DepthConfig(enabled=True), memory=MemoryConfig(enabled=False))
    pipeline = SectorPipeline(config, ROBOT)
    passed_box(pipeline)
    toward = math.atan2(0.45, 0.1 - 0.26)
    seeing = View(0.26, 0.0, toward, -0.3, 0.3, 0.1, 3.0)
    pose = Pose2D(0.9, 0.0, 0.0)
    seen = pipeline.process(frame(walls(), odom=pose, stamp=3.1), camera([], view=seeing))
    assert seen.safety.min() > 2.5
    later = pipeline.process(frame(walls(), odom=pose, stamp=3.2), camera([]))
    assert later.safety.min() > 2.5 and len(pipeline.depth_memory) == 0


def test_a_low_box_stays_in_the_safety_scan_while_the_robot_passes_it():
    """Driving past a 15 cm box beside the path with a synthetic L1W camera."""
    config = PerceptionConfig(
        depth=DepthConfig(enabled=True),
        filter=FilterConfig(level_source='none'),
        memory=MemoryConfig(enabled=False),
    )
    camera_filter, pipeline = DepthFilter(config, ROBOT), SectorPipeline(config, ROBOT)
    mount = l1w_mount()
    box = (2.0, 2.3, 0.40, 0.55, 0.15)  # odom; the robot drives along y = 0
    lidar = walls(radius=6.0)
    for k in range(60):
        t, x = k / 10.0, 0.035 * k
        seen = (box[0] - x, box[1] - x, *box[2:])
        cloud = camera_filter.process(render(mount, boxes=[seen], seed=k), mount, None, t)
        result = pipeline.process(frame(lidar, odom=Pose2D(x, 0.0, 0.0), stamp=t), cloud)
        ahead, beside = seen[0], max(0.0, -seen[1])
        nearest = math.hypot(max(ahead, beside, 0.0) if ahead > 0.0 else beside, box[2])
        if k >= 3:  # confirmed from the second cloud on, at the latest
            assert result.safety.min() == pytest.approx(nearest, abs=0.05), f't {t:.1f} s'
    assert len(cloud.obstacles.xy) == 0  # the camera does not see it any more at the end
