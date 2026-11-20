"""The guard covers the control mux's field: the mux never cuts what the follower publishes."""

# The follower drives with the field of x30_control (beside this package in the workspace)
# between it and the robot, as on the X30. A cut is a command the guard let through and the field
# limited or stopped: the robot jerks, and where the guard plans what the field refuses it stands.

from __future__ import annotations

import numpy as np
import pytest
from rig import Rig, person
from sim import line, route_of, step, straight_route
from test_detour import ROUTE, pocket

from x30_navigation.core.config.follower import FollowerConfig
from x30_navigation.core.controllers import CONTROLLERS
from x30_utils.math import Pose2D
from x30_utils.robot import Footprint, Limits, RobotProfile

field = pytest.importorskip('x30_control.core.field', reason='x30_control is not beside it')
from x30_control.core.command import Command as MuxCommand  # noqa: E402
from x30_control.core.config import FieldConfig  # noqa: E402
from x30_control.core.scan import ScanState, ScanView  # noqa: E402

DONE = ('GOAL_REACHED', 'FAILED')


def drive_through_the_mux(rig: Rig, seconds: float) -> list:
    """Run the follower through the field to the robot; return the field's cuts (t, state)."""
    mux = field.ProtectiveField(rig.robot, FieldConfig())
    cuts = []
    for _ in range(int(seconds / rig.dt)):
        decision = rig.follower.tick(rig.inputs())
        rig.decisions.append(decision)
        if decision.command is not None:
            c = decision.command
            view = ScanView(ScanState.OK, rig.scan().guard_points)
            out = mux.limit(MuxCommand(c.vx, c.vy, c.wz), view, remote=False)
            if out.state is not field.FieldState.CLEAR:
                cuts.append((round(rig.t, 2), out.state.value))
            rig.pose = step(rig.pose, out.command.vx, out.command.vy, out.command.wz, rig.dt)
        rig.t += rig.dt
        if decision.report.state.name in DONE:
            break
    return cuts


def test_the_pocket_detour_passes_without_a_cut():
    """Field (L1W): the guard planned turns the field refused, and the robot stood in the gap."""
    rig = Rig(ROUTE, pose=Pose2D(4.0, 0.0, 0.0))
    rig.obstacles = pocket()
    assert rig.follower.start(rig.pose)[0]
    assert drive_through_the_mux(rig, 30.0) == []
    assert rig.pose.x > 9.0 and abs(rig.pose.y) < 0.1


def test_a_person_on_the_route_is_passed_without_a_cut():
    rig = Rig(route_of(straight_route(0.0, 9.0)))
    rig.obstacles = person(4.0, 0.0)
    assert rig.follower.start(rig.pose)[0]
    assert drive_through_the_mux(rig, 40.0) == []
    assert rig.state.name == 'GOAL_REACHED'


def test_neupan_passes_a_box_in_a_corridor_without_a_cut():
    """NeuPAN plans close to the points: its ramps once outran the guard and the field cut."""
    from test_neupan import MODEL, X30, need_neupan

    need_neupan()
    from x30_navigation.core.controllers.neupan_mpc import NeupanConfig

    robot = RobotProfile(
        name='x30',
        footprint=Footprint(front=0.5, rear=0.5, half_width=0.235),
        limits=Limits(max_vx=0.75, max_vy=0.15, max_wz=0.45, max_accel=0.5, max_yaw_accel=1.5),
    )
    config = FollowerConfig(require_localization=False, controller='neupan')
    rig = Rig(route_of(straight_route(-3.0, 6.0)), config=config, robot=robot)
    controller = CONTROLLERS.create('neupan', NeupanConfig(dune_checkpoint=MODEL), X30)
    rig.follower.set_controller('neupan', controller)
    rig.sectors = 72
    corridor = np.vstack([line(-4.0, 7.0, -1.2), line(-4.0, 7.0, 1.2)])
    rig.obstacles = np.vstack([person(1.5, 0.15, r=0.3, n=24), corridor])
    assert rig.follower.start(rig.pose)[0]
    assert drive_through_the_mux(rig, 40.0) == []
    assert rig.state.name == 'GOAL_REACHED'
