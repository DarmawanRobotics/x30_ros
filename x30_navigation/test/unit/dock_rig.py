"""A kinematic robot in front of a dock tag, seen by a forward (or rear) camera, for Docking."""

from __future__ import annotations

from dataclasses import replace
import math

import numpy as np

from x30_navigation.core.config.docking import DockingConfig
from x30_navigation.core.docking.control import DockLimits
from x30_navigation.core.docking.fsm import Docking
from x30_navigation.core.docking.geometry import tag_pose
from x30_navigation.core.docking.state import D, DockInputs, PoseInput
from x30_utils.math import Pose2D, default_rng, planar_transform

DT = 0.05
TAG = Pose2D(3.0, 0.0, math.pi)  # on a wall at x = 3, its normal toward -x (the robot)


def tag_matrix(tag: Pose2D, z_toward_viewer: bool = True) -> np.ndarray:
    """Return a tag frame on a wall: z along the plate normal (toward the viewer or away)."""
    m = planar_transform(tag.x, tag.y, tag.yaw)  # x along the normal
    n, side = m[:3, 0].copy(), m[:3, 1].copy()
    z = n if z_toward_viewer else -n
    out = np.eye(4)
    out[:3, 2], out[:3, 0], out[:3, 1] = z, side, np.cross(z, side)
    out[:3, 3] = (tag.x, tag.y, 0.5)
    return out


class DockRig:
    """Robot + camera + tag + battery; run() ticks Docking and integrates its commands."""

    def __init__(self, start: Pose2D, config: DockingConfig | None = None, can_strafe=True,
                 rear_camera=False, seed=0):  # fmt: skip
        self.config = config or DockingConfig()
        limits = DockLimits(0.8, 0.4 if can_strafe else 0.0, 1.0)
        self.docking = Docking(self.config, can_strafe, limits)
        self.pose, self.now, self.can_strafe = start, 0.0, can_strafe
        self.rear_camera, self.rng = rear_camera, default_rng(seed)
        self.camera_fov = math.radians(70.0)
        self.camera_min_range = 0.35  # closer the tag fills the image: not detected
        self.noise, self.visible, self.moves = 0.004, True, True
        self.estop, self.remote, self.armed = False, False, True
        self.charging_after = None  # seconds after the charge hook the battery charges
        self.hook_answers = {}  # group -> (ok, message); missing: ok
        self.charge_started = None
        self.decisions, self.commands, self.hooks = [], [], []

    def detection(self):
        """Return the tag in odom as the camera sees it (None: not in view)."""
        if not self.visible:
            return None
        local = self.pose.between(TAG)
        bearing = math.atan2(local.y, local.x)
        if self.rear_camera:
            bearing = math.remainder(bearing - math.pi, 2.0 * math.pi)
        distance = math.hypot(local.x, local.y)
        if abs(bearing) > self.camera_fov / 2.0 or distance < self.camera_min_range:
            return None
        noisy = Pose2D(
            TAG.x + self.rng.normal(0.0, self.noise), TAG.y + self.rng.normal(0.0, self.noise),
            TAG.yaw + self.rng.normal(0.0, 0.01),
        )  # fmt: skip
        pose = tag_pose(tag_matrix(noisy, z_toward_viewer=False), (self.pose.x, self.pose.y))
        return PoseInput(pose, 0.05)

    def charging(self):
        if self.charge_started is None or self.charging_after is None:
            return False
        return self.now - self.charge_started >= self.charging_after

    def tick(self):
        inputs = DockInputs(
            self.now, PoseInput(self.pose, 0.01), self.detection(), self.estop, self.remote,
            self.armed, self.charging(),
        )  # fmt: skip
        decision = self.docking.tick(inputs)
        self.decisions.append(decision)
        if decision.hooks is not None:
            self.hooks.append(decision.hooks.group)
            ok, message = self.hook_answers.get(decision.hooks.group, (True, 'ok'))
            if decision.hooks.group == 'charge' and ok:
                self.charge_started = self.now
            self.docking.on_hooks(decision.hooks.ticket, ok, message)
        command = decision.command
        if command is not None:
            self.commands.append(command)
            if self.moves:
                vy = command.vy if self.can_strafe else 0.0
                self.pose = self.pose @ Pose2D(command.vx * DT, vy * DT, command.wz * DT)
        self.now += DT
        return decision

    def run(self, seconds: float, until=None):
        decision = None
        for _ in range(int(seconds / DT)):
            decision = self.tick()
            if until is not None and until(decision):
                break
        return decision

    def states(self) -> list:
        out = []
        for d in self.decisions:
            if not out or out[-1] is not d.report.state:
                out.append(d.report.state)
        return out


def docked(decision) -> bool:
    return decision.report.state in (D.DOCKED, D.FAILED)


def with_goal(config: DockingConfig, **goal) -> DockingConfig:
    return replace(config, goal=replace(config.goal, **goal))


def goal_pose(config: DockingConfig) -> Pose2D:
    """The docked pose in front of TAG (forward docking)."""
    g = config.goal
    return Pose2D(TAG.x - g.distance, TAG.y + g.lateral_offset, 0.0)
