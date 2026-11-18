"""A charging dock in the simulator: a tag camera and the posture and charge services."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from x30_sim.core.config import CameraConfig
from x30_sim.core.world import World, cast_rays
from x30_utils.math import (
    Pose2D,
    compose,
    inverse,
    make_transform,
    planar_transform,
    quat_from_rpy,
)

OCCLUSION_SLACK = 0.05  # m, a wall hit this much before the tag still shows the tag (its wall)
STILL = 0.05  # m/s and rad/s: slower is standing still (sit_down is refused while walking)


OPTICAL = (-math.pi / 2.0, 0.0, -math.pi / 2.0)  # rpy of an optical frame: z ahead, x right


class TagCamera:
    """Which tags a forward (or rear) camera on the robot sees, as base_link -> tag."""

    def __init__(self, config: CameraConfig, world: World, rng):
        self.config, self.world, self.rng = config, world, rng
        self.tags = {name: world.tags[name] for name in config.tags if name in world.tags}
        mx, my, _ = config.mount_xyz
        self.mount = Pose2D(mx, my, math.radians(config.yaw_deg))
        roll, pitch, yaw = OPTICAL
        q = quat_from_rpy(roll, pitch, yaw + math.radians(config.yaw_deg))
        self.base_camera = make_transform(config.mount_xyz, q)  # base_link -> optical frame

    def detections(self, pose: Pose2D) -> dict[str, np.ndarray]:
        """Return {tag frame: 4x4 base_link -> tag} for the tags in view from pose (map)."""
        out = {}
        base = pose.to_matrix()
        camera = pose @ self.mount
        for name, (x, y, z, qx, qy, qz, qw) in self.tags.items():
            tag = make_transform((x, y, z), (qx, qy, qz, qw))
            if self._sees(camera, tag):
                out[name] = self._noisy(compose(inverse(base), tag))
        return out

    def _sees(self, camera: Pose2D, tag: np.ndarray) -> bool:
        c = self.config
        dx, dy = float(tag[0, 3]) - camera.x, float(tag[1, 3]) - camera.y
        distance = math.hypot(dx, dy)
        if not c.range_min <= distance <= c.range_max:
            return False
        bearing = math.atan2(dy, dx)
        if abs(math.remainder(bearing - camera.yaw, 2.0 * math.pi)) > math.radians(c.fov_deg) / 2:
            return False
        normal = tag[:2, 2]  # toward the viewer for a plate seen from the front
        cos_incidence = -(normal[0] * dx + normal[1] * dy) / max(distance, 1e-9)
        if cos_incidence < math.cos(math.radians(c.max_incidence_deg)):
            return False
        hit = cast_rays(self.world, camera.x, camera.y, np.array([bearing]), distance + 1.0)[0]
        return hit >= distance - OCCLUSION_SLACK

    def _noisy(self, m: np.ndarray) -> np.ndarray:
        sigma = self.config.noise_m
        if sigma <= 0.0:
            return m
        dx, dy = self.rng.normal(0.0, sigma, 2)
        dyaw = self.rng.normal(0.0, math.radians(self.config.noise_deg))
        return compose(planar_transform(dx, dy, dyaw), m)


@dataclass(frozen=True)
class DockSpec:
    """Where the robot charges: dock.distance in front of the dock tag (world file)."""

    tag: str = ''
    distance: float = 0.75
    tolerance: float = 0.08
    yaw_tolerance_deg: float = 8.0

    @staticmethod
    def from_dict(data: dict | None) -> DockSpec:
        """Build from the world's dock: {tag, distance, tolerance, yaw_tolerance_deg}."""
        data = data or {}
        return DockSpec(
            str(data.get('tag', '')),
            float(data.get('distance', 0.75)),
            float(data.get('tolerance', 0.08)),
            float(data.get('yaw_tolerance_deg', 8.0)),
        )


class DockStation:
    """Sit, stand and charge, as x30_driver's sit_down, stand_up and charge_* would."""

    def __init__(self, spec: DockSpec, world: World):
        self.spec = spec
        self.contact = None  # docked base_link pose in the map frame
        if spec.tag in world.tags:
            x, y, _, qx, qy, qz, qw = world.tags[spec.tag]
            normal = make_transform((x, y, 0.0), (qx, qy, qz, qw))[:2, 2]
            facing = math.atan2(normal[1], normal[0])
            self.contact = Pose2D(
                x + spec.distance * math.cos(facing),
                y + spec.distance * math.sin(facing),
                math.remainder(facing + math.pi, 2.0 * math.pi),
            )
        self.sitting, self.charging = False, False

    def on_contact(self, pose: Pose2D) -> tuple[bool, str]:
        """Return whether pose is on the charging contacts, and how far off it is."""
        if self.contact is None:
            return False, 'no dock in this world'
        off = self.contact.distance(pose)
        yaw = abs(math.degrees(math.remainder(pose.yaw - self.contact.yaw, 2.0 * math.pi)))
        ok = off <= self.spec.tolerance and yaw <= self.spec.yaw_tolerance_deg
        return ok, f'{off:.3f} m and {yaw:.1f} deg from the contacts'

    def sit(self, speed: float = 0.0) -> tuple[bool, str]:
        """Sit down (the body stops taking commands); refused while walking."""
        if speed > STILL:
            return False, f'walking at {speed:.2f}: stop first'
        self.sitting = True
        return True, 'sitting'

    def stand(self) -> tuple[bool, str]:
        """Stand up; refused while charging."""
        if self.charging:
            return False, 'charging: stop the charge first'
        self.sitting = False
        return True, 'standing'

    def start_charge(self, pose: Pose2D) -> tuple[bool, str]:
        """Start charging when sitting on the contacts."""
        if not self.sitting:
            return False, 'not sitting'
        ok, detail = self.on_contact(pose)
        self.charging = ok
        return ok, f'charging ({detail})' if ok else f'not on the dock: {detail}'

    def stop_charge(self) -> tuple[bool, str]:
        """Stop charging."""
        self.charging = False
        return True, 'charge stopped'
