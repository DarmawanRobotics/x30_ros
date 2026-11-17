"""A synthetic depth camera for the tests: pinhole rays cast onto the floor and boxes."""

from __future__ import annotations

import math

import numpy as np

from x30_utils.math import default_rng, make_transform, quat_from_rpy

WIDTH, HEIGHT = 212, 120  # a D435 at 424 x 240, decimated by 2
HFOV, VFOV = math.radians(87.0), math.radians(58.0)
BASE_HEIGHT = 0.35
OPTICAL = make_transform(quat=quat_from_rpy(-math.pi / 2.0, 0.0, -math.pi / 2.0))


def l1w_mount(pitch_deg: float = 10.0, roll_deg: float = 0.0) -> np.ndarray:
    """Return base_link <- camera_depth_optical_frame of a D435i mounted like on the L1W."""
    screw = make_transform(
        (0.25, 0.0, 0.096), quat_from_rpy(math.radians(roll_deg), math.radians(pitch_deg), 0.0)
    )
    link = make_transform((0.0106, 0.0175, 0.0125))
    return screw @ link @ OPTICAL


def render(
    mount: np.ndarray,
    boxes=(),
    base_height: float = BASE_HEIGHT,
    noise: float = 0.0,
    seed: int = 0,
    floor: bool = True,
) -> np.ndarray:
    """Return the points the camera sees in its optical frame; boxes (x0, x1, y0, y1, top)."""
    fx = (WIDTH / 2.0) / math.tan(HFOV / 2.0)
    fy = (HEIGHT / 2.0) / math.tan(VFOV / 2.0)
    u, v = np.meshgrid(np.arange(WIDTH) + 0.5, np.arange(HEIGHT) + 0.5)
    rays = np.column_stack(
        (((u - WIDTH / 2.0) / fx).ravel(), ((v - HEIGHT / 2.0) / fy).ravel(), np.ones(u.size))
    )
    rotation, origin = mount[:3, :3], mount[:3, 3]
    dirs = rays @ rotation.T  # base_link, per unit of depth
    depth = np.full(len(rays), np.inf)
    if floor:
        down = dirs[:, 2] < -1e-9
        depth[down] = (-base_height - origin[2]) / dirs[down, 2]
    for x0, x1, y0, y1, top in boxes:
        low = np.array([x0, y0, -base_height])
        high = np.array([x1, y1, top - base_height])
        with np.errstate(divide='ignore', invalid='ignore'):
            t0 = (low - origin) / dirs
            t1 = (high - origin) / dirs
        near = np.nanmax(np.minimum(t0, t1), axis=1)
        far = np.nanmin(np.maximum(t0, t1), axis=1)
        hit = (near <= far) & (near > 0.0)
        depth = np.where(hit & (near < depth), near, depth)
    seen = np.isfinite(depth) & (depth > 0.0)
    z = depth[seen]
    if noise > 0.0:
        z = z + default_rng(seed).normal(0.0, 1.0, len(z)) * noise * z**2
    return rays[seen] * z[:, None]
