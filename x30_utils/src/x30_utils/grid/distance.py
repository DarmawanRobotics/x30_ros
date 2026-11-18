from __future__ import annotations

import numpy as np


def distance_field(occupied: np.ndarray, resolution: float, max_distance: float) -> np.ndarray:
    """Return the distance (m) from every cell to the nearest occupied cell, capped."""
    occupied = np.asarray(occupied, dtype=bool)
    if not occupied.any():
        return np.full(occupied.shape, max_distance, dtype=np.float32)
    from scipy.ndimage import distance_transform_edt

    field = distance_transform_edt(~occupied) * resolution
    return np.minimum(field, max_distance).astype(np.float32)


def likelihood_field(distance: np.ndarray, sigma: float) -> np.ndarray:
    """Return exp(-d^2 / 2 sigma^2): 1 on an obstacle, falling off with distance."""
    return np.exp(-(distance.astype(np.float64) ** 2) / (2.0 * sigma * sigma)).astype(np.float32)
