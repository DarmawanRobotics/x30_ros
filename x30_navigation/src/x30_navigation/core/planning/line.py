"""Line of sight on the planning grid and line-of-sight shortcuts of a cell path."""

from __future__ import annotations

import numpy as np


def line_cells(a: tuple[int, int], b: tuple[int, int]) -> tuple[np.ndarray, np.ndarray]:
    """Return (rows, cols) of every cell a line between the centres of cells a and b touches."""
    (r0, c0), (r1, c1) = a, b
    n = 4 * max(abs(r1 - r0), abs(c1 - c0)) + 1
    t = np.linspace(0.0, 1.0, n + 1)
    rows = np.floor(r0 + 0.5 + t * (r1 - r0)).astype(np.int64)
    cols = np.floor(c0 + 0.5 + t * (c1 - c0)).astype(np.int64)
    corner = (np.diff(rows) != 0) & (np.diff(cols) != 0)
    rows = np.concatenate((rows, rows[:-1][corner], rows[1:][corner]))
    cols = np.concatenate((cols, cols[1:][corner], cols[:-1][corner]))
    return rows, cols


def line_ok(cost: np.ndarray, a, b, max_cost: float) -> bool:
    """Return True when every cell the line a-b touches is passable and costs <= max_cost."""
    rows, cols = line_cells(a, b)
    height, width = cost.shape
    if rows.min() < 0 or cols.min() < 0 or rows.max() >= height or cols.max() >= width:
        return False
    values = cost[rows, cols]
    return bool(np.all(np.isfinite(values)) and np.all(values <= max_cost))


def shortcut(cost: np.ndarray, path: list[tuple[int, int]]) -> list[tuple[int, int]]:
    """Replace runs of the path by straight lines where nothing is in the way."""
    if len(path) < 3:
        return list(path)
    out = [path[0]]
    anchor = 0
    worst = float(cost[path[0]])
    for i in range(1, len(path)):
        worst = max(worst, float(cost[path[i]]))
        if i - anchor > 1 and not line_ok(cost, path[anchor], path[i], worst + 1e-9):
            out.append(path[i - 1])
            anchor = i - 1
            worst = max(float(cost[path[anchor]]), float(cost[path[i]]))
    out.append(path[-1])
    return out
