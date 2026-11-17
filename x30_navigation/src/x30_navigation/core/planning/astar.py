"""Grid search on a costmap: A* and the nearest free cell."""

from __future__ import annotations

import heapq
import math
import time

import numpy as np

SQRT2 = math.sqrt(2.0)


class SearchTimeoutError(RuntimeError):
    """The search ran out of time or expansions."""


def astar(
    cost: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    heuristic_weight: float = 1.0,
    max_expansions: int = 3_000_000,
    timeout_sec: float = 0.0,
) -> list[tuple[int, int]] | None:
    """Return the cheapest 8-connected path of cells from start to goal, or None."""
    rows, cols = cost.shape
    width = cols + 2
    padded = np.full((rows + 2, width), np.inf)
    padded[1:-1, 1:-1] = cost
    c = padded.ravel().tolist()
    s = (start[0] + 1) * width + start[1] + 1
    g = (goal[0] + 1) * width + goal[1] + 1
    if not (math.isfinite(c[s]) and math.isfinite(c[g])):
        return None
    parent = _search(c, width, s, g, max(1.0, heuristic_weight), max_expansions, timeout_sec)
    if parent is None:
        return None
    path, i = [], g
    while i != -1:
        r, k = divmod(i, width)
        path.append((r - 1, k - 1))
        i = parent[i]
    path.reverse()
    return path


def _search(c, width, s, g, weight, max_expansions, timeout_sec):
    """Run A* on the padded flat grid; return the parent list or None when unreachable."""
    gr, gc = divmod(g, width)
    straight = ((1, 1.0), (-1, 1.0), (width, 1.0), (-width, 1.0))
    diagonal = (
        (width + 1, 1, width),
        (width - 1, -1, width),
        (-width + 1, 1, -width),
        (-width - 1, -1, -width),
    )
    inf = math.inf
    best = [inf] * len(c)
    parent = [-1] * len(c)
    closed = bytearray(len(c))
    best[s] = 0.0

    def h(i):
        r, k = divmod(i, width)
        dr, dc = abs(r - gr), abs(k - gc)
        return weight * (max(dr, dc) + (SQRT2 - 1.0) * min(dr, dc))

    heap = [(h(s), 0.0, s)]
    deadline = time.monotonic() + timeout_sec if timeout_sec > 0.0 else None
    expansions = 0
    while heap:
        _, cost_here, i = heapq.heappop(heap)
        if closed[i]:
            continue
        if i == g:
            return parent
        closed[i] = 1
        expansions += 1
        if expansions > max_expansions or (
            deadline is not None and expansions % 4096 == 0 and time.monotonic() > deadline
        ):
            raise SearchTimeoutError(f'search stopped after {expansions} expansions')
        moves = [(i + step, length) for step, length in straight]
        moves += [
            (i + step, SQRT2) for step, dc, dr in diagonal if c[i + dc] != inf and c[i + dr] != inf
        ]
        for j, length in moves:
            cj = c[j]
            if cj == inf or closed[j]:
                continue
            ng = cost_here + length * cj
            if ng < best[j]:
                best[j] = ng
                parent[j] = i
                heapq.heappush(heap, (ng + h(j), ng, j))
    return None


def disk_offsets(radius: int) -> list[tuple[int, int, float]]:
    """Return (drow, dcol, distance) within a radius in cells, nearest first."""
    r = max(int(radius), 0)
    offsets = [
        (dr, dc, math.hypot(dr, dc))
        for dr in range(-r, r + 1)
        for dc in range(-r, r + 1)
        if math.hypot(dr, dc) <= r + 1e-9
    ]
    return sorted(offsets, key=lambda o: o[2])


def nearest_free(cost: np.ndarray, cell: tuple[int, int], max_cells: int, accept=None):
    """Return the closest passable cell within max_cells (accepted by accept(cell)), or None."""
    rows, cols = cost.shape
    r0, c0 = cell
    for dr, dc, _ in disk_offsets(max_cells):
        r, c = r0 + dr, c0 + dc
        inside = 0 <= r < rows and 0 <= c < cols
        if inside and math.isfinite(cost[r, c]) and (accept is None or accept((r, c))):
            return r, c
    return None
