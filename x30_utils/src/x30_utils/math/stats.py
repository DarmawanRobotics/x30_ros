from __future__ import annotations

import numpy as np

# Chi-square quantiles for gating innovations (degrees of freedom -> threshold).
CHI2 = {
    0.95: {1: 3.841, 2: 5.991, 3: 7.815},
    0.99: {1: 6.635, 2: 9.210, 3: 11.345},
    0.999: {1: 10.828, 2: 13.816, 3: 16.266},
}


def chi2_threshold(dof: int, confidence: float = 0.99) -> float:
    """Return the chi-square gate for an innovation with dof degrees of freedom."""
    try:
        return CHI2[confidence][dof]
    except KeyError as error:
        raise ValueError(f'no chi-square value for dof={dof}, p={confidence}') from error


def mahalanobis2(innovation, covariance) -> float:
    """Return the squared Mahalanobis distance of an innovation."""
    nu = np.asarray(innovation, dtype=float).reshape(-1)
    return float(nu @ np.linalg.solve(np.asarray(covariance, dtype=float), nu))


def kth_smallest(values: np.ndarray, k: int) -> float:
    """Return the k-th smallest value (1-based), or inf when there are fewer than k."""
    values = np.asarray(values, dtype=float)
    if values.size < k:
        return float('inf')
    return float(np.partition(values, k - 1)[k - 1])
