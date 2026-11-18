"""Asymmetric smoothing of avoidance outputs: slow to relax, instant to brake."""

from __future__ import annotations

from dataclasses import replace


class AsymmetricSmoother:
    """Exponential smoothing that never delays a stop."""

    def __init__(self, alpha: float):
        self.alpha = alpha
        self.value = None

    def reset(self):
        """Forget the filtered value."""
        self.value = None

    def step(self, raw):
        """Return the filtered AvoidanceOut."""
        if self.value is None or self.alpha <= 0.0:
            self.value = raw
            return raw
        a, old = self.alpha, self.value

        def blend(previous: float, new: float) -> float:
            return a * previous + (1.0 - a) * new

        scale = raw.velocity_scale
        if scale > old.velocity_scale:
            scale = blend(old.velocity_scale, scale)
        self.value = replace(
            raw,
            heading_offset=blend(old.heading_offset, raw.heading_offset),
            yaw_rate_bias=blend(old.yaw_rate_bias, raw.yaw_rate_bias),
            lateral_bias=blend(old.lateral_bias, raw.lateral_bias),
            velocity_scale=scale,
        )
        return self.value
