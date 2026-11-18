from __future__ import annotations

from collections import deque
import math


class Ema:
    """Exponential moving average; alpha is the weight of a new sample (0..1]."""

    def __init__(self, alpha: float):
        if not 0.0 < alpha <= 1.0:
            raise ValueError(f'alpha must be in (0, 1], got {alpha}')
        self.alpha = alpha
        self.value: float | None = None

    def update(self, sample: float) -> float:
        """Add a sample and return the average."""
        if self.value is None:
            self.value = sample
        else:
            self.value += self.alpha * (sample - self.value)
        return self.value

    def reset(self):
        """Forget the average."""
        self.value = None


class SlewLimiter:
    """Limit how fast a value changes: rise and fall in units per second."""

    def __init__(self, rise: float, fall: float | None = None, value: float = 0.0):
        fall = rise if fall is None else fall
        if not (rise >= 0.0 and fall >= 0.0):
            raise ValueError('slew rates must be >= 0')
        self.rise, self.fall, self.value = rise, fall, value

    def step(self, target: float, dt: float) -> float:
        """Move toward target by at most the allowed change over dt."""
        if not math.isfinite(target):
            target = 0.0
        dt = max(0.0, dt)
        growing = abs(target) > abs(self.value) and target * self.value >= 0.0
        limit = (self.rise if growing else self.fall) * dt
        self.value += max(-limit, min(limit, target - self.value))
        return self.value

    def reset(self, value: float = 0.0):
        """Set the value without limiting."""
        self.value = value


class RateMeter:
    """Message rate over a sliding time window."""

    def __init__(self, window_sec: float = 5.0):
        self.window_sec = window_sec
        self.times: deque[float] = deque()

    def tick(self, t: float):
        """Record an arrival at time t (s)."""
        self.times.append(t)
        self._prune(t)

    def _prune(self, now: float):
        while self.times and now - self.times[0] > self.window_sec:
            self.times.popleft()

    def rate(self, now: float) -> float:
        """Return the rate in Hz over the window ending now; 0 when silent for three periods."""
        self._prune(now)
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        hz = (len(self.times) - 1) / span if span > 0.0 else 0.0
        if hz <= 0.0 or now - self.times[-1] > max(1.0, 3.0 / hz):
            return 0.0
        return hz

    def age(self, now: float) -> float:
        """Return the time since the last arrival (inf when none)."""
        return now - self.times[-1] if self.times else math.inf


class Cusum:
    """One-sided CUSUM change detector: s = max(0, s + x - drift), alarm when s > threshold."""

    def __init__(self, drift: float, threshold: float, cap: float = math.inf):
        self.drift, self.threshold, self.cap, self.value = drift, threshold, cap, 0.0

    def update(self, sample: float) -> bool:
        """Add a sample; return True while in alarm. The statistic never exceeds cap."""
        self.value = min(self.cap, max(0.0, self.value + sample - self.drift))
        return self.value > self.threshold

    def reset(self):
        """Clear the statistic."""
        self.value = 0.0


class Hold:
    """Debounced boolean: True after on_sec of True input, False after off_sec of False."""

    def __init__(self, on_sec: float, off_sec: float = 0.0):
        self.on_sec, self.off_sec = on_sec, off_sec
        self.state = False
        self._since: float | None = None

    def update(self, value: bool, t: float) -> bool:
        """Feed the raw input at time t and return the debounced state."""
        if value == self.state:
            self._since = None
            return self.state
        if self._since is None:
            self._since = t
        if t - self._since >= (self.on_sec if value else self.off_sec):
            self.state, self._since = value, None
        return self.state
