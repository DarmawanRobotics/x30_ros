"""One inspection point, target by target: point, settle, measure, judge, keep the evidence."""

# The devices are passed in (the ROS adapters in utils/, fakes in the tests), so the sequence,
# its deadline and its failure handling are tested without ROS.

from __future__ import annotations

from dataclasses import dataclass, field, replace
import time

from x30_inspection.core.config import InspectionConfig
from x30_inspection.core.plan import Point, Target
from x30_inspection.core.thermal import (
    FRAME,
    Finding,
    Reading,
    analyse,
    median_reading,
    regions_of,
)


class Camera:
    """What the runner needs from the PTZ camera (utils.camera.HikmicroCamera on the robot)."""

    def connected(self) -> bool:
        """Return True while the camera reports."""
        raise NotImplementedError

    def point(self, pan_deg: float, tilt_deg: float, zoom: float, cancelled) -> str:
        """Move there and wait until the camera has arrived; return '' or the error."""
        raise NotImplementedError

    def measure(self, box) -> tuple[Reading | None, str]:
        """Return (temperatures of a box in fractions of the frame, None: the frame; error)."""
        raise NotImplementedError

    def snapshot(self, prefix: str) -> list[str]:
        """Save the thermal and the visible image as <prefix>_thermal.jpg, ...; return files."""
        raise NotImplementedError

    def park(self) -> str:
        """Go to the park position after a point; return '' or the error."""
        raise NotImplementedError


@dataclass
class TargetResult:
    """What one target gave."""

    target: str
    findings: list[Finding] = field(default_factory=list)
    files: list[str] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    measured: bool = False


@dataclass
class PointResult:
    """What one inspection point gave."""

    point: str
    targets: list[TargetResult] = field(default_factory=list)
    duration_sec: float = 0.0

    @property
    def findings(self) -> list[Finding]:
        """Return every finding of every target."""
        return [f for t in self.targets for f in t.findings]

    @property
    def state(self) -> str:
        """Return done, partial, failed or no_targets."""
        if not self.targets:
            return 'no_targets'
        measured = sum(t.measured for t in self.targets)
        if measured == len(self.targets) and not any(t.errors for t in self.targets):
            return 'done'
        return 'partial' if measured else 'failed'

    @property
    def message(self) -> str:
        """Return a one-line summary."""
        errors = [f'{t.target}: {e}' for t in self.targets for e in t.errors]
        done = sum(t.measured for t in self.targets)
        text = f'{self.point}: {done}/{len(self.targets)} targets measured'
        return f'{text}; {"; ".join(errors)}' if errors else text


class Runner:
    """Inspects the targets of a point within the configured deadline."""

    def __init__(
        self, config: InspectionConfig, camera: Camera, clock=time.monotonic, sleep=time.sleep
    ):
        self.config = config
        self.camera = camera
        self.clock = clock
        self.sleep = sleep
        self.checks = []  # more checks per target: f(target, prefix, result) (acoustic)

    def inspect(self, point: Point, prefix_of, cancelled=lambda: False) -> PointResult:
        """Inspect every target of a point; prefix_of(target) names its evidence files."""
        start = self.clock()
        result = PointResult(point.name)
        deadline = start + self.config.max_duration_sec
        for target in point.targets:
            outcome = TargetResult(target.name)
            result.targets.append(outcome)
            if cancelled():
                outcome.errors.append('cancelled')
            elif self.clock() > deadline:
                outcome.errors.append(f'skipped: over {self.config.max_duration_sec:.0f} s')
            else:
                self.inspect_target(target, prefix_of(target), outcome, cancelled)
        if self.config.camera.park_preset and not cancelled() and self.camera.connected():
            self.camera.park()
        result.duration_sec = self.clock() - start
        return result

    def inspect_target(self, target: Target, prefix: str, outcome: TargetResult, cancelled):
        """Point, settle, measure and judge one target."""
        if not (target.thermal.enabled or target.acoustic.enabled):
            outcome.measured = True  # nothing to check here
            return
        if target.thermal.enabled:
            error = self.camera.point(target.pan_deg, target.tilt_deg, target.zoom, cancelled)
            if error:
                outcome.errors.append(f'camera: {error}')
            else:
                self.sleep(self.config.camera.settle_sec)
                self.thermal(target, prefix, outcome, cancelled)
        for check in self.checks:
            check(target, prefix, outcome)

    def thermal(self, target: Target, prefix: str, outcome: TargetResult, cancelled):
        """Measure every region of a target (median of samples) and judge them."""
        cfg = self.config.thermal
        boxes = regions_of(target) or (None,)
        readings = []
        for region in boxes:
            if cancelled():
                outcome.errors.append('thermal: cancelled')
                return
            samples = []
            for i in range(cfg.samples):
                if i:
                    self.sleep(cfg.sample_interval_sec)
                reading, error = self.camera.measure(None if region is None else region.box)
                if reading is None:
                    outcome.errors.append(f'thermal: {error}')
                    return
                name = FRAME if region is None else region.name
                samples.append(replace(reading, region=name))
            readings.append(median_reading(samples))
        outcome.findings += analyse(target, readings, cfg)
        outcome.files += self.camera.snapshot(prefix)
        outcome.measured = True
