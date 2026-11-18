"""Partial discharge at a target: one Hertzino detection measured after the robot stood still."""

# hertzino_driver's detect action restarts a detection already running, so its result is always
# measured after the goal, and hands the sound map and PRPD photos over with it. Corona, surface
# and floating discharge each have a severity (config); the intensity can raise it.

from __future__ import annotations

from dataclasses import dataclass, field

from x30_inspection.core.config import SEVERITIES, AcousticConfig
from x30_inspection.core.plan import Target
from x30_inspection.core.thermal import Finding, Severity

PARTIAL_DISCHARGE, LEAKAGE = 1, 2  # hertzino_msgs DetectionConfig.work_mode, detect_mode
DISCHARGES = ((2, 'corona'), (4, 'surface'), (8, 'floating'))  # DetectionResult.status bits
CLASSES = ('noise', 'corona', 'surface', 'floating')


@dataclass(frozen=True)
class Detection:
    """What is judged of one hertzino_msgs/DetectionResult."""

    mode: int = PARTIAL_DISCHARGE
    status: int = 1  # bits: 1 noise, 2 corona, 4 surface, 8 floating
    intensity: float = 0.0  # sound map intensity
    probabilities: dict = field(default_factory=dict)  # class -> probability


def types_of(status: int) -> list[str]:
    """Return the discharge types of a status (empty: noise only)."""
    return [name for bit, name in DISCHARGES if status & bit]


def judge(detection: Detection, config: AcousticConfig) -> tuple[Severity, str]:
    """Return the severity of a partial discharge detection and its label."""
    types = types_of(detection.status)
    if not types:
        return Severity.NORMAL, 'noise'
    severity = max(Severity(SEVERITIES.index(getattr(config, t))) for t in types)
    if config.intensity_critical > 0 and detection.intensity >= config.intensity_critical:
        severity = Severity.CRITICAL
    elif config.intensity_warning > 0 and detection.intensity >= config.intensity_warning:
        severity = max(severity, Severity.WARNING)
    return severity, '+'.join(types)


def finding(target: Target, detection: Detection, config: AcousticConfig) -> Finding:
    """Return the Finding of a detection."""
    severity, label = judge(detection, config)
    probs = ', '.join(
        f'{name} {float(detection.probabilities.get(name, 0.0)):.2f}' for name in CLASSES
    )
    text = f'{label}, intensity {detection.intensity:.0f} ({probs})'
    return Finding(target.name, 'acoustic', 'discharge', label, severity, detection.intensity,
                   unit='dB', message=text)  # fmt: skip


class Acoustic:
    """What the check needs from the acoustic camera (utils.acoustic on the robot)."""

    def connected(self) -> bool:
        """Return True while the driver reports a connected camera."""
        raise NotImplementedError

    def mode(self) -> int:
        """Return the camera's work mode: PARTIAL_DISCHARGE, LEAKAGE or 0 (unknown)."""
        raise NotImplementedError

    def detect(self, duration: float, prefix: str, cancelled) -> tuple:
        """Detect once, save the photos as <prefix>_soundmap.jpg ...: (Detection, files, error)."""
        raise NotImplementedError


class AcousticCheck:
    """The runner's per-target check: one detection for every acoustic target."""

    def __init__(self, acoustic: Acoustic, config: AcousticConfig, cancelled=lambda: False):
        self.acoustic, self.config, self.cancelled = acoustic, config, cancelled

    def __call__(self, target: Target, prefix: str, outcome):
        """Detect, judge and keep the photos; errors go to the outcome, never raise."""
        if not target.acoustic.enabled:
            return
        if not self.config.enabled:
            outcome.errors.append('acoustic: off')
            return
        if self.acoustic.mode() == LEAKAGE:
            outcome.errors.append(
                'acoustic: the camera is in gas leak mode, not partial discharge'
            )
            return
        duration = target.acoustic.duration_sec or self.config.duration_sec
        detection, files, error = self.acoustic.detect(duration, prefix, self.cancelled)
        outcome.files += files
        if detection is None:
            outcome.errors.append(f'acoustic: {error}')
            return
        if detection.mode == LEAKAGE:
            outcome.errors.append('acoustic: a gas leak detection, not partial discharge')
            return
        outcome.findings.append(finding(target, detection, self.config))
        outcome.measured = True  # a failed thermal part leaves the target partial, not failed
