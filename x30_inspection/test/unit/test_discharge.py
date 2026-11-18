"""Partial discharge: severities of the driver's results, the check around one detection."""

from __future__ import annotations

import dataclasses

import pytest

from x30_inspection.core.config import AcousticConfig
from x30_inspection.core.discharge import (
    LEAKAGE,
    Acoustic,
    AcousticCheck,
    Detection,
    finding,
    judge,
    types_of,
)
from x30_inspection.core.plan import AcousticTarget, Target, ThermalTarget
from x30_inspection.core.runner import TargetResult
from x30_inspection.core.thermal import Severity

CONFIG = AcousticConfig()
NOISE, CORONA, SURFACE, FLOATING = 1, 2, 4, 8
PD_ONLY = Target('pd', 0, 0, thermal=ThermalTarget(enabled=False),
                 acoustic=AcousticTarget(enabled=True, duration_sec=3.0))  # fmt: skip
BOTH = Target('bushing', 10, 0, acoustic=AcousticTarget(enabled=True))


class FakeAcoustic(Acoustic):
    """Returns one detection (or an error), records the calls."""

    def __init__(self, detection=None, error='', files=(), mode=1):
        self.detection, self.error, self.files, self.work_mode = detection, error, files, mode
        self.calls = []

    def connected(self):
        return True

    def mode(self):
        return self.work_mode

    def detect(self, duration, prefix, cancelled):
        self.calls.append((duration, prefix))
        return self.detection, list(self.files), self.error


def test_types_of_the_status_bits():
    assert types_of(NOISE) == [] and types_of(CORONA | FLOATING) == ['corona', 'floating']


@pytest.mark.parametrize(
    ('status', 'severity', 'label'),
    [
        (NOISE, Severity.NORMAL, 'noise'),
        (CORONA, Severity.ATTENTION, 'corona'),
        (SURFACE, Severity.WARNING, 'surface'),
        (FLOATING, Severity.CRITICAL, 'floating'),
        (CORONA | SURFACE, Severity.WARNING, 'corona+surface'),
    ],
)
def test_each_discharge_type_has_its_severity(status, severity, label):
    assert judge(Detection(status=status, intensity=10.0), CONFIG) == (severity, label)


def test_intensity_raises_a_discharge_and_severities_are_configurable():
    config = dataclasses.replace(CONFIG, intensity_warning=30.0, intensity_critical=60.0)
    assert judge(Detection(status=CORONA, intensity=35.0), config)[0] is Severity.WARNING
    assert judge(Detection(status=CORONA, intensity=65.0), config)[0] is Severity.CRITICAL
    assert judge(Detection(status=NOISE, intensity=99.0), config)[0] is Severity.NORMAL
    calm = dataclasses.replace(CONFIG, corona='normal')
    assert judge(Detection(status=CORONA), calm)[0] is Severity.NORMAL


def test_finding_carries_the_intensity_and_the_probabilities():
    detection = Detection(status=SURFACE, intensity=42.0, probabilities={'surface': 0.81})
    f = finding(PD_ONLY, detection, CONFIG)
    assert (f.sensor, f.criterion, f.label, f.value, f.unit) == (
        'acoustic',
        'discharge',
        'surface',
        42.0,
        'dB',
    )
    assert 'surface 0.81' in f.message and 'noise 0.00' in f.message


def test_check_detects_judges_and_keeps_the_photos():
    acoustic = FakeAcoustic(Detection(status=FLOATING, intensity=50.0), files=['/r/pd_prpd.jpg'])
    outcome = TargetResult('pd')
    AcousticCheck(acoustic, CONFIG)(PD_ONLY, '/r/pd', outcome)
    assert acoustic.calls == [(3.0, '/r/pd')]  # the target's own duration
    assert outcome.measured and outcome.files == ['/r/pd_prpd.jpg']
    assert [f.severity for f in outcome.findings] == [Severity.CRITICAL]


def test_a_detection_measures_a_target_whose_thermal_part_failed():
    acoustic = FakeAcoustic(Detection(status=NOISE))
    outcome = TargetResult('bushing', errors=['camera: stalled'])
    AcousticCheck(acoustic, CONFIG)(BOTH, '/r/bushing', outcome)
    assert acoustic.calls == [(CONFIG.duration_sec, '/r/bushing')]
    assert outcome.measured and len(outcome.findings) == 1  # the point: partial, not failed


def test_check_reports_instead_of_raising():
    outcome = TargetResult('pd')
    AcousticCheck(FakeAcoustic(error='detect timeout: no result', files=['/r/pd_soundmap.jpg']),
                  CONFIG)(PD_ONLY, '/r/pd', outcome)  # fmt: skip
    assert outcome.errors == ['acoustic: detect timeout: no result'] and not outcome.measured
    assert outcome.files == ['/r/pd_soundmap.jpg']  # whatever came back is kept


def test_gas_leak_mode_is_refused_before_and_after():
    leak = FakeAcoustic(Detection(status=CORONA), mode=LEAKAGE)
    outcome = TargetResult('pd')
    AcousticCheck(leak, CONFIG)(PD_ONLY, '/r/pd', outcome)
    assert not leak.calls and 'gas leak mode' in outcome.errors[0]
    switched = FakeAcoustic(Detection(mode=LEAKAGE, status=2), mode=0)  # mode unknown before
    outcome = TargetResult('pd')
    AcousticCheck(switched, CONFIG)(PD_ONLY, '/r/pd', outcome)
    assert 'gas leak detection' in outcome.errors[0] and not outcome.findings


def test_targets_without_acoustic_or_with_it_off():
    acoustic = FakeAcoustic(Detection())
    outcome = TargetResult('t')
    AcousticCheck(acoustic, CONFIG)(Target('t', 0, 0), '/r/t', outcome)
    assert not acoustic.calls and outcome == TargetResult('t')
    off = dataclasses.replace(CONFIG, enabled=False)
    AcousticCheck(acoustic, off)(PD_ONLY, '/r/pd', outcome)
    assert not acoustic.calls and outcome.errors == ['acoustic: off']
