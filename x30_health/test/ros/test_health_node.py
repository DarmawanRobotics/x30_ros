"""health_monitor_node in-process: HealthStatus, diagnostics, stamp ages, states, nodes, TF."""

from __future__ import annotations

import io
import itertools
import os
import subprocess
import sys
import time

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import rospy
from sensor_msgs.msg import PointCloud2

from x30_health.core.checks.state import Field, LocalizationState, OdometryState
from x30_health.core.stamp import ros1_stamp
from x30_health.health_monitor_node import HealthMonitorNode
from x30_msgs.msg import (
    ControlStatus,
    HealthStatus,
    LocalizationStatus,
    OdometryStatus,
    SectorScan,
)
from x30_utils.math import make_transform
from x30_utils.ros import X30Node
from x30_utils.ros.time import to_stamp
from x30_utils.testing.harness import Collector, connected, delete_params, wait_for

QUIET_SYSTEM = {
    'system.max_temp_c': 0.0,
    'system.max_memory_percent': 0.0,
    'system.min_disk_free_gb': 0.0,
    'system.max_load_per_cpu': 0.0,
}
NAMES = itertools.count()
PEER = 'import rospy; rospy.init_node("peer"); rospy.spin()'


class Rig:
    """The monitor watching /<ns>/..., a helper publishing its inputs, collectors of outputs."""

    def __init__(self, tf: bool = False, **overrides):
        self.ns = f'/health_test_{next(NAMES)}'
        ns = self.ns
        params = {
            'sensors.topics': [f'{ns}/cloud:5.0'],
            'degradation.rules': [f'sensor:{ns}/*=0.5'],
            'period_sec': 0.2,
            'startup_grace_sec': 0.0,
            'sensors.discover_period_sec': 0.2,
            'tf.enabled': tf,
            'state.odometry_topic': f'{ns}/odometry/status',
            'state.localization_topic': f'{ns}/localization/status',
            'state.control_topic': f'{ns}/control/status',
            'state.safety_scan_topic': f'{ns}/perception/safety_scan',
            **QUIET_SYSTEM,
            **overrides,
        }
        self.node = HealthMonitorNode(param_ns=f'{ns}/health', overrides=params)
        self.helper = X30Node('helper', param_ns=f'{ns}/helper')
        self.status = Collector(HealthStatus, 'health/status')
        self.diagnostics = Collector(DiagnosticArray, 'diagnostics')
        h = self.helper
        self.cloud = h.create_publisher(PointCloud2, f'{ns}/cloud', 10)
        self.odometry = h.latched(OdometryStatus, f'{ns}/odometry/status')
        self.localization = h.latched(LocalizationStatus, f'{ns}/localization/status')
        self.control = h.latched(ControlStatus, f'{ns}/control/status')
        self.scan = h.create_publisher(SectorScan, f'{ns}/perception/safety_scan', 10)
        assert connected(self.scan)

    def publish_states(self, odometry=OdometryState.OK, localization=LocalizationState.LOCALIZED):
        self.odometry.publish(OdometryStatus(state=int(odometry), message='test'))
        self.localization.publish(LocalizationStatus(state=int(localization), sigma_xy=0.05))
        self.control.publish(ControlStatus(field=int(Field.CLEAR)))

    def stream(self, seconds: float, age: float, hz: float = 10.0):
        """Publish clouds stamped age seconds in the past, and clear safety scans."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            cloud = PointCloud2()
            cloud.header.stamp = to_stamp(rospy.get_time() - age)
            self.cloud.publish(cloud)
            self.scan.publish(SectorScan(angle_increment=0.1, range_max=10.0, ranges=[5.0] * 8))
            time.sleep(1.0 / hz)

    def diagnostic(self, name: str) -> DiagnosticStatus | None:
        last = self.diagnostics.last
        found = [s for s in (last.status if last else []) if s.name == f'health: {name}']
        return found[0] if found else None

    def close(self):
        self.status.close()
        self.diagnostics.close()
        self.helper.destroy_node()
        self.node.destroy_node()
        delete_params(self.ns)


def test_old_stamps_not_localized_then_ok_then_lost(ros):
    """Typed status, speed scale and one diagnostic per check."""
    rig = Rig()
    try:
        rig.publish_states(localization=LocalizationState.NONE)
        rig.stream(2.0, age=2.0)
        status = rig.status.last
        assert status.level == HealthStatus.WARN, status.issues
        cloud = f'{rig.ns}/cloud'
        assert any(line.startswith(f'{cloud} stamps 2.') for line in status.issues), status.issues
        assert 'not localized' in status.issues
        assert abs(status.speed_scale - 0.5) < 1e-6
        diag = rig.diagnostic(f'sensor:{cloud}')
        assert diag is not None and diag.level == DiagnosticStatus.WARN
        assert {kv.key for kv in diag.values} >= {'hz', 'min_hz', 'age_sec'}

        rig.publish_states()
        rig.stream(1.0, age=0.01)
        status = rig.status.last
        assert status.level == HealthStatus.OK, status.issues
        assert status.speed_scale == 1.0 and list(status.issues) == []
        assert rig.diagnostic('state:localization').level == DiagnosticStatus.OK

        rig.publish_states(odometry=OdometryState.LOST)
        rig.stream(0.6, age=0.01)
        status = rig.status.last
        assert status.level == HealthStatus.ERROR and status.speed_scale == 0.0
        assert status.issues[0] == 'odometry lost: test'
    finally:
        rig.close()


def test_blind_safety_scan_is_an_error(ros):
    rig = Rig()
    try:
        rig.publish_states()
        for _ in range(6):
            rig.scan.publish(SectorScan(ranges=[0.0] * 8))
            time.sleep(0.1)
        scan = lambda: rig.diagnostic('state:safety_scan')  # noqa: E731
        assert wait_for(lambda: scan() is not None and scan().level == DiagnosticStatus.ERROR, 2.0)
        # health/status is a topic of its own: it may come after the diagnostics
        assert rig.status.wait_until(lambda m: m.speed_scale == 0.0, 2.0), rig.status.last
    finally:
        rig.close()


def peer(namespace: str) -> subprocess.Popen:
    return subprocess.Popen(
        [sys.executable, '-c', PEER, f'__ns:={namespace}'],
        env=dict(os.environ),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def test_expected_node_in_another_namespace_is_missing(ros):
    """The same node name in a different namespace does not count."""
    rig = Rig(**{'nodes.expected': ['/health_nodes/peer']})
    procs = [peer('/health_nodes_elsewhere')]
    try:
        time.sleep(1.5)
        assert rig.diagnostic('node:/health_nodes/peer').level == DiagnosticStatus.ERROR
        procs.append(peer('/health_nodes'))
        ok = lambda: rig.diagnostic('node:/health_nodes/peer').level == DiagnosticStatus.OK  # noqa: E731
        assert wait_for(ok, 10.0)
    finally:
        for proc in procs:
            proc.terminate()
            proc.wait(10.0)
        rig.close()


def test_tf_edge_with_a_steady_offset_is_a_clock_problem(ros):
    """An edge updated at 20 Hz but stamped 1.8 s in the past."""
    rig = Rig(tf=True)
    try:
        parent, child = f'{rig.ns[1:]}_lidar_odom', f'{rig.ns[1:]}_lidar_body'
        end = time.monotonic() + 1.5
        while time.monotonic() < end:
            rig.helper.tf_out.send(make_transform(), parent, child, rospy.get_time() - 1.8)
            time.sleep(0.05)
        diag = rig.diagnostic(f'tf:{parent}->{child}')
        assert diag is not None and diag.level == DiagnosticStatus.WARN
        assert 'steady (source clock offset?)' in diag.message
    finally:
        rig.close()


def test_parameters_are_validated_and_live(ros):
    rig = Rig()
    node, ns = rig.node, f'{rig.ns}/health'
    try:
        since = len(node.param_notes)
        rospy.set_param(f'{ns}/nodes/expected', ['no_namespace'])
        noted = lambda: any(n.startswith('rejected') for n in node.param_notes[since:])  # noqa: E731
        assert wait_for(noted, 3.0)
        assert 'fully qualified' in node.param_notes[-1]
        rospy.set_param(f'{ns}/nodes/expected', [])
        rospy.set_param(f'{ns}/sensors/max_age_sec', 1.5)
        assert wait_for(lambda: node.sensor_check.config.max_age_sec == 1.5, 3.0)
    finally:
        rig.close()


def test_core_states_match_the_messages():
    assert [s.value for s in OdometryState] == [
        OdometryStatus.INITIALIZING,
        OdometryStatus.OK,
        OdometryStatus.DEGRADED,
        OdometryStatus.LOST,
    ]
    assert [s.value for s in LocalizationState] == [
        LocalizationStatus.NONE,
        LocalizationStatus.LOCALIZED,
        LocalizationStatus.UNCERTAIN,
    ]
    assert Field.STALE == ControlStatus.FIELD_STALE and Field.BLIND == ControlStatus.FIELD_BLIND
    assert (HealthStatus.OK, HealthStatus.WARN, HealthStatus.ERROR) == (0, 1, 2)


def test_stamp_from_a_real_serialized_message():
    cloud = PointCloud2(height=1, width=3)
    cloud.header.stamp = rospy.Time(1234, 500_000_000)
    cloud.header.frame_id = 'lidar'
    buff = io.BytesIO()
    cloud.serialize(buff)
    assert ros1_stamp(buff.getvalue()) == 1234.5
