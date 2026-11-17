"""Sensor topics: rate and header stamp age (H1), clock offset, silence, raw stamps."""

from __future__ import annotations

from dataclasses import replace
import struct

from x30_health.core.checks.base import Level
from x30_health.core.checks.sensors import SensorCheck
from x30_health.core.config import SensorsConfig
from x30_health.core.stamp import ros1_stamp, starts_with_header


def check(*topics: str, **kwargs) -> SensorCheck:
    sensors = SensorCheck(SensorsConfig(topics=topics, **kwargs))
    for topic in sensors.topics:
        sensors.subscribed(topic)
    return sensors


def feed(sensors, topic, hz, seconds, age, start=100.0):
    n = int(hz * seconds)
    for i in range(n):
        sensors.on_message(topic, start + i / hz, age)
    return start + (n - 1) / hz


def test_fresh_10hz_topic_is_ok():
    sensors = check('/lidar:10')
    now = feed(sensors, '/lidar', 10.0, 3.0, age=0.05)
    (issue,) = sensors.run(now)
    assert issue.level is Level.OK and issue.name == 'sensor:/lidar'
    assert abs(issue.values['hz'] - 10.0) < 0.1 and issue.values['age_sec'] == 0.05


def test_10hz_topic_with_2s_old_stamps_warns():
    """H1: the rate alone passed a topic whose data is 2 s late."""
    sensors = check('/lidar:10')
    now = feed(sensors, '/lidar', 10.0, 3.0, age=2.0)
    (issue,) = sensors.run(now)
    assert issue.level is Level.WARN
    assert 'stamps 2.00 s old (max 0.50)' in issue.message


def test_stamps_from_the_future_are_a_clock_offset():
    sensors = check('/lidar:10')
    now = feed(sensors, '/lidar', 10.0, 2.0, age=-1.2)
    (issue,) = sensors.run(now)
    assert issue.level is Level.WARN
    assert '1.20 s in the future (clock offset?)' in issue.message
    small = check('/lidar:10')
    assert small.run(feed(small, '/lidar', 10.0, 2.0, age=-0.05))[0].ok  # within max_future


def test_per_topic_age_limit_and_off():
    sensors = check('/slow:1:5.0', '/any:1:0')
    feed(sensors, '/slow', 2.0, 3.0, age=3.0)
    now = feed(sensors, '/any', 2.0, 3.0, age=100.0)
    assert all(i.ok for i in sensors.run(now))


def test_default_age_limit_is_live():
    sensors = check('/a:1')
    now = feed(sensors, '/a', 2.0, 3.0, age=0.8)
    assert not sensors.run(now)[0].ok
    sensors.configure(replace(sensors.config, max_age_sec=1.0))
    assert sensors.run(now)[0].ok


def test_slow_and_silent_topics():
    sensors = check('/lidar:10')
    now = feed(sensors, '/lidar', 5.0, 3.0, age=None)
    (issue,) = sensors.run(now)
    assert issue.level is Level.WARN and 'Hz (min 10.0)' in issue.message
    assert 'age_sec' not in issue.values  # no header: no age
    assert sensors.run(now + 10.0)[0].values['hz'] == 0.0


def test_unpublished_and_single_message_topics():
    """Empty window: no division by zero, a single message is 0 Hz."""
    sensors = SensorCheck(SensorsConfig(topics=('/a:5', '/b:5')))
    sensors.subscribed('/b')
    sensors.on_message('/b', 10.0)
    a, b = sensors.run(10.0)
    assert a.level is Level.WARN and a.message == '/a not published'
    assert b.level is Level.WARN and b.values['hz'] == 0.0


def test_rate_tolerance():
    sensors = check('/a:10', rate_tolerance=0.8)
    now = feed(sensors, '/a', 8.5, 3.0, age=0.0)
    assert sensors.run(now)[0].ok


def test_ros1_stamp_after_the_seq():
    data = struct.pack('<III', 7, 1700000000, 250_000_000) + struct.pack('<I', 4) + b'base'
    assert abs(ros1_stamp(data) - 1700000000.25) < 1e-6
    assert ros1_stamp(data[:12]) is None  # too short for a header
    assert ros1_stamp(struct.pack('<IIII', 0, 1, 2_000_000_000, 0)) is None  # bad nsecs


def test_header_detection_from_the_message_definition():
    cloud = '# a comment\nstd_msgs/Header header\nuint32 height\n'
    imu = 'Header header\ngeometry_msgs/Quaternion orientation\n'
    livox = 'uint8 NOISE=1  # a constant first\nstd_msgs/Header header\nuint64 timebase\n'
    assert starts_with_header(cloud) and starts_with_header(imu) and starts_with_header(livox)
    assert not starts_with_header('string data\n')
    assert not starts_with_header('float64 x\n====\nMSG: std_msgs/Header\nuint32 seq\n')
    assert not starts_with_header('')


def test_published_topic_without_its_type_installed():
    sensors = SensorCheck(SensorsConfig(topics=('/livox/lidar:5',)))
    sensors.unavailable('/livox/lidar', 'type livox_ros_driver2/CustomMsg not installed')
    (issue,) = sensors.run(0.0)
    assert issue.level is Level.WARN
    assert issue.message == (
        '/livox/lidar not subscribed: type livox_ros_driver2/CustomMsg not installed'
    )


def test_topics_are_resolved():
    sensors = SensorCheck(SensorsConfig(topics=('cloud:5',)), lambda t: f'/ns/{t}')
    assert list(sensors.topics) == ['/ns/cloud']
    assert sensors.run(0.0)[0].name == 'sensor:/ns/cloud'
