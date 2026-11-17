"""Configuration entries: topics, rules, edges and node names are validated on start."""

from __future__ import annotations

import pytest

from x30_health.core.config import HealthConfig
from x30_health.core.specs import parse_edge, parse_rule, parse_topic, parse_topics
from x30_utils.config import ConfigError, build


def test_topic_entries():
    assert parse_topic('/a:5') == parse_topic('/a:5.0')
    spec = parse_topic(' /b/c:12.5:0.3 ')
    assert (spec.topic, spec.min_hz, spec.max_age_sec) == ('/b/c', 12.5, 0.3)
    assert parse_topic('/d').min_hz == 0.0
    assert parse_topic('/d').max_age_sec is None  # the default applies at run time
    assert parse_topic('/e:1:0').max_age_sec == 0.0  # no stamp check
    assert [s.topic for s in parse_topics(['/a:1', '', ' ', '/b:2'])] == ['/a', '/b']


@pytest.mark.parametrize('spec', ['', ':5', '/a:x', '/a:-1', '/a:1:2:3', '/a:nan', '/a:1:inf'])
def test_invalid_topic_entries(spec):
    with pytest.raises(ValueError):
        parse_topic(spec)


def test_duplicate_topics():
    with pytest.raises(ValueError, match='twice'):
        parse_topics(['/a:1', '/a:2'])


def test_rules_and_edges():
    rule = parse_rule('sensor:/livox/*=0.5')
    assert rule.scale == 0.5 and rule.matches('sensor:/livox/lidar')
    assert not rule.matches('sensor:/Odometry')
    assert parse_rule('tf:a->b=0').scale == 0.0  # '=' splits at the last one
    for bad in ('nope', '=0.5', 'x=2', 'x=-1', 'x=y'):
        with pytest.raises(ValueError):
            parse_rule(bad)
    assert parse_edge(' /camera_init -> body ') == ('camera_init', 'body')
    for bad in ('a', 'a->', '->b'):
        with pytest.raises(ValueError):
            parse_edge(bad)


def test_defaults_build():
    config = build(HealthConfig)
    assert config.state.localization_topic == 'localization/status'
    assert not config.clock.enabled  # chrony is optional
    assert config.nodes.expected == ()


@pytest.mark.parametrize(
    ('name', 'value', 'match'),
    [
        ('sensors.topics', ['/a:x'], 'sensors.topics'),
        ('sensors.topics', ['/a:1', '/a:2'], 'twice'),
        ('degradation.rules', ['x=3'], 'degradation.rules'),
        ('tf.edges', ['map'], 'tf.edges'),
        ('nodes.expected', ['control_mux_node'], 'fully qualified'),
        ('nodes.expected', ['/ns/'], 'fully qualified'),
        ('system.max_temp_c', -1.0, 'below'),
    ],
)
def test_invalid_values_are_rejected(name, value, match):
    with pytest.raises(ConfigError, match=match):
        build(HealthConfig, {name: value})


def test_empty_list_entries_from_yaml_are_dropped():
    """ROS cannot type an empty list in YAML: [""] means none."""
    config = build(HealthConfig, {'nodes.expected': [''], 'tf.edges': ['']})
    assert config.nodes.expected == () and config.tf.edges == ()
