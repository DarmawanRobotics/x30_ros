"""teleop_udp_node in this process: a real UDP client against it, fakes of the stack around it."""

from __future__ import annotations

import json
import threading
import time

import pytest
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from teleop_rig import TOKEN, Rig

from x30_bridge.core import protocol
from x30_msgs.msg import ControlStatus, NavigationStatus, RecorderStatus, Waypoint, WaypointPath
from x30_msgs.srv import SetMode, SetModeResponse
from x30_utils.testing.harness import wait_for


@pytest.fixture
def make(ros, tmp_path, monkeypatch):
    monkeypatch.setenv('X30_TELEOP_TOKEN', TOKEN)
    rigs = []

    def factory(**params):
        rigs.append(Rig(tmp_path, **params))
        return rigs[-1]

    yield factory
    for rig in rigs:
        rig.close()


def states(rig, sock, timeout: float = 2.0, until=None) -> list:
    """Return the state messages arriving within timeout (or until until(states) holds)."""
    found, end = [], time.monotonic() + timeout
    while time.monotonic() < end and not (until is not None and found and until(found)):
        found += [m for m in rig.received(sock) if m['t'] == 'state']
        time.sleep(0.02)
    return found


def test_a_handheld_is_welcomed_and_follows_the_robot(make):
    rig = make()
    a = rig.client()
    rig.send(a, {'t': 'hello', 'name': 'h12'})
    welcome = next(m for m in rig.received(a) if m['t'] == 'welcome')
    assert welcome['robot'] == 'x30' and welcome['limits'] == [0.8, 0.4, 1.0]
    assert {'recorder_start', 'recorder_stop', 'inspect', 'nav_start', 'set_mode',
            'list_routes', 'stand_up'} <= set(welcome['commands'])  # fmt: skip
    assert welcome['robot_commands'][:2] == ['stand_up', 'sit_down']
    assert welcome['cameras'] == ['visible', 'thermal']
    control = rig.publisher('control/status', ControlStatus)
    recorder = rig.publisher('recorder/status', RecorderStatus)
    nav = rig.publisher('navigation/status', NavigationStatus)
    control.publish(ControlStatus(mode=ControlStatus.MODE_REMOTE, field=ControlStatus.FIELD_STOP))
    nav.publish(NavigationStatus(state=NavigationStatus.FOLLOWING, index=11, count=240))
    session = '/rec/20261121/101500_patrol'
    recorder.publish(RecorderStatus(recording=True, reason='patrol', thermal_frames=60,
                                    session=session, free_gb=88.0))  # fmt: skip
    found = states(
        rig, a, 5.0, lambda s: len(s) > 1 and s[-1]['recorder'] and s[-1]['mode'] == 'remote'
    )
    last = found[-1]
    assert (last['mode'], last['field'], last['nav']) == ('remote', 'stop', 'FOLLOWING')
    assert last['mission'] == 'waypoint 12/240' and last['camera'] == 'visible'
    assert last['recorder']['on'] and last['recorder']['session'] == '101500_patrol'
    assert last['recorder']['frames'] == 60 and not last['recorder']['offline']
    assert last['substation'] is None and last['inspection'] is None  # no inspection running
    seqs = [s['seq'] for s in found]
    assert len(seqs) >= 2 and seqs == sorted(seqs)


def test_a_wrong_token_is_denied_and_drives_nothing(make):
    rig = make()
    a = rig.client()
    rig.send(a, json.dumps({'t': 'hello'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'hello', 'k': 'wrong'}).encode(), raw=True)
    rig.send(a, json.dumps({'t': 'joy', 'vx': 0.5, 'k': 'wrong'}).encode(), raw=True)
    assert [m['t'] for m in rig.received(a)] == ['denied']
    assert rig.twists.messages == []


def test_the_stick_is_clamped_garbage_stops_and_silence_zeroes_it(make):
    rig = make(client_timeout_sec=0.5)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'joy', 'vx': 5.0, 'vy': -5.0, 'wz': 0.3})
    assert rig.twists.wait_until(lambda t: t.linear.x == 0.8, 3.0)
    t = rig.twists.last
    assert (t.linear.x, t.linear.y, t.angular.z) == (0.8, -0.4, pytest.approx(0.3))
    for bad in (b'NaN', b'Infinity', b'"fast"', b'[1]', b'true'):
        rig.send(a, b'{"k": "secret", "t": "joy", "vx": ' + bad + b'}', raw=True)
        assert rig.twists.wait_until(lambda t: t.linear.x == 0.0, 3.0), bad
    rig.send(a, {'t': 'joy', 'vx': 0.5})
    assert rig.twists.wait_until(lambda t: t.linear.x == 0.5, 3.0)
    assert rig.twists.wait_until(lambda t: t.linear.x == 0.0, 3.0)  # the handheld went silent


def test_a_second_device_is_busy_while_the_first_drives(make):
    rig = make()
    a, b = rig.client('127.0.0.1'), rig.client('127.0.0.2')
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    rig.send(b, {'t': 'hello'})
    assert [m['t'] for m in rig.received(b)] == ['busy']
    rig.send(b, {'t': 'joy', 'vx': 0.5})
    assert all(t.linear.x == 0.0 for t in rig.twists.messages)
    a2 = rig.client('127.0.0.1')  # app restart: the same device from a new port
    rig.send(a2, {'t': 'hello'})
    assert any(m['t'] == 'welcome' for m in rig.received(a2))


def test_commands_call_the_stack_once_and_are_acknowledged(make):
    rig = make()
    modes = []

    def set_mode(request):
        modes.append((request.mode, request.source))
        return SetModeResponse(True, 'remote')

    rig.serve('control/set_mode', SetMode, set_mode)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.received(a)
    for cid, name, args in ((1, 'set_mode', {'mode': 'remote'}), (1, 'set_mode', {}),
                            (2, 'fly', {}), (3, 'load_route', {'file': '../../etc/x.csv'}),
                            (4, 'set_mode', {'mode': 'fly'})):  # fmt: skip
        rig.send(a, {'t': 'cmd', 'id': cid, 'name': name, 'args': args}, settle=0.05)
    acks = rig.acks(a, 3.0, lambda acks: any(m['ok'] for m in acks.get(1, [])))
    assert modes == [('remote', 'handheld')]  # resent (id, name): run once
    assert any(m['ok'] is True for m in acks[1])
    assert acks[2][-1]['ok'] is False and 'unknown command' in acks[2][-1]['msg']
    assert acks[3][-1]['ok'] is False and 'under' in acks[3][-1]['msg']
    assert acks[4][-1]['ok'] is False and 'auto or remote' in acks[4][-1]['msg']


def test_the_estop_latches_at_once_and_a_slow_inspection_blocks_nothing(make):
    rig = make()
    release = threading.Event()
    answer = TriggerResponse(True, 'bay1: 2/2 targets')
    rig.serve('inspection/inspect', Trigger, lambda _r: release.wait(5.0) and answer)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 7, 'name': 'inspect'})
    rig.send(a, {'t': 'cmd', 'id': 8, 'name': 'estop'})
    assert rig.estops.wait_until(lambda m: m.data == 'handheld estop', 3.0)
    rig.send(a, {'t': 'ping', 'ts': 5})
    assert {'t': 'pong', 'ts': 5} in rig.received(a)  # the gateway answers meanwhile
    rig.send(a, {'t': 'cmd', 'id': 7, 'name': 'inspect'})  # resent: still running
    acks = rig.acks(a, 4.0, lambda acks: 7 in acks and 8 in acks)
    assert acks[7][-1]['ok'] is None and acks[8][-1]['ok'] is False  # no mux in this test
    release.set()
    done = rig.acks(a, 5.0, lambda acks: 7 in acks)
    assert done[7][-1] == {'t': 'ack', 'id': 7, 'ok': True, 'msg': 'bay1: 2/2 targets'}


def test_the_patrol_speed_is_the_followers_parameter(make):
    rig = make()
    param = f'{rig.ns}/path_follower_node/speed/cruise'
    rospy.set_param(param, 0.4)
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    found = states(rig, a, 5.0, lambda s: s[-1]['speed'] is not None)
    assert found[-1]['speed'] == {'target': 0.4, 'min': 0.1, 'max': 0.6}
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'nav_speed', 'args': {'speed': 0.25}})
    rig.send(a, {'t': 'cmd', 'id': 2, 'name': 'nav_speed', 'args': {'speed': 0.9}})
    acks = rig.acks(a, 3.0, lambda acks: 1 in acks and 2 in acks)
    assert acks[1][-1]['ok'] and not acks[2][-1]['ok']
    assert rospy.get_param(param) == 0.25
    found = states(rig, a, 5.0, lambda s: (s[-1]['speed'] or {}).get('target') == 0.25)
    assert found[-1]['speed']['target'] == 0.25


def test_routes_are_listed_and_the_loaded_one_streamed(make, tmp_path):
    (tmp_path / 'gardu' / 'routes').mkdir(parents=True)
    (tmp_path / 'gardu' / 'routes' / 'loop.csv').write_text('x,y\n')
    rig = make()
    a = rig.client()
    rig.send(a, {'t': 'hello'})
    rig.send(a, {'t': 'cmd', 'id': 1, 'name': 'list_routes'})
    acks = rig.acks(a, 2.0, lambda acks: 1 in acks)
    assert acks[1][-1]['data'] == ['gardu/routes/loop.csv']
    waypoints = rig.publisher('navigation/waypoints', WaypointPath)
    waypoints.publish(WaypointPath(waypoints=[Waypoint(x=1.234, y=2.0, dwell_sec=10.0),
                                              Waypoint(x=3.0, y=4.0)]))  # fmt: skip
    route, reassembler, end = None, protocol.Reassembler(), time.monotonic() + 5.0
    while route is None and time.monotonic() < end:
        for datagram in rig.binary(a):
            frame = reassembler.push(datagram)
            if frame and frame[0] == protocol.STREAM_ROUTE and json.loads(frame[2])['points']:
                route = json.loads(frame[2])['points']
    assert route == [[1.23, 2.0, 10.0], [3.0, 4.0, 0.0]]


def test_discovery_needs_no_token_and_tells_who_drives(make):
    rig = make()
    a, b = rig.client(), rig.client('127.0.0.2')
    rig.send(a, json.dumps({'t': 'discover'}).encode(), raw=True)
    (beacon,) = rig.received(a)
    assert beacon['t'] == 'beacon' and beacon['secured'] is True and beacon['in_use_by'] == ''
    rig.send(a, {'t': 'hello', 'name': 'h12'})
    rig.send(b, json.dumps({'t': 'discover'}).encode(), raw=True)
    assert rig.received(b)[0]['in_use_by'] == 'h12'


def test_streams_are_subscribed_only_while_wanted(make):
    rig = make()
    a = rig.client()
    assert rig.node.streams.subs == {} and rig.node.streams.tf is None
    rig.send(a, {'t': 'hello', 'streams': {'camera': True, 'lidar': True}})
    assert set(rig.node.streams.subs) == {'camera', 'lidar'}
    assert rig.node.streams.subs['camera'].name == '/hikmicro/visible/image_raw/compressed'
    rig.send(a, {'t': 'streams', 'camera_name': 'thermal', 'lidar': False})
    assert rig.node.streams.subs['camera'].name == '/hikmicro/thermal/image_raw/compressed'
    assert 'lidar' not in rig.node.streams.subs
    rig.send(a, {'t': 'bye'})
    assert wait_for(lambda: rig.node.streams.subs == {} and rig.node.streams.tf is None, 2.0)
    rig.send(a, {'t': 'hello'})  # a handheld without streams: no camera, no cloud
    assert rig.node.streams.subs == {} and rig.node.streams.tf is not None
