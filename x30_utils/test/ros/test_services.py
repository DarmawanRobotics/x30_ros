"""ServiceClient and AsyncCall: readiness, answers, failures and timeouts, reported once."""

from __future__ import annotations

import itertools
import threading
import time

import pytest
import rospy
from std_srvs.srv import Trigger, TriggerRequest

from x30_utils.ros import X30Node
from x30_utils.ros.services import AsyncCall, ServiceClient
from x30_utils.testing.harness import wait_for

NAMES = itertools.count()


@pytest.fixture
def server(ros):
    """A node serving <ns>/ok, <ns>/slow (2 s) and <ns>/broken."""
    ns = f'/services_test_{next(NAMES)}'
    node = X30Node('server', param_ns=f'{ns}/server')

    def ok(_request, response):
        response.success, response.message = True, 'done'
        return response

    def slow(_request, response):
        time.sleep(2.0)
        response.success = True
        return response

    def broken(_request, _response):
        raise ValueError('nope')

    node.create_service(Trigger, f'{ns}/ok', ok)
    node.create_service(Trigger, f'{ns}/slow', slow, concurrent=True)
    node.create_service(Trigger, f'{ns}/broken', broken)
    for name in ('ok', 'slow', 'broken'):
        rospy.wait_for_service(f'{ns}/{name}', 5.0)
    yield ns
    node.destroy_node()


def test_blocking_calls_answer_fail_or_time_out(server):
    ok = ServiceClient(Trigger, f'{server}/ok')
    assert ok.ready() and ok.name == f'{server}/ok'
    response, error = ok.call(TriggerRequest(), 2.0)
    assert error == '' and response.success and response.message == 'done'
    response, error = ServiceClient(Trigger, f'{server}/broken').call(TriggerRequest(), 2.0)
    assert response is None and 'failed' in error
    started = time.monotonic()
    response, error = ServiceClient(Trigger, f'{server}/slow').call(TriggerRequest(), 0.5)
    assert response is None and 'timed out' in error and time.monotonic() - started < 1.5
    missing = ServiceClient(Trigger, f'{server}/missing')
    assert not missing.ready()
    assert missing.call(TriggerRequest(), 0.3) == (None, f'{server}/missing not available')


def test_async_calls_report_once_as_a_node_callback(server):
    node = X30Node('caller', param_ns=f'{server}/caller')
    results = {}
    inside = threading.Event()

    def record(name):
        def done(ok, message):
            results.setdefault(name, []).append((ok, message))
            if node._lock._is_owned():
                inside.set()

        return done

    try:
        for name in ('ok', 'broken', 'slow', 'missing'):
            client = ServiceClient(Trigger, f'{server}/{name}')
            AsyncCall(node, client, TriggerRequest(), 0.5, record(name))
        assert wait_for(lambda: len(results) == 4, 5.0), results
        time.sleep(2.0)  # the slow answer comes after its timeout: not reported again
        assert results['ok'] == [(True, 'done')]
        assert results['broken'][0][0] is False and 'failed' in results['broken'][0][1]
        assert results['slow'] == [(False, f'{server}/slow timed out')]
        assert results['missing'] == [(False, f'{server}/missing not available')]
        assert inside.is_set()  # under the node's lock, like its other callbacks
    finally:
        node.destroy_node()
