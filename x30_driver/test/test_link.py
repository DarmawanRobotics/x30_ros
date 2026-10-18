#!/usr/bin/env python3
"""UdpLink on the loopback interface."""

import os
import queue
import sys
import time
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from x30_driver import protocol as p  # noqa: E402
from x30_driver.link import UdpLink  # noqa: E402


def wait(predicate, timeout=2.0):
    """Poll until predicate() holds; return its last value."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


class TestUdpLink(unittest.TestCase):
    def setUp(self):
        self.inbox = queue.Queue()
        self.server = UdpLink('127.0.0.1', 9, '127.0.0.1', 0,
                              lambda data, address: self.inbox.put((data, address)))
        self.server.start()
        self.replies = queue.Queue()
        self.client = UdpLink('127.0.0.1', self.server.local[1], '127.0.0.1', 0,
                              lambda data, address: self.replies.put(data))

    def tearDown(self):
        self.client.stop()
        self.server.stop()

    def test_send_before_start_fails(self):
        self.assertFalse(self.client.send(b'x'))
        self.assertEqual(self.client.last_error, 'link not started')
        self.assertFalse(self.client.started)

    def test_round_trip(self):
        """Datagrams arrive in order with the sender's address; counters follow."""
        self.client.start()
        self.assertTrue(self.client.started)
        data = [p.simple(p.code(p.HEARTBEAT)), p.simple(p.code(p.CONNECT))]
        self.assertEqual(self.client.send_all(data), 2)
        got = [self.inbox.get(timeout=2.0) for _ in data]
        self.assertEqual([d for d, _ in got], data)
        self.assertEqual(got[0][1], ('127.0.0.1', self.client.local[1]))
        self.assertTrue(wait(lambda: self.server.rx_count == 2))
        self.assertEqual(self.client.tx_count, 2)
        self.assertLess(self.server.age(), 2.0)
        self.assertEqual(self.client.age(), float('inf'))

    def test_reply_to_another_address(self):
        """send() takes an explicit address (the perception host has two ports)."""
        self.client.start()
        self.assertTrue(self.server.send(b'reply', ('127.0.0.1', self.client.local[1])))
        self.assertEqual(self.replies.get(timeout=2.0), b'reply')

    def test_failing_callback_keeps_the_loop(self):
        def explode(data, address):
            if data == b'bad':
                raise ValueError('boom')
            self.inbox.put((data, address))

        self.server._on_datagram = explode
        self.client.start()
        self.client.send(b'bad')
        self.client.send(b'good')
        self.assertEqual(self.inbox.get(timeout=2.0)[0], b'good')
        self.assertEqual(self.server.rx_errors, 1)
        self.assertIn('boom', self.server.last_error)

    def test_stop_is_idempotent(self):
        self.client.start()
        self.client.stop()
        self.client.stop()
        self.assertFalse(self.client.send(b'x'))


if __name__ == '__main__':
    unittest.main()
