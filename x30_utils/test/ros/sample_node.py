"""A node for the run() tests: a shutdown hook publishes 'bye', a stuck mode blocks a callback."""

from __future__ import annotations

from dataclasses import dataclass
import sys
import time

from std_msgs.msg import String

from x30_utils.config import param
from x30_utils.ros import LATCHED, X30Node, run


@dataclass(frozen=True)
class SampleConfig:
    rate: float = param(10.0, min=0.1, max=100.0, unit='Hz', live=False)
    stuck: bool = param(False, live=False, doc='block in a timer callback')


class SampleNode(X30Node):
    def __init__(self):
        super().__init__('sample_node')
        self.config = self.declare_config(SampleConfig).value
        self.pub = self.create_publisher(String, 'sample/out', LATCHED)
        self.pub.publish(String(data='ready'))
        self.add_shutdown_hook(lambda: self.pub.publish(String(data='bye')))
        self.every(1.0 / self.config.rate, self.tick, steady=True)

    def tick(self):
        if self.config.stuck:
            self.pub.publish(String(data='stuck'))
            time.sleep(60.0)


if __name__ == '__main__':
    run(SampleNode, 'sample_node', argv=sys.argv)
