#!/usr/bin/env python3
"""config/driver_params.yaml documents every parameter the node reads, with its defaults."""

import ast
import os
import sys
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PACKAGE, 'src'))
from x30_driver import protocol  # noqa: E402


def node_parameters():
    """Return {name: default} of every param('name', default) call in driver.py."""
    with open(os.path.join(PACKAGE, 'src', 'x30_driver', 'driver.py')) as f:
        tree = ast.parse(f.read())
    found = {}
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
                and node.func.id in ('param', 'rate') and len(node.args) == 2
                and isinstance(node.args[0], ast.Constant)):
            source = compile(ast.Expression(node.args[1]), 'driver.py', 'eval')
            found[node.args[0].value] = eval(source, {'p': protocol})  # noqa: S307
    return found


class TestParameters(unittest.TestCase):
    def test_yaml_matches_the_node(self):
        with open(os.path.join(PACKAGE, 'config', 'driver_params.yaml')) as f:
            documented = yaml.safe_load(f)
        defaults = node_parameters()
        self.assertGreater(len(defaults), 40)
        self.assertEqual(sorted(documented), sorted(defaults))
        for name, value in documented.items():
            self.assertEqual(value, defaults[name], name)

    def test_overrides_are_known(self):
        defaults = node_parameters()
        for name in ('fake_robot.yaml',):
            with open(os.path.join(PACKAGE, 'config', name)) as f:
                self.assertLessEqual(set(yaml.safe_load(f)), set(defaults), name)
        with open(os.path.join(HERE, 'smoke_params.yaml')) as f:
            self.assertLessEqual(set(yaml.safe_load(f)), set(defaults))


if __name__ == '__main__':
    unittest.main()
