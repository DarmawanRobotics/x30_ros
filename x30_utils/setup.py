from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=[
            'x30_utils',
            'x30_utils.config',
            'x30_utils.grid',
            'x30_utils.io',
            'x30_utils.log',
            'x30_utils.math',
            'x30_utils.plugins',
            'x30_utils.robot',
            'x30_utils.ros',
            'x30_utils.safety',
            'x30_utils.testing',
        ],
        package_dir={'': 'src'},
    )
)
