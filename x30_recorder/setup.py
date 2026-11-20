from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_recorder', 'x30_recorder.core', 'x30_recorder.utils'],
        package_dir={'': 'src'},
    )
)
