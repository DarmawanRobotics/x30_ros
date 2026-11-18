from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_inspection', 'x30_inspection.core', 'x30_inspection.utils'],
        package_dir={'': 'src'},
    )
)
