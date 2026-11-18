from __future__ import annotations

from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(
    **generate_distutils_setup(
        packages=['x30_mapping', 'x30_mapping.core', 'x30_mapping.utils'],
        package_dir={'': 'src'},
    )
)
