from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup

setup(**generate_distutils_setup(packages=['x30_driver'], package_dir={'': 'src'}))
