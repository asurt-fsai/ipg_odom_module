"""
Install the lidar library
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(packages=["state_machine"], package_dir={"": "src/state_machine"})

setup(**d)
