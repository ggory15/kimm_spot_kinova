from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['kimm_spot_teleop'],
    scripts=['scripts/kimm_spot_teleop.py'],
    package_dir={'': 'src'}
)

setup(**d)