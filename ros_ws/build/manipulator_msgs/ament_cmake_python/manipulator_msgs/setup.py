from setuptools import find_packages
from setuptools import setup

setup(
    name='manipulator_msgs',
    version='1.0.0',
    packages=find_packages(
        include=('manipulator_msgs', 'manipulator_msgs.*')),
)
