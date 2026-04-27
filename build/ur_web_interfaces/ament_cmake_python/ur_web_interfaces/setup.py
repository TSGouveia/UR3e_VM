from setuptools import find_packages
from setuptools import setup

setup(
    name='ur_web_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('ur_web_interfaces', 'ur_web_interfaces.*')),
)
