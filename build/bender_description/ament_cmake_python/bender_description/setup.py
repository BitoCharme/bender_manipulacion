from setuptools import find_packages
from setuptools import setup

setup(
    name='bender_description',
    version='0.0.1',
    packages=find_packages(
        include=('bender_description', 'bender_description.*')),
)
