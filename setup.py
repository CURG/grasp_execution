## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# requirements = [line.strip() for line in open("requirements.txt")]

d = generate_distutils_setup()

d['name'] = "grasp_execution"
d['description'] = "Code for interfacing with Movit to check reachability of grasps"
d['packages'] = ['grasp_execution']
d['package_dir'] = {'': 'src'}
# d['requirements'] = requirements

setup(**d)
