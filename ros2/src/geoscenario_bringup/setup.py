import os
from glob import glob
from setuptools import setup

package_name = 'geoscenario_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Spencer Delcore, Michal Antkiewicz',
    maintainer_email='sdelcore@uwaterloo.ca',
    description='Launch files and end-to-end tests for GeoScenario',
    license='MIT',
    tests_require=['pytest', 'launch_testing'],
    entry_points={
        'console_scripts': [],
    },
)
