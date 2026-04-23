from setuptools import setup
import os
from glob import glob

package_name = 'waypoint_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='robot',
    maintainer_email='robot@fnr.com',
    description='Waypoint recording and saving for autonomous navigation',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_saver = waypoint_recorder.waypoint_saver:main',
        ],
    },
)
