from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'minimu9_publisher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='user@example.com',
    description='ROS2 node for publishing Pololu MinIMU-9 v5 sensor data',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'minimu9_node = minimu9_publisher.minimu9_node:main',
        ],
    },
)