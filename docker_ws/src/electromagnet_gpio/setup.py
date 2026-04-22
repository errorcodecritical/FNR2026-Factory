from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'electromagnet_gpio'

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
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS 2 Raspberry Pi GPIO node for the factory robot electromagnet.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'electromagnet_gpio_node = electromagnet_gpio.electromagnet_gpio_node:main',
        ],
    },
)
