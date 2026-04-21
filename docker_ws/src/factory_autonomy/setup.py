from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'factory_autonomy'

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
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Autonomous crate scheduling and Nav2 goal dispatch for the factory robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autonomy_node = factory_autonomy.autonomy_node:main',
        ],
    },
)
