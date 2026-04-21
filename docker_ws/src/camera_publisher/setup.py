from setuptools import setup

package_name = 'camera_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/camera_compressed.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='FNR26',
    author_email='todo@todo.com',
    maintainer='FNR26',
    maintainer_email='todo@todo.com',
    description='ROS2 camera publisher with compressed JPEG output',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
