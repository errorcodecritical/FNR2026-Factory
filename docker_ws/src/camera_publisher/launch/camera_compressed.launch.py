"""
Camera launch file - uses libcamera via camera_ros for RPi camera support
Publishes both raw and compressed JPEG images
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start camera_ros node - native libcamera support for Raspberry Pi
        Node(
            package='camera_ros',
            executable='camera_ros_node',
            namespace='camera',
            parameters=[
                '/shared/camera_params.yaml',
                {
                    'camera_info_url': 'file:///shared/camera_info/unicam.yaml',
                }
            ],
        ),
        # Image transport node to republish as compressed JPEG
        # This subscribes to /camera/image and republishes as /camera/image/compressed
        Node(
            package='image_transport',
            executable='republish',
            namespace='camera',
            remappings=[
                ('in', 'image'),
                ('out', 'image_compressed'),
            ],
            arguments=['raw', 'compressed'],
            parameters=[{
                'use_sensor_data_qos': True,
            }],
        ),
    ])
