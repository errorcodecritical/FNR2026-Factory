"""
Camera launch file - publishes only compressed JPEG images
Subscribes to v4l2_camera's image_raw and republishes only the compressed version
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Start v4l2_camera node (will publish to /camera/image_raw internally)
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            namespace='camera',
            parameters=['/shared/camera_params.yaml'],
        ),
        # Image transport node to republish as compressed only
        # This subscribes to /camera/image_raw and republishes as /camera/image_raw/compressed
        Node(
            package='image_transport',
            executable='republish',
            namespace='camera',
            remappings=[
                ('in', 'image_raw'),
                ('out', 'image_raw/compressed'),
            ],
            arguments=['raw', 'compressed'],
            parameters=[{
                'compressed_image_transport.jpeg_quality': 80,
            }],
        ),
    ])
