from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/shared/autonomy_params.yaml',
        description='Absolute path to the factory autonomy parameter file.',
    )

    autonomy_node = Node(
        package='factory_autonomy',
        executable='autonomy_node',
        name='factory_autonomy',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        autonomy_node,
    ])
