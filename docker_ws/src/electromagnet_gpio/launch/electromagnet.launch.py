from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='/shared/eletro_params.yaml',
        description='Absolute path to the electromagnet parameter file.',
    )

    electromagnet_gpio_node = Node(
        package='electromagnet_gpio',
        executable='electromagnet_gpio_node',
        name='electromagnet_gpio',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        electromagnet_gpio_node,
    ])
