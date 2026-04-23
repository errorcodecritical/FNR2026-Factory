from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os

def generate_launch_description():
    # Declare a launch argument for the params file
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            get_package_share_directory('ekf_launch'),
            'params',
            'ekf.yaml'
        ),
        description='Full path to the EKF parameters file'
    )

    # Declare autostart argument for lifecycle control
    autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically configure and activate the node. Set to false for managed lifecycle control.'
    )

    # LifecycleNode to launch the EKF with custom parameters
    ekf_node = launch_ros.actions.LifecycleNode(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='',
        output='screen',
        autostart=LaunchConfiguration('autostart'),
        parameters=[LaunchConfiguration('params_file')]
    )

    return LaunchDescription([
        params_file,
        autostart,
        ekf_node
    ])