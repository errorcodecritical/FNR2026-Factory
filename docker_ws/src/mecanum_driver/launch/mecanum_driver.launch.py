"""
mecanum_driver.launch.py
------------------------
Launches both the motor_driver node (serial ↔ Arduino) and the
mecanum_driver node (cmd_vel ↔ odometry) driven entirely by a YAML
parameter file.

Typical usage
-------------
  ros2 launch mecanum_driver mecanum_driver.launch.py \
      params_file:=/shared/mecanum_params.yaml

Node graph
----------
  [motor_driver]    ←─── /motor{0-3}/cmd    ───  [mecanum_driver]
  [motor_driver]    ───► /motor{0-3}/encoder ──► [mecanum_driver]
  [mecanum_driver] ◄──── /cmd_vel
  [mecanum_driver] ────► /odom + /tf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def generate_launch_description():
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        description='Absolute path to the YAML parameter file, e.g. '
                    'params_file:=/shared/mecanum_params.yaml')

    # ================================================================== #
    # Nodes                                                               #
    # ================================================================== #

    motor_driver_node = Node(
        package='mecanum_driver',
        executable='motor_driver',
        name='motor_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    mecanum_driver_node = Node(
        package='mecanum_driver',
        executable='mecanum_driver',
        name='mecanum_driver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        params_file_arg,
        motor_driver_node,
        mecanum_driver_node,
    ])