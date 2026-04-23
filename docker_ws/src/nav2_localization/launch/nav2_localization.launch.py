#!/usr/bin/env python3
"""
nav2_launch.py
──────────────
Custom Nav2 launch stack — no nav2_bringup, every node and param is explicit.

Usage:
  ros2 launch nav2_launch.py \
      map:=/shared/maps/fnr26_map/fnr26_map.yaml \
      params_file:=/shared/nav2_params.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────────
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="/shared/maps/fnr26_map/fnr26_map.yaml",
        description="Full path to the map YAML file",
    )
    params_arg = DeclareLaunchArgument(
        "params_file",
        default_value="/shared/nav2_params.yaml",
        description="Full path to the Nav2 params YAML file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock",
    )
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start the lifecycle nodes",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level (debug, info, warn, error)",
    )

    map_yaml        = LaunchConfiguration("map")
    params_file     = LaunchConfiguration("params_file")
    use_sim_time    = LaunchConfiguration("use_sim_time")
    autostart       = LaunchConfiguration("autostart")
    log_level       = LaunchConfiguration("log_level")

    # ── Rewrite params to inject use_sim_time everywhere ─────────────────────
    # This guarantees use_sim_time is applied to every node from the one file.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={"use_sim_time": use_sim_time},
        convert_types=True,
    )

    # ── Shared arguments used by every node ───────────────────────────────────
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "collision_monitor",
    ]

    # ── Nodes ─────────────────────────────────────────────────────────────────

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            {"yaml_filename": map_yaml},
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            # Hard-coded safety overrides — these win over the YAML default
            # so base_footprint can never sneak back in.
            {
                "base_frame_id":   "base_link",
                "odom_frame_id":   "odom",
                "global_frame_id": "map",
            },
        ],
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[configured_params],
        # Remap controller output → collision_monitor reads cmd_vel_smoothed
        # and forwards safe commands to /cmd_vel
        remappings=[("cmd_vel", "cmd_vel_smoothed")],
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            {
                "robot_base_frame": "base_link",
                "global_frame":     "map",
                "odom_topic":       "/odometry/filtered",
            },
        ],
    )

    recoveries_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            {"robot_base_frame": "base_link"},
        ],
    )

    costmap_nodes = GroupAction([
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="local_costmap",
            namespace="local_costmap",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                configured_params,
                {"robot_base_frame": "base_link"},
            ],
            remappings=[("scan", "/scan")],
        ),
        Node(
            package="nav2_costmap_2d",
            executable="nav2_costmap_2d",
            name="global_costmap",
            namespace="global_costmap",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                configured_params,
                {"robot_base_frame": "base_link"},
            ],
            remappings=[("scan", "/scan")],
        ),
    ])

    collision_monitor = Node(
        package="nav2_collision_monitor",
        executable="collision_monitor",
        name="collision_monitor",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            configured_params,
            {"base_frame_id": "base_link"},
        ],
    )

    # ── Lifecycle manager ─────────────────────────────────────────────────────
    # One lifecycle manager handles all nodes — it will bring them up in order
    # and shut them down cleanly.
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        arguments=["--ros-args", "--log-level", log_level],
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart":    autostart,
                "node_names":   lifecycle_nodes,
            }
        ],
    )

    return LaunchDescription([
        # Args
        map_arg,
        params_arg,
        use_sim_time_arg,
        autostart_arg,
        log_level_arg,

        # Nodes
        map_server,
        amcl,
        planner_server,
        controller_server,
        bt_navigator,
        recoveries_server,
        costmap_nodes,
        collision_monitor,

        # Lifecycle manager last
        lifecycle_manager,
    ])