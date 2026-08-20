#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("lidar_local_planner"),
        "config",
        "local_scan_planner.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the planner parameter file",
        ),
        Node(
            package="lidar_local_planner",
            executable="local_scan_planner_node",
            name="local_scan_planner",
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
