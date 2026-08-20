#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory("lidar_web_viewer"),
        "config",
        "lidar_web_viewer.yaml",
    )

    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="Path to the viewer parameter file",
        ),
        Node(
            package="lidar_web_viewer",
            executable="lidar_web_viewer.py",
            name="lidar_web_viewer",
            output="screen",
            emulate_tty=True,
            parameters=[params_file],
        ),
    ])
