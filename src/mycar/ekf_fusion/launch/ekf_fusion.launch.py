import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    del args, kwargs
    params_file = LaunchConfiguration("params_file").perform(context)
    use_lidar = LaunchConfiguration("use_lidar_odom_in_ekf").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    if not params_file:
        config_name = "ekf_with_lidar_odom.yaml" if use_lidar else "ekf.yaml"
        params_file = os.path.join(
            get_package_share_directory("ekf_fusion"),
            "config",
            config_name,
        )

    return [
        Node(
            package="ekf_fusion",
            executable="ekf_fusion_node",
            name="ekf_fusion_node",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
            remappings=[("odometry/filtered", "/odom")],
        )
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="",
            description=(
                "Optional explicit ekf_fusion parameter file. When empty, "
                "use_lidar_odom_in_ekf selects ekf.yaml or ekf_with_lidar_odom.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "use_lidar_odom_in_ekf",
            default_value="false",
            description="Fuse /lidar_odom pose as odom1. Default false for Stage-A debugging.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
