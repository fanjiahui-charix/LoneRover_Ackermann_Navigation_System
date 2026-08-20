from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("origincarpro_base")
    default_params = os.path.join(pkg_share, "config", "origincarpro_base.yaml")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=default_params,
        description="底盘驱动参数文件"
    )

    base_node = Node(
        package="origincarpro_base",
        executable="origincarpro_base_node",
        name="origincarpro_base_node",
        output="screen",
        parameters=[LaunchConfiguration("params_file")]
    )

    return LaunchDescription([
        params_file_arg,
        base_node
    ])
