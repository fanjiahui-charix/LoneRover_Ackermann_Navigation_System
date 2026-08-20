from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    base_pkg_share = get_package_share_directory("origincarpro_base")
    ekf_pkg_share = get_package_share_directory("ekf_fusion")
    default_base_params = os.path.join(base_pkg_share, "config", "origincarpro_base.yaml")
    default_ekf_params = os.path.join(ekf_pkg_share, "config", "ekf.yaml")

    base_params_arg = DeclareLaunchArgument(
        "base_params_file",
        default_value=default_base_params,
        description="底盘驱动参数文件"
    )

    ekf_params_arg = DeclareLaunchArgument(
        "ekf_params_file",
        default_value=default_ekf_params,
        description="ekf_fusion 参数文件"
    )

    base_node = Node(
        package="origincarpro_base",
        executable="origincarpro_base_node",
        name="origincarpro_base_node",
        output="screen",
        parameters=[LaunchConfiguration("base_params_file")]
    )

    ekf_node = Node(
        package="ekf_fusion",
        executable="ekf_fusion_node",
        name="ekf_fusion_node",
        output="screen",
        parameters=[LaunchConfiguration("ekf_params_file")],
        remappings=[
            ("odometry/filtered", "/odom")
        ]
    )

    return LaunchDescription([
        base_params_arg,
        ekf_params_arg,
        base_node,
        ekf_node
    ])
