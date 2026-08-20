import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _share_path(package, *parts):
    return os.path.join(get_package_share_directory(package), *parts)


def _truthy(value):
    return str(value).lower() in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    ekf_params_file = LaunchConfiguration("ekf_params_file").perform(context)
    use_lidar_odom_in_ekf = _truthy(
        LaunchConfiguration("use_lidar_odom_in_ekf").perform(context))
    if not ekf_params_file:
        ekf_params_file = LaunchConfiguration(
            "lidar_odom_ekf_params_file" if use_lidar_odom_in_ekf else "default_ekf_params_file"
        ).perform(context)

    base_params_file = LaunchConfiguration("base_params_file")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_base = LaunchConfiguration("use_base")
    use_map = LaunchConfiguration("use_map")
    use_rviz = LaunchConfiguration("use_rviz")
    cleanup_existing = LaunchConfiguration("cleanup_existing")
    use_sim_time = LaunchConfiguration("use_sim_time")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    raw_imu_topic = LaunchConfiguration("raw_imu_topic")
    fused_odom_topic = LaunchConfiguration("fused_odom_topic")
    base_cmd_vel_topic = LaunchConfiguration("base_cmd_vel_topic")
    map_topic = LaunchConfiguration("map_topic")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    map_to_odom_y = LaunchConfiguration("map_to_odom_y")
    map_to_odom_z = LaunchConfiguration("map_to_odom_z")
    map_to_odom_roll = LaunchConfiguration("map_to_odom_roll")
    map_to_odom_pitch = LaunchConfiguration("map_to_odom_pitch")
    map_to_odom_yaw = LaunchConfiguration("map_to_odom_yaw")
    ekf_map_frame = LaunchConfiguration("ekf_map_frame")
    ekf_odom_frame = LaunchConfiguration("ekf_odom_frame")
    ekf_world_frame = LaunchConfiguration("ekf_world_frame")
    ekf_base_link_frame = LaunchConfiguration("ekf_base_link_frame")
    ekf_base_link_output_frame = LaunchConfiguration("ekf_base_link_output_frame")
    sigterm_timeout = LaunchConfiguration("sigterm_timeout")
    sigkill_timeout = LaunchConfiguration("sigkill_timeout")
    critical_cpu_cores = LaunchConfiguration("critical_cpu_cores")
    critical_nice_level = LaunchConfiguration("critical_nice_level")
    critical_prefix = [
        "taskset -c ",
        critical_cpu_cores,
        " nice -n ",
        critical_nice_level,
    ]

    cleanup_existing_nodes = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            (
                "set +e; "
                "pkill -TERM -f '[e]kf_fusion_node'; "
                "pkill -TERM -f '[o]rigincarpro_base_node'; "
                "pkill -TERM -f '[s]tatic_map_publisher.py'; "
                "pkill -TERM -f '[m]ap_to_odom_static_tf'; "
                "sleep 1; "
                "pkill -KILL -f '[e]kf_fusion_node'; "
                "pkill -KILL -f '[o]rigincarpro_base_node'; "
                "pkill -KILL -f '[s]tatic_map_publisher.py'; "
                "pkill -KILL -f '[m]ap_to_odom_static_tf'; "
                "true"
            ),
        ],
        output="screen",
        condition=IfCondition(cleanup_existing),
    )

    managed_nodes = [
        Node(
            package="origincarpro_base",
            executable="origincarpro_base_node",
            name="origincarpro_base_node",
            output="screen",
            emulate_tty=True,
            prefix=critical_prefix,
            parameters=[
                base_params_file,
                {"cmd_vel_topic": base_cmd_vel_topic},
            ],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
            condition=IfCondition(use_base),
        ),
        Node(
            package="ekf_fusion",
            executable="ekf_fusion_node",
            name="ekf_fusion_node",
            output="screen",
            emulate_tty=True,
            prefix=critical_prefix,
            parameters=[
                ekf_params_file,
                {
                    "use_sim_time": use_sim_time,
                    "map_frame": ekf_map_frame,
                    "odom_frame": ekf_odom_frame,
                    "world_frame": ekf_world_frame,
                    "base_link_frame": ekf_base_link_frame,
                    "base_link_frame_output": ekf_base_link_output_frame,
                },
            ],
            remappings=[
                ("/odom/data_raw", raw_odom_topic),
                ("/imu/data_raw", raw_imu_topic),
                ("odometry/filtered", fused_odom_topic),
            ],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
        ),
        Node(
            package="ekf_fusion",
            executable="static_map_publisher.py",
            name="ekf_fusion_static_map_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "map_yaml_file": map_yaml_file,
                "map_topic": map_topic,
                "frame_id": map_frame,
                "use_sim_time": use_sim_time,
            }],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
            condition=IfCondition(use_map),
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_static_tf",
            arguments=[
                "--x", map_to_odom_x,
                "--y", map_to_odom_y,
                "--z", map_to_odom_z,
                "--roll", map_to_odom_roll,
                "--pitch", map_to_odom_pitch,
                "--yaw", map_to_odom_yaw,
                "--frame-id", map_frame,
                "--child-frame-id", odom_frame,
            ],
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
            condition=IfCondition(LaunchConfiguration("publish_map_to_odom_tf")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
            sigterm_timeout=sigterm_timeout,
            sigkill_timeout=sigkill_timeout,
            condition=IfCondition(use_rviz),
        ),
    ]

    return [
        cleanup_existing_nodes,
        TimerAction(period=2.0, actions=managed_nodes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "ekf_params_file",
            default_value="",
            description=(
                "Explicit EKF parameter file. When empty, use_lidar_odom_in_ekf "
                "selects default_ekf_params_file or lidar_odom_ekf_params_file."
            ),
        ),
        DeclareLaunchArgument(
            "default_ekf_params_file",
            default_value=_share_path("ekf_fusion", "config", "ekf.yaml"),
        ),
        DeclareLaunchArgument(
            "lidar_odom_ekf_params_file",
            default_value=_share_path("ekf_fusion", "config", "ekf_with_lidar_odom.yaml"),
        ),
        DeclareLaunchArgument(
            "use_lidar_odom_in_ekf",
            default_value="false",
            description="Fuse /lidar_odom pose as odom1. Default false for Stage-A debugging.",
        ),
        DeclareLaunchArgument(
            "base_params_file",
            default_value=_share_path("origincarpro_base", "config", "origincarpro_base.yaml"),
            description="origincarpro_base parameter file.",
        ),
        DeclareLaunchArgument(
            "map_yaml_file",
            default_value=_share_path(
                "ekf_fusion", "maps", "rdk_2026_hospital_static_1cm.yaml"),
            description="Static map yaml file for RViz preview.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=_share_path("ekf_fusion", "rviz", "ekf_fusion_debug.rviz"),
            description="RViz config file.",
        ),
        DeclareLaunchArgument("use_base", default_value="true"),
        DeclareLaunchArgument("use_map", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "cleanup_existing",
            default_value="true",
            description="Stop stale EKF/base/map/TF nodes from previous runs before starting.",
        ),
        DeclareLaunchArgument(
            "publish_map_to_odom_tf",
            default_value="true",
            description=(
                "Publish identity map->odom TF for standalone EKF/RViz debug. "
                "Turn this off when another external localization node publishes map->odom."
            ),
        ),
        DeclareLaunchArgument(
            "map_to_odom_x",
            default_value="0.0",
            description="Static-preview map->odom x translation in metres.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_y",
            default_value="0.0",
            description="Static-preview map->odom y translation in metres.",
        ),
        DeclareLaunchArgument("map_to_odom_z", default_value="0.0"),
        DeclareLaunchArgument("map_to_odom_roll", default_value="0.0"),
        DeclareLaunchArgument("map_to_odom_pitch", default_value="0.0"),
        DeclareLaunchArgument(
            "map_to_odom_yaw",
            default_value="0.0",
            description="Static-preview map->odom yaw in radians.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("raw_odom_topic", default_value="/odom/data_raw"),
        DeclareLaunchArgument("raw_imu_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("fused_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("base_cmd_vel_topic", default_value="/cmd_vel"),
        DeclareLaunchArgument("map_topic", default_value="/map"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("ekf_map_frame", default_value="map"),
        DeclareLaunchArgument("ekf_odom_frame", default_value="odom"),
        DeclareLaunchArgument("ekf_world_frame", default_value="odom"),
        DeclareLaunchArgument("ekf_base_link_frame", default_value="base_link"),
        DeclareLaunchArgument("ekf_base_link_output_frame", default_value="base_link"),
        DeclareLaunchArgument("sigterm_timeout", default_value="2"),
        DeclareLaunchArgument("sigkill_timeout", default_value="2"),
        DeclareLaunchArgument(
            "critical_cpu_cores",
            default_value="0-1",
            description="CPU cores reserved for the chassis bridge and EKF.",
        ),
        DeclareLaunchArgument(
            "critical_nice_level",
            default_value="-5",
            description="Linux nice level for the chassis bridge and EKF.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])


# Standard map->odom->base_link EKF debug:
# ros2 launch ekf_fusion ekf_fusion_only.launch.py
#
# EKF-only LiDAR odom consumer (requires an external /lidar_odom publisher):
# ros2 launch ekf_fusion ekf_fusion_only.launch.py \
#   use_lidar_odom_in_ekf:=true use_map:=false use_rviz:=false \
#   publish_map_to_odom_tf:=false
#
# For the complete driver/filter/LiDAR-odom/EKF chain use:
# ros2 launch ekf_fusion ekf_fusion_only_with_lidar_odom.launch.py
