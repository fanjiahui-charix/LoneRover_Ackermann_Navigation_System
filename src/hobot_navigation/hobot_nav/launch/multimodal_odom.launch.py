import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


VALID_MODES = ('final', 'conservative', 'wheel_imu_only', 'lidar_disabled')


def _mode_includes(context, *args, **kwargs):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in VALID_MODES:
        raise RuntimeError(f"Unsupported mode '{mode}'; choose one of {VALID_MODES}")

    hobot_share = get_package_share_directory('hobot_nav')
    ekf_share = get_package_share_directory('ekf_fusion')
    lidar_odom_share = get_package_share_directory('simple_lidar_odom')

    # wheel_imu_only intentionally keeps the LiDAR frontend alive for quality
    # diagnostics while using ekf.yaml (no odom1). lidar_disabled is the
    # lowest-load fallback and stops lidar odometry entirely.
    requested_lidar_odom = LaunchConfiguration(
        'enable_lidar_odom').perform(context).lower() in ('1', 'true', 'yes')
    lidar_enabled = requested_lidar_odom and mode != 'lidar_disabled'
    fuse_lidar = lidar_enabled and mode in ('final', 'conservative')
    conservative = mode == 'conservative'

    base_args = {
        'launch_base': LaunchConfiguration('launch_base').perform(context),
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
        'base_cmd_vel_topic': LaunchConfiguration('base_cmd_vel_topic').perform(context),
        'log_level': LaunchConfiguration('log_level').perform(context),
        'critical_nice_level': LaunchConfiguration(
            'critical_nice_level').perform(context),
        'publish_static_frames': LaunchConfiguration(
            'publish_static_frames').perform(context),
        'vehicle_profile_file': LaunchConfiguration(
            'vehicle_profile_file').perform(context),
        'camera_tf_file': LaunchConfiguration(
            'camera_tf_file').perform(context),
        'ekf_params_file': os.path.join(
            ekf_share, 'config',
            'ekf_with_lidar_odom.yaml' if fuse_lidar else 'ekf.yaml'),
        'ekf_overlay_file': os.path.join(
            ekf_share, 'config', 'ekf_conservative_overlay.yaml') if conservative else '',
    }
    lidar_args = {
        'serial_port': LaunchConfiguration('serial_port').perform(context),
        'enable_cone_detector': LaunchConfiguration(
            'enable_cone_detector').perform(context),
        'log_level': LaunchConfiguration('log_level').perform(context),
        'enable_lidar_odom': 'true' if lidar_enabled else 'false',
        'lidar_odom_overlay_file': os.path.join(
            lidar_odom_share, 'config',
            'lidar_odom_conservative_overlay.yaml') if conservative else '',
    }

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hobot_share, 'launch', 'competition_base_ekf.launch.py')),
            launch_arguments=base_args.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hobot_share, 'launch', 'n10_lidar_pipeline.launch.py')),
            launch_arguments=lidar_args.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='final',
            description=(
                'final/conservative fuse odom1; wheel_imu_only keeps LiDAR '
                'diagnostics without fusion; lidar_disabled stops LiDAR odometry')),
        DeclareLaunchArgument(
            'enable_lidar_odom', default_value='false',
            description='Explicit debug switch. Formal competition keeps this false.'),
        DeclareLaunchArgument('launch_base', default_value='true'),
        DeclareLaunchArgument(
            'base_cmd_vel_topic', default_value='/cmd_vel',
            description='The single velocity topic consumed by the chassis.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'critical_nice_level',
            default_value='0',
            description='Nice level forwarded to chassis feedback and EKF.'),
        DeclareLaunchArgument(
            'enable_cone_detector',
            default_value='true',
            description=(
                'Start semantic cone outputs; filtered /scan is independent '
                'and remains active when false.')),
        DeclareLaunchArgument('publish_static_frames', default_value='false'),
        DeclareLaunchArgument(
            'vehicle_profile_file',
            default_value=os.path.join(
                get_package_share_directory('hobot_nav'),
                'config', 'competition_vehicle_profile.yaml')),
        DeclareLaunchArgument('camera_tf_file', default_value=''),
        OpaqueFunction(function=_mode_includes),
    ])
