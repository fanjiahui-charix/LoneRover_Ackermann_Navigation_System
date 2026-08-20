from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('launch_base', default_value='true'),
        DeclareLaunchArgument('launch_lidar_pipeline', default_value='true'),
        DeclareLaunchArgument('launch_static_map', default_value='true'),
        DeclareLaunchArgument('enable_lidar_odom', default_value='true'),
        DeclareLaunchArgument('enable_web_viewer', default_value='false'),
        DeclareLaunchArgument('publish_static_frames', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'maps',
            'rdk_2026_hospital_static_1cm.yaml'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'base_ekf.launch.py'])),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'launch_base': LaunchConfiguration('launch_base'),
                'publish_static_frames': LaunchConfiguration('publish_static_frames'),
                'base_cmd_vel_topic': '/cmd_vel_safe',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'lidar_pipeline.launch.py'])),
            condition=IfCondition(LaunchConfiguration('launch_lidar_pipeline')),
            launch_arguments={
                'enable_lidar_odom': LaunchConfiguration('enable_lidar_odom'),
                'enable_web_viewer': LaunchConfiguration('enable_web_viewer'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'static_map.launch.py'])),
            condition=IfCondition(LaunchConfiguration('launch_static_map')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'nav2_navigation.launch.py'])),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),
        Node(
            package='hobot_nav', executable='ackermann_command_limiter',
            name='ackermann_command_limiter', output='screen',
            parameters=[{
                'input_topic': '/cmd_vel_nav',
                'output_topic': '/cmd_vel_safe',
                'reverse_only_topic': '/navigation/reverse_only',
                'direct_gate_topic': '/navigation/direct_gate_active',
                'profile_max_speed': 0.70,
                'profile_max_reverse_speed': 0.35,
                'min_turning_radius': 0.35,
                'lateral_accel_limit': 0.80,
            }],
        ),
    ])
