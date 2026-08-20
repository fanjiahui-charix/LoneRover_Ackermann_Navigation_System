import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def _ekf_node(context, *args, **kwargs):
    params = [LaunchConfiguration('ekf_params_file').perform(context)]
    overlay = LaunchConfiguration('ekf_overlay_file').perform(context).strip()
    if overlay:
        params.append(overlay)
    params.append({
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context).lower() == 'true',
    })
    return [Node(
        package='ekf_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        parameters=params,
        remappings=[('odometry/filtered', '/odom')],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level').perform(context)],
    )]


def _static_frames_node(context, *args, **kwargs):
    if LaunchConfiguration('publish_static_frames').perform(context).lower() not in (
        '1', 'true', 'yes'):
        return []
    return [Node(
        package='hobot_nav',
        executable='publish_static_frames.py',
        name='static_frames_publisher',
        output='screen',
        parameters=[
            LaunchConfiguration('frames_params_file').perform(context),
            ParameterFile(
                LaunchConfiguration('vehicle_profile_file').perform(context),
                allow_substs=True),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('launch_base', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('base_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('origincarpro_base'), 'config', 'origincarpro_base.yaml'])),
        DeclareLaunchArgument('base_cmd_vel_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('ekf_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('ekf_fusion'), 'config', 'ekf_with_lidar_odom.yaml'])),
        DeclareLaunchArgument('ekf_overlay_file', default_value=''),
        DeclareLaunchArgument('publish_static_frames', default_value='false'),
        DeclareLaunchArgument('frames_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'config', 'frames.yaml'])),
        DeclareLaunchArgument('vehicle_profile_file', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'config', 'vehicle_profile.yaml'])),
        Node(
            condition=IfCondition(LaunchConfiguration('launch_base')),
            package='origincarpro_base',
            executable='origincarpro_base_node',
            name='origincarpro_base_node',
            output='screen',
            parameters=[
                LaunchConfiguration('base_params_file'),
                {'cmd_vel_topic': LaunchConfiguration('base_cmd_vel_topic')},
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        ),
        OpaqueFunction(function=_static_frames_node),
        OpaqueFunction(function=_ekf_node),
    ])
