from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'maps',
            'rdk_2026_hospital_static_1cm.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{'yaml_filename': map_file,
                                         'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                         'node_names': ['map_server']}],
        ),
    ])
