from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('params_file'),
        Node(
            package='mycar_navigation',
            executable='local_navigator_node',
            name='local_navigator',
            output='screen',
            parameters=[
                params_file,
                {'map_yaml': map_yaml},
            ],
        ),
    ])
