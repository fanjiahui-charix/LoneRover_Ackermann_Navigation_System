from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    use_namespace = LaunchConfiguration('use_namespace')
    namespace = LaunchConfiguration('namespace')
    rviz_config = LaunchConfiguration('rviz_config')

    rviz_node = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    namespaced_rviz_config = ReplaceString(
        source_file=rviz_config,
        replacements={'<robot_namespace>': ('/', namespace)},
    )

    namespaced_rviz_node = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config],
        output='screen',
        remappings=[
            ('/map', 'map'),
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/goal_pose', 'goal_pose'),
            ('/clicked_point', 'clicked_point'),
            ('/initialpose', 'initialpose'),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_namespace', default_value='false'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'rviz',
                'slam_nav_debug.rviz',
            ]),
            description='Full path to the RViz config file to use.',
        ),
        rviz_node,
        namespaced_rviz_node,
        RegisterEventHandler(
            condition=UnlessCondition(use_namespace),
            event_handler=OnProcessExit(
                target_action=rviz_node,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        ),
        RegisterEventHandler(
            condition=IfCondition(use_namespace),
            event_handler=OnProcessExit(
                target_action=namespaced_rviz_node,
                on_exit=EmitEvent(event=Shutdown(reason='rviz exited')),
            ),
        ),
    ])
