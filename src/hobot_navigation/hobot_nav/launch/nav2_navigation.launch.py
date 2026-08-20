from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    vehicle_profile_file = LaunchConfiguration('vehicle_profile_file')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    controller_cmd_vel_topic = LaunchConfiguration('controller_cmd_vel_topic')
    log_level = LaunchConfiguration('log_level')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
                'default_nav_through_poses_bt_xml': nav_through_poses_bt_xml,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )
    vehicle_params = ParameterFile(vehicle_profile_file, allow_substs=True)
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    common = {
        'output': 'screen',
        'parameters': [configured_params, vehicle_params],
        'arguments': ['--ros-args', '--log-level', log_level],
        'respawn': False,
        'respawn_delay': 2.0,
        'remappings': remappings,
    }

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'config', 'nav2_params.yaml'])),
        DeclareLaunchArgument('vehicle_profile_file', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'config', 'vehicle_profile.yaml'])),
        DeclareLaunchArgument('controller_cmd_vel_topic', default_value='cmd_vel_nav_raw'),
        DeclareLaunchArgument('nav_to_pose_bt_xml', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'behavior_trees',
            'navigate_to_pose_ackermann_recovery.xml'])),
        DeclareLaunchArgument('nav_through_poses_bt_xml', default_value=PathJoinSubstitution([
            FindPackageShare('hobot_nav'), 'behavior_trees',
            'navigate_through_poses_ackermann_recovery.xml'])),
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', **dict(
                 common, remappings=remappings + [('cmd_vel', controller_cmd_vel_topic)])),
        Node(package='nav2_smoother', executable='smoother_server',
             name='smoother_server', **common),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', **common),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', **dict(
                 common, remappings=remappings + [('cmd_vel', controller_cmd_vel_topic)])),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', **common),
        Node(package='nav2_waypoint_follower', executable='waypoint_follower',
             name='waypoint_follower', **common),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother',
             name='velocity_smoother', **dict(
                 common, remappings=remappings + [
                     ('cmd_vel', controller_cmd_vel_topic),
                     ('cmd_vel_smoothed', 'cmd_vel_nav')])),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'autostart': autostart,
                         'node_names': [
                             'controller_server', 'smoother_server', 'planner_server',
                             'behavior_server', 'bt_navigator', 'waypoint_follower',
                             'velocity_smoother']}],
            arguments=['--ros-args', '--log-level', log_level],
        ),
    ])
