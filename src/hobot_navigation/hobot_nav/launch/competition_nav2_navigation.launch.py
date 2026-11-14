from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    minimal_action_stack = LaunchConfiguration('minimal_action_stack')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')
    vehicle_profile_file = LaunchConfiguration('vehicle_profile_file')
    controller_cmd_vel_topic = LaunchConfiguration('controller_cmd_vel_topic')
    nav_to_pose_bt_xml = LaunchConfiguration('nav_to_pose_bt_xml')
    nav_through_poses_bt_xml = LaunchConfiguration('nav_through_poses_bt_xml')
    container_name_full = (namespace, '/', container_name)

    # All four Nav2/Supervisor map participants use the same deterministic
    # UDP-only profile.  Camera/BEV and the remaining perception/base nodes do
    # not inherit this environment.
    nav_udp_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(
            Path(get_package_share_directory('hobot_nav')) /
            'config' / 'nav_udp_fastdds.xml'),
        'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        'ROS_DISABLE_LOANED_MESSAGES': '1',
    }

    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

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
    vehicle_profile_params = ParameterFile(
        RewrittenYaml(
            source_file=vehicle_profile_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    standalone_nodes = GroupAction(
        condition=UnlessCondition(use_composition),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                additional_env=nav_udp_env,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', controller_cmd_vel_topic)],
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                condition=UnlessCondition(minimal_action_stack),
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                additional_env=nav_udp_env,
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                condition=UnlessCondition(minimal_action_stack),
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                # Generic Nav2 recovery behaviors must enter the same command
                # chain as controller commands.
                remappings=remappings + [
                    ('cmd_vel', controller_cmd_vel_topic),
                ],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                condition=UnlessCondition(minimal_action_stack),
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                condition=UnlessCondition(minimal_action_stack),
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                # Camera zero-copy launch sets this process-wide variable to
                # 0 while launch descriptions are expanded. VelocitySmoother
                # does not use const-reference subscription callbacks on this
                # Humble build and can segfault on loaned Twist messages.
                additional_env={'ROS_DISABLE_LOANED_MESSAGES': '1'},
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, vehicle_profile_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [
                    # controller_cmd_vel_topic is the raw controller output;
                    # the smoother emits the temporal-filtered command on the
                    # public nav topic consumed by the final limiter.
                    ('cmd_vel', controller_cmd_vel_topic),
                    ('cmd_vel_smoothed', 'cmd_vel_nav'),
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                condition=UnlessCondition(minimal_action_stack),
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_minimal_action_stack',
                condition=IfCondition(minimal_action_stack),
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': [
                        'controller_server', 'planner_server', 'velocity_smoother']},
                ],
                arguments=['--ros-args', '--log-level', log_level],
            ),
        ],
    )

    full_composed_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression([
            "'", use_composition, "' == 'true' and '",
            minimal_action_stack, "' == 'false'",
        ])),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings + [('cmd_vel', controller_cmd_vel_topic)],
            ),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings + [
                    ('cmd_vel', controller_cmd_vel_topic),
                ],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings + [
                    ('cmd_vel', controller_cmd_vel_topic),
                    ('cmd_vel_smoothed', 'cmd_vel_nav'),
                ],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes,
                }],
            ),
        ],
    )

    minimal_composed_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression([
            "'", use_composition, "' == 'true' and '",
            minimal_action_stack, "' == 'true'",
        ])),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings + [('cmd_vel', controller_cmd_vel_topic)],
            ),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings,
            ),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params, vehicle_profile_params],
                remappings=remappings + [
                    ('cmd_vel', controller_cmd_vel_topic),
                    ('cmd_vel_smoothed', 'cmd_vel_nav'),
                ],
            ),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_minimal_action_stack',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': [
                        'controller_server', 'planner_server', 'velocity_smoother'],
                }],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument(
            'minimal_action_stack', default_value='false',
            description=(
                'Competition mode: launch only controller/planner/velocity '
                'smoother. The C++ supervisor owns high-level orchestration.')),
        DeclareLaunchArgument('container_name', default_value='nav2_container'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'nav_to_pose_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'behavior_trees',
                'navigate_to_pose_ackermann_recovery.xml',
            ]),
        ),
        DeclareLaunchArgument(
            'nav_through_poses_bt_xml',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'behavior_trees',
                'navigate_through_poses_ackermann_recovery.xml',
            ]),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'config',
                'competition_nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'vehicle_profile_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'config',
                'competition_vehicle_profile.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'controller_cmd_vel_topic',
            default_value='cmd_vel_nav_raw',
            description=(
                'Raw controller output before velocity_smoother; keep '
                'cmd_vel_nav_raw so /cmd_vel_nav remains smoother-only.'
            ),
        ),
        standalone_nodes,
        full_composed_nodes,
        minimal_composed_nodes,
    ])
