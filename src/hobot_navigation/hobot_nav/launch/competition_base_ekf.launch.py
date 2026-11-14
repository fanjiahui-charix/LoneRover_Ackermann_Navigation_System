import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (EnvironmentVariable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def _ekf_node(context, *args, **kwargs):
    params = [LaunchConfiguration('ekf_params_file').perform(context)]
    overlay = LaunchConfiguration('ekf_overlay_file').perform(context)
    if overlay:
        params.append(overlay)
    params.append({'use_sim_time':
                   LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'})
    return [Node(
        package='ekf_fusion',
        executable='ekf_fusion_node',
        name='ekf_fusion_node',
        output='screen',
        prefix=[
            'nice -n ',
            LaunchConfiguration('critical_nice_level').perform(context),
        ],
        arguments=[
            '--ros-args', '--log-level',
            LaunchConfiguration('log_level').perform(context),
        ],
        parameters=params,
        remappings=[('odometry/filtered', '/odom')],
    )]


def _static_frames_node(context, *args, **kwargs):
    enabled = LaunchConfiguration(
        'publish_static_frames').perform(context).lower() in ('1', 'true', 'yes')
    if not enabled:
        return []

    parameters = [
        LaunchConfiguration('frames_params_file').perform(context),
        ParameterFile(
            LaunchConfiguration('vehicle_profile_file').perform(context),
            allow_substs=True),
    ]
    camera_tf_file = LaunchConfiguration(
        'camera_tf_file').perform(context).strip()
    vehicle_id = LaunchConfiguration('vehicle_id').perform(context).strip()
    calibration_root = os.path.expanduser(
        LaunchConfiguration('calibration_root').perform(context).strip())
    if camera_tf_file:
        camera_tf_file = os.path.expanduser(camera_tf_file)
    elif vehicle_id:
        if not all(character.isalnum() or character in '_-'
                   for character in vehicle_id):
            raise RuntimeError('vehicle_id contains unsafe characters')
        camera_tf_file = os.path.join(
            calibration_root, vehicle_id, 'camera_tf.yaml')
    if camera_tf_file:
        if not os.path.isfile(camera_tf_file):
            raise RuntimeError(
                f'camera calibration TF does not exist: {camera_tf_file}')
        parameters.append(camera_tf_file)

    return [Node(
        package='hobot_nav',
        executable='competition_publish_static_frames.py',
        name='static_frames_publisher',
        output='screen',
        arguments=[
            '--ros-args', '--log-level',
            LaunchConfiguration('log_level').perform(context),
        ],
        parameters=parameters,
    )]


def generate_launch_description():
    launch_base = LaunchConfiguration('launch_base')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_params_file = LaunchConfiguration('base_params_file')
    base_cmd_vel_topic = LaunchConfiguration('base_cmd_vel_topic')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    log_level = LaunchConfiguration('log_level')
    critical_nice_level = LaunchConfiguration('critical_nice_level')
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_base',
            default_value='true',
            description='Whether to launch origincarpro_base together with EKF.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'critical_nice_level',
            default_value='0',
            description=(
                'Nice level for the chassis feedback and EKF processes. '
                'Competition uses -5 to protect scan-time TF publication.'),
        ),
        DeclareLaunchArgument(
            'base_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('origincarpro_base'),
                'config',
                'origincarpro_base.yaml',
            ]),
            description='Full path to origincarpro_base parameter file.',
        ),
        DeclareLaunchArgument(
            'base_cmd_vel_topic',
            default_value='/cmd_vel',
            description='Velocity command topic consumed by origincarpro_base.',
        ),
        DeclareLaunchArgument(
            'ekf_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ekf_fusion'),
                'config',
                'ekf_with_lidar_odom.yaml',
            ]),
            description='Full path to ekf_fusion parameter file.',
        ),
        DeclareLaunchArgument(
            'ekf_overlay_file',
            default_value='',
            description='Optional parameter overlay applied after ekf_params_file.',
        ),
        DeclareLaunchArgument(
            'publish_static_frames',
            default_value='false',
            description='Whether to publish fallback static TFs from competition_frames.yaml.',
        ),
        DeclareLaunchArgument(
            'frames_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'config',
                'competition_frames.yaml',
            ]),
            description='Full path to static frame parameter file.',
        ),
        DeclareLaunchArgument(
            'vehicle_profile_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'config',
                'competition_vehicle_profile.yaml',
            ]),
            description='Centralized real-vehicle override file for TF and navigation geometry.',
        ),
        DeclareLaunchArgument(
            'vehicle_id',
            default_value=EnvironmentVariable(
                'MYCAR_VEHICLE_ID', default_value=''),
            description='Vehicle calibration ID used for camera TF'),
        DeclareLaunchArgument(
            'calibration_root',
            default_value=EnvironmentVariable(
                'MYCAR_CALIBRATION_ROOT',
                default_value=os.path.expanduser('~/calib')),
            description='Per-vehicle calibration root'),
        DeclareLaunchArgument(
            'camera_tf_file',
            default_value='',
            description='Explicit camera_tf.yaml path; overrides vehicle path'),
        Node(
            condition=IfCondition(launch_base),
            package='origincarpro_base',
            executable='origincarpro_base_node',
            name='origincarpro_base_node',
            output='screen',
            prefix=['nice -n ', critical_nice_level],
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[base_params_file, {
                'cmd_vel_topic': base_cmd_vel_topic,
                # Single-owner contract: camera TF is published only by the
                # calibrated static_frames_publisher below.
                'publish_camera_alias_tf': False,
            }],
        ),
        OpaqueFunction(function=_static_frames_node),
        OpaqueFunction(function=_ekf_node),
    ])
