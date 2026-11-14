from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _lidar_odom_node(context, *args, **kwargs):
    params = [LaunchConfiguration('lidar_odom_params_file').perform(context)]
    overlay = LaunchConfiguration('lidar_odom_overlay_file').perform(context)
    if overlay:
        params.append(overlay)
    params.append({
        'scan_topic': LaunchConfiguration('filtered_scan_topic').perform(context),
        'odom_topic': LaunchConfiguration('lidar_odom_topic').perform(context),
        'use_scan_deskew': True,
        'deskew_source': LaunchConfiguration('lidar_odom_deskew_source').perform(context),
        'reset_pose_on_map_reset': LaunchConfiguration(
            'lidar_odom_reset_pose_on_map_reset').perform(context).lower() == 'true',
        'ekf_odom_topic': LaunchConfiguration('ekf_odom_topic').perform(context),
        'raw_odom_topic': LaunchConfiguration('raw_odom_topic_for_lidar_odom').perform(context),
        'imu_topic': LaunchConfiguration('imu_topic_for_lidar_odom').perform(context),
        'publish_tf': False,
    })
    return [Node(
        condition=IfCondition(LaunchConfiguration('enable_lidar_odom')),
        package='simple_lidar_odom',
        executable='lidar_odom_node',
        name='simple_lidar_odom',
        output='screen',
        arguments=[
            '--ros-args', '--log-level',
            LaunchConfiguration('log_level').perform(context),
        ],
        parameters=params,
    )]


def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port')
    frame_id = LaunchConfiguration('frame_id')
    raw_scan_topic = LaunchConfiguration('raw_scan_topic')
    filtered_scan_topic = LaunchConfiguration('filtered_scan_topic')
    deskewed_scan_topic = LaunchConfiguration('deskewed_scan_topic')
    cone_points_topic = LaunchConfiguration('cone_points_topic')
    cone_poses_topic = LaunchConfiguration('cone_poses_topic')
    cone_markers_topic = LaunchConfiguration('cone_markers_topic')
    min_range = LaunchConfiguration('min_range')
    max_range = LaunchConfiguration('max_range')
    cone_max_range = LaunchConfiguration('cone_max_range')
    range_scale = LaunchConfiguration('range_scale')
    range_bias = LaunchConfiguration('range_bias')
    angle_offset_deg = LaunchConfiguration('angle_offset_deg')
    scan_period = LaunchConfiguration('scan_period')
    enable_deskew = LaunchConfiguration('enable_deskew')
    enable_cone_detector = LaunchConfiguration('enable_cone_detector')
    enable_lidar_odom = LaunchConfiguration('enable_lidar_odom')
    enable_web_viewer = LaunchConfiguration('enable_web_viewer')
    use_rviz = LaunchConfiguration('use_rviz')
    lidar_params_file = LaunchConfiguration('lidar_params_file')
    perception_params_file = LaunchConfiguration('perception_params_file')
    lidar_odom_params_file = LaunchConfiguration('lidar_odom_params_file')
    lidar_odom_topic = LaunchConfiguration('lidar_odom_topic')
    lidar_odom_deskew_source = LaunchConfiguration('lidar_odom_deskew_source')
    lidar_odom_reset_pose_on_map_reset = LaunchConfiguration('lidar_odom_reset_pose_on_map_reset')
    ekf_odom_topic = LaunchConfiguration('ekf_odom_topic')
    raw_odom_topic_for_lidar_odom = LaunchConfiguration('raw_odom_topic_for_lidar_odom')
    imu_topic_for_lidar_odom = LaunchConfiguration('imu_topic_for_lidar_odom')
    web_viewer_params_file = LaunchConfiguration('web_viewer_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    log_level = LaunchConfiguration('log_level')

    filter_input_topic = PythonExpression([
        "'", deskewed_scan_topic, "' if '", enable_deskew, "' == 'true' else '", raw_scan_topic, "'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyS1',
            description='Serial device for the N10 driver.',
        ),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_link',
            description='LaserScan frame id and perception output frame.',
        ),
        DeclareLaunchArgument(
            'raw_scan_topic',
            default_value='/scan_raw',
            description='Raw LaserScan topic from n10_driver.',
        ),
        DeclareLaunchArgument(
            'filtered_scan_topic',
            default_value='/scan',
            description='Filtered LaserScan topic for SLAM, Nav2, and collision monitor.',
        ),
        DeclareLaunchArgument(
            'deskewed_scan_topic',
            default_value='/scan_deskewed',
            description='Reserved output topic for the optional deskew placeholder.',
        ),
        DeclareLaunchArgument(
            'cone_points_topic',
            default_value='/cones/points',
            description='Detected cone centers as PointCloud2.',
        ),
        DeclareLaunchArgument(
            'cone_poses_topic',
            default_value='/cones/poses',
            description='Detected cone centers as PoseArray.',
        ),
        DeclareLaunchArgument(
            'cone_markers_topic',
            default_value='/cones/markers',
            description='Detected cone RViz markers.',
        ),
        DeclareLaunchArgument(
            'min_range',
            default_value='0.15',
            description='Minimum usable scan range in meters.',
        ),
        DeclareLaunchArgument(
            'max_range',
            default_value='6.0',
            description='Maximum filtered scan range in meters.',
        ),
        DeclareLaunchArgument(
            'cone_max_range',
            default_value='4.5',
            description='Maximum cone detection range in meters.',
        ),
        DeclareLaunchArgument(
            'range_scale',
            default_value='1.0',
            description='Linear range calibration scale applied by scan_filter_node.',
        ),
        DeclareLaunchArgument(
            'range_bias',
            default_value='0.0',
            description='Linear range calibration bias in meters applied by scan_filter_node.',
        ),
        DeclareLaunchArgument(
            'angle_offset_deg',
            default_value='0.0',
            description='LaserScan angular zero correction in degrees.',
        ),
        DeclareLaunchArgument(
            'scan_period',
            default_value='0.10',
            description='Fallback scan period used when the driver does not fill time_increment.',
        ),
        DeclareLaunchArgument(
            'enable_deskew',
            default_value='false',
            description='Start the placeholder scan_deskew_node before filtering.',
        ),
        DeclareLaunchArgument(
            'enable_cone_detector',
            default_value='true',
            description='Start cone_detector_node.',
        ),
        DeclareLaunchArgument(
            'enable_lidar_odom',
            default_value='true',
            description='Start simple_lidar_odom on the filtered /scan output.',
        ),
        DeclareLaunchArgument(
            'enable_web_viewer',
            default_value='false',
            description='Start the lightweight browser scan viewer.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz with the existing navigation debug config.',
        ),
        DeclareLaunchArgument(
            'lidar_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('n10_driver'),
                'params',
                'n10.yaml',
            ]),
            description='Parameter file for the current n10_driver package.',
        ),
        DeclareLaunchArgument(
            'perception_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lidar_perception'),
                'config',
                'lidar_perception.yaml',
            ]),
            description='Parameter file for lidar_perception nodes.',
        ),
        DeclareLaunchArgument(
            'lidar_odom_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('simple_lidar_odom'),
                'config',
                'lidar_odom.yaml',
            ]),
            description='Parameter file for simple_lidar_odom.',
        ),
        DeclareLaunchArgument(
            'lidar_odom_overlay_file',
            default_value='',
            description='Optional parameter overlay applied after lidar_odom_params_file.',
        ),
        DeclareLaunchArgument(
            'lidar_odom_topic',
            default_value='/lidar_odom',
            description='Output odometry topic from simple_lidar_odom.',
        ),
        DeclareLaunchArgument(
            'lidar_odom_deskew_source',
            default_value='raw_odom',
            description='Deskew source for simple_lidar_odom.',
        ),
        DeclareLaunchArgument(
            'lidar_odom_reset_pose_on_map_reset',
            default_value='true',
            description='Whether reset_lidar_odom_map also resets the /lidar_odom pose.',
        ),
        DeclareLaunchArgument(
            'ekf_odom_topic',
            default_value='/odom',
            description='EKF odometry topic used only as lagged twist for simple_lidar_odom.',
        ),
        DeclareLaunchArgument(
            'raw_odom_topic_for_lidar_odom',
            default_value='/odom/data_raw',
            description='Raw odom fallback topic for simple_lidar_odom deskew.',
        ),
        DeclareLaunchArgument(
            'imu_topic_for_lidar_odom',
            default_value='/imu/data_raw',
            description='Raw IMU fallback topic for simple_lidar_odom deskew.',
        ),
        DeclareLaunchArgument(
            'web_viewer_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('lidar_web_viewer'),
                'config',
                'lidar_web_viewer.yaml',
            ]),
            description='Parameter file for lidar_web_viewer.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'rviz',
                'slam_nav_debug.rviz',
            ]),
            description='RViz config file.',
        ),
        Node(
            package='n10_driver',
            executable='n10_driver_node',
            name='n10_driver_node',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                lidar_params_file,
                {
                    'frame_id': frame_id,
                    'scan_topic': raw_scan_topic,
                    'serial_port': serial_port,
                    'min_range': min_range,
                    'max_range': max_range,
                    'scan_time': scan_period,
                    'use_measured_scan_time': True,
                },
            ],
        ),
        Node(
            condition=IfCondition(enable_deskew),
            package='lidar_perception',
            executable='scan_deskew_node',
            name='scan_deskew_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                perception_params_file,
                {
                    'scan_topic': raw_scan_topic,
                    'output_topic': deskewed_scan_topic,
                    'output_frame': frame_id,
                    'scan_period': scan_period,
                    'enable_deskew': True,
                },
            ],
        ),
        Node(
            package='lidar_perception',
            executable='scan_filter_node',
            name='scan_filter_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                perception_params_file,
                {
                    'input_topic': filter_input_topic,
                    'output_topic': filtered_scan_topic,
                    'target_frame': frame_id,
                    'min_range': min_range,
                    'max_range': max_range,
                    'range_scale': range_scale,
                    'range_bias': range_bias,
                    'angle_offset_deg': angle_offset_deg,
                },
            ],
        ),
        Node(
            condition=IfCondition(enable_cone_detector),
            package='lidar_perception',
            executable='cone_detector_node',
            name='cone_detector_node',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                perception_params_file,
                {
                    'scan_topic': filtered_scan_topic,
                    'cone_points_topic': cone_points_topic,
                    'cone_poses_topic': cone_poses_topic,
                    'marker_topic': cone_markers_topic,
                    'output_frame': frame_id,
                    'min_range': min_range,
                    'max_range': cone_max_range,
                },
            ],
        ),
        OpaqueFunction(function=_lidar_odom_node),
        Node(
            condition=IfCondition(enable_web_viewer),
            package='lidar_web_viewer',
            executable='lidar_web_viewer.py',
            name='lidar_web_viewer',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[
                web_viewer_params_file,
                {
                    'scan_topic': filtered_scan_topic,
                },
            ],
        ),
        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', rviz_config,
                '--ros-args', '--log-level', log_level,
            ],
        ),
    ])
