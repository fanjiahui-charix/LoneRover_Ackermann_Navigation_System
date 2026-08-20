from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    raw_scan = LaunchConfiguration('raw_scan_topic')
    filtered_scan = LaunchConfiguration('filtered_scan_topic')
    deskewed_scan = LaunchConfiguration('deskewed_scan_topic')
    filter_input = PythonExpression([
        "'", deskewed_scan, "' if '", LaunchConfiguration('enable_deskew'),
        "' == 'true' else '", raw_scan, "'",
    ])

    return LaunchDescription([
        DeclareLaunchArgument('raw_scan_topic', default_value='/scan_raw',
                              description='LaserScan topic from an external lidar driver.'),
        DeclareLaunchArgument('filtered_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('deskewed_scan_topic', default_value='/scan_deskewed'),
        DeclareLaunchArgument('enable_deskew', default_value='false'),
        DeclareLaunchArgument('enable_lidar_odom', default_value='true'),
        DeclareLaunchArgument('enable_web_viewer', default_value='false'),
        DeclareLaunchArgument('frame_id', default_value='laser_link'),
        DeclareLaunchArgument('perception_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('lidar_perception'), 'config', 'lidar_perception.yaml'])),
        DeclareLaunchArgument('lidar_odom_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('simple_lidar_odom'), 'config', 'lidar_odom.yaml'])),
        DeclareLaunchArgument('web_viewer_params_file', default_value=PathJoinSubstitution([
            FindPackageShare('lidar_web_viewer'), 'config', 'lidar_web_viewer.yaml'])),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_deskew')),
            package='lidar_perception', executable='scan_deskew_node',
            name='scan_deskew_node', output='screen',
            parameters=[{
                'scan_topic': raw_scan,
                'odom_topic': '/odom',
                'output_topic': deskewed_scan,
                'output_frame': LaunchConfiguration('frame_id'),
            }],
        ),
        Node(
            package='lidar_perception', executable='scan_filter_node',
            name='scan_filter_node', output='screen',
            parameters=[LaunchConfiguration('perception_params_file'), {
                'input_topic': filter_input,
                'output_topic': filtered_scan,
                'target_frame': LaunchConfiguration('frame_id'),
            }],
        ),
        Node(
            package='lidar_perception', executable='cone_detector_node',
            name='cone_detector_node', output='screen',
            parameters=[LaunchConfiguration('perception_params_file'), {
                'scan_topic': filtered_scan,
                'output_frame': LaunchConfiguration('frame_id'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_lidar_odom')),
            package='simple_lidar_odom', executable='lidar_odom_node',
            name='simple_lidar_odom', output='screen',
            parameters=[LaunchConfiguration('lidar_odom_params_file'), {
                'scan_topic': filtered_scan,
                'publish_tf': False,
                'ekf_odom_topic': '/odom',
                'raw_odom_topic': '/odom/data_raw',
                'imu_topic': '/imu/data_raw',
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('enable_web_viewer')),
            package='lidar_web_viewer', executable='lidar_web_viewer.py',
            name='lidar_web_viewer', output='screen',
            parameters=[LaunchConfiguration('web_viewer_params_file'), {
                'scan_topic': filtered_scan,
            }],
        ),
    ])
