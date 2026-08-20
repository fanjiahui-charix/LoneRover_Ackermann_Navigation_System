from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    scan_topic = LaunchConfiguration("scan_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    use_scan_deskew = LaunchConfiguration("use_scan_deskew")
    deskew_source = LaunchConfiguration("deskew_source")
    ekf_odom_topic = LaunchConfiguration("ekf_odom_topic")
    raw_odom_topic = LaunchConfiguration("raw_odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    reset_pose_on_map_reset = LaunchConfiguration("reset_pose_on_map_reset")
    publish_tf = LaunchConfiguration("publish_tf")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("simple_lidar_odom"),
                "config",
                "lidar_odom.yaml",
            ]),
            description="Parameter file for simple_lidar_odom.",
        ),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("odom_topic", default_value="/lidar_odom"),
        DeclareLaunchArgument("use_scan_deskew", default_value="true"),
        DeclareLaunchArgument("deskew_source", default_value="ekf_twist_lagged"),
        DeclareLaunchArgument("ekf_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("raw_odom_topic", default_value="/odom/data_raw"),
        DeclareLaunchArgument("imu_topic", default_value="/imu/data_raw"),
        DeclareLaunchArgument("reset_pose_on_map_reset", default_value="true"),
        DeclareLaunchArgument("publish_tf", default_value="false"),
        Node(
            package="simple_lidar_odom",
            executable="lidar_odom_node",
            name="simple_lidar_odom",
            output="screen",
            parameters=[
                params_file,
                {
                    "scan_topic": scan_topic,
                    "odom_topic": odom_topic,
                    "use_scan_deskew": use_scan_deskew,
                    "deskew_source": deskew_source,
                    "ekf_odom_topic": ekf_odom_topic,
                    "raw_odom_topic": raw_odom_topic,
                    "imu_topic": imu_topic,
                    "reset_pose_on_map_reset": reset_pose_on_map_reset,
                    "publish_tf": publish_tf,
                },
            ],
        ),
    ])
