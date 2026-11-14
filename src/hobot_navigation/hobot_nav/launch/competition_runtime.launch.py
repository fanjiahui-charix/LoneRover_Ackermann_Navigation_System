import os
import sys
from pathlib import Path

import yaml

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_SCRIPTS_DIR = (
    Path(get_package_share_directory('hobot_nav')) / 'scripts')
if str(_SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS_DIR))

from fixed_start_utils import load_fixed_start_points  # noqa: E402


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _validate_competition(context, *args, **kwargs):
    del args, kwargs
    load_fixed_start_points(
        LaunchConfiguration('points_file').perform(context),
        require_calibrated=True,
    )

    start_mission = _as_bool(
        LaunchConfiguration('start_mission').perform(context))
    wait_for_start_signal = _as_bool(
        LaunchConfiguration('wait_for_start_signal').perform(context))
    mission_will_start = start_mission
    expected_domain = LaunchConfiguration(
        'expected_ros_domain_id').perform(context).strip()
    actual_domain = os.environ.get('ROS_DOMAIN_ID', '0').strip()
    use_qr = _as_bool(LaunchConfiguration('use_qr').perform(context))
    launch_bev_stack = _as_bool(
        LaunchConfiguration('launch_bev_stack').perform(context))
    use_cones = _as_bool(
        LaunchConfiguration('enable_cone_detector').perform(context))
    mission_scope = LaunchConfiguration('mission_scope').perform(context)
    if mission_scope not in ('full', 'qr_reverse_gate', 'channel_validation'):
        raise RuntimeError(
            'mission_scope must be full, qr_reverse_gate, or channel_validation')
    direction_override = LaunchConfiguration(
        'direction_override').perform(context).strip().lower()
    channel_v2 = _as_bool(
        LaunchConfiguration('channel_avoidance_v2_enabled').perform(context))
    channel_admitted = _as_bool(
        LaunchConfiguration('channel_templates_runtime_allowed').perform(context))
    launch_updates = []
    if _as_bool(LaunchConfiguration('competition_quiet_mode').perform(context)):
        launch_updates.extend([
            SetLaunchConfiguration('log_level', 'fatal'),
            SetLaunchConfiguration('competition_log_level', 'fatal'),
            SetLaunchConfiguration('qr_log_level', 'fatal'),
            SetLaunchConfiguration('camera_log_level', 'fatal'),
            SetLaunchConfiguration('capture_diagnostics_enabled', 'false'),
        ])
    if channel_v2:
        asset_dir = LaunchConfiguration('channel_v2_asset_directory').perform(context)
        admission_sha = LaunchConfiguration('channel_v2_admission_sha256').perform(context)
        if not channel_admitted or not asset_dir or len(admission_sha) != 64:
            raise RuntimeError(
                'Channel v2 requires explicit runtime admission, asset directory, '
                'and 64-character admission SHA256.')
        tuning_path = Path(
            LaunchConfiguration('channel_runtime_tuning_file').perform(context)
        ).expanduser()
        if not tuning_path.is_file():
            raise RuntimeError(f'Channel runtime tuning file not found: {tuning_path}')
        tuning = yaml.safe_load(tuning_path.read_text(encoding='utf-8')) or {}
        if tuning.get('schema_version') != 2:
            raise RuntimeError('Channel runtime tuning schema_version must be 2.')
        if tuning.get('speed', {}).get('straight_mode') != 'selected_profile':
            raise RuntimeError('Channel straight speed must use selected_profile.')
        if tuning.get('contracts', {}).get('hard_rmin_m') != 0.35:
            raise RuntimeError('Channel hard_rmin_m must remain 0.35 m.')
        selected_speed = float(
            LaunchConfiguration('linear_speed_limit').perform(context)
        )
        if selected_speed <= 0.0:
            selected_profile = LaunchConfiguration('speed_profile').perform(context)
            selected_speed = {
                'safe_05': 0.50, 'race_06': 0.60, 'race_07': 0.70,
                'race_08': 0.80, 'race_09': 0.90,
            }.get(selected_profile, 0.50)
        speed = tuning['speed']
        preview = tuning['corner_preview']
        avoidance = tuning['avoidance']
        values = {
            'channel_straight_ceiling': selected_speed,
            'channel_corner_ceiling': float(speed['corner_ceiling_mps']),
            'channel_lane_change_ceiling': float(speed['lane_change_ceiling_mps']),
            'channel_corner_preview_distance': float(preview['distance_m']),
            'channel_corner_exit_hold_distance': float(
                preview.get('exit_hold_distance_m', 0.0)),
            'channel_corner_curvature_threshold': float(
                preview['curvature_threshold_1pm']),
            'channel_cone_lookahead': float(avoidance['cone_trigger_lookahead_m']),
            'channel_cone_passed_margin': float(
                avoidance['cone_passed_return_margin_m']),
            'channel_cone_lateral_relevance': float(
                avoidance['cone_lateral_relevance_m']),
        }
        if any(value <= 0.0 for name, value in values.items()
               if name != 'channel_corner_exit_hold_distance'):
            raise RuntimeError('Channel runtime tuning values must be positive.')
        if values['channel_corner_exit_hold_distance'] < 0.0:
            raise RuntimeError('Channel runtime tuning values must be positive.')
        launch_updates.extend(
            SetLaunchConfiguration(name, str(value))
            for name, value in values.items()
        )
    if direction_override not in ('auto', 'clockwise', 'counterclockwise'):
        raise RuntimeError(
            'direction_override must be auto, clockwise, or counterclockwise')
    if mission_will_start and not use_qr:
        raise RuntimeError(
            'A real mission requires stage-gated QR recognition.')
    if mission_will_start and actual_domain != expected_domain:
        raise RuntimeError(
            f'Competition requires isolated ROS_DOMAIN_ID={expected_domain}; '
            f'current ROS_DOMAIN_ID={actual_domain}. Domain 230 has an active '
            '192.168.31.2 navigation graph and must not be used for vehicle motion.')
    if mission_will_start and mission_scope != 'qr_reverse_gate' and not launch_bev_stack:
        raise RuntimeError(
            'A real mission requires stage-gated BEV correction for Gate, '
            'Channel, and Return.')
    if mission_will_start and not use_cones:
        raise RuntimeError(
            'A formal mission requires cone clustering; Nav2 costmaps do not '
            'consume the full filtered scan.')
    if mission_will_start and direction_override != 'auto':
        raise RuntimeError(
            'A real mission direction must come from QR parity; '
            'overrides are diagnostic-only.')
    launch_updates.append(SetLaunchConfiguration(
        'competition_start_requested',
        'true' if mission_will_start else 'false'))
    launch_updates.append(SetLaunchConfiguration(
        'competition_initial_start',
        'false' if wait_for_start_signal else ('true' if start_mission else 'false')))
    return launch_updates


def generate_launch_description():
    start_mission = LaunchConfiguration('start_mission')
    competition_start_requested = LaunchConfiguration(
        'competition_start_requested')
    competition_initial_start = LaunchConfiguration(
        'competition_initial_start')
    start_camera = LaunchConfiguration('start_camera')
    use_qr = LaunchConfiguration('use_qr')
    use_vlm = LaunchConfiguration('use_vlm')
    use_hdmi = LaunchConfiguration('use_hdmi')
    points_file = LaunchConfiguration('points_file')
    competition_points_file = LaunchConfiguration('competition_points_file')
    route_pose_bt = LaunchConfiguration('route_pose_behavior_tree')
    through_poses_bt = LaunchConfiguration('through_poses_behavior_tree')
    image_topic = LaunchConfiguration('image_topic')
    code_topic = LaunchConfiguration('code_topic')
    vlm_trigger_topic = LaunchConfiguration('vlm_trigger_topic')
    ollama_topic = LaunchConfiguration('ollama_topic')
    display_topic = LaunchConfiguration('display_topic')
    log_level = LaunchConfiguration('log_level')
    # Keep the high-rate navigation log switch scoped away from the
    # competition coordinator.  Nested camera/QR includes may declare their
    # own log_level and must not silently change the supervisor command line.
    competition_log_level = LaunchConfiguration('competition_log_level')
    qr_log_level = LaunchConfiguration('qr_log_level')
    camera_log_level = LaunchConfiguration('camera_log_level')
    capture_diagnostics_enabled = LaunchConfiguration(
        'capture_diagnostics_enabled')

    nav_udp_profile = str(Path(
        get_package_share_directory('hobot_nav')) /
        'config' / 'nav_udp_fastdds.xml')
    nav_udp_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': nav_udp_profile,
        'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        'ROS_DISABLE_LOANED_MESSAGES': '1',
    }
    vision_shm_profile = str(
        Path(get_package_prefix('hobot_shm')) /
        'lib' / 'hobot_shm' / 'config' / 'shm_fastdds.xml')
    vision_shm_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': vision_shm_profile,
        'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        'ROS_DISABLE_LOANED_MESSAGES': '0',
    }

    competition_node = Node(
        condition=IfCondition(competition_start_requested),
        package='hobot_nav',
        executable='competition_supervisor',
        name='competition_supervisor',
        output='screen',
        emulate_tty=True,
        # Supervisor consumes global/local costmaps and must share the same
        # deterministic UDP-only navigation transport as their publishers.
        additional_env=nav_udp_env,
        arguments=['--ros-args', '--log-level', competition_log_level],
        parameters=[{
            'start_mission': ParameterValue(
                competition_initial_start, value_type=bool),
            'start_topic': '/race/start',
            'mission_scope': LaunchConfiguration('mission_scope'),
            'minimum_turning_radius': 0.35,
            'channel_templates_runtime_allowed': ParameterValue(
                LaunchConfiguration('channel_templates_runtime_allowed'),
                value_type=bool),
            'channel_avoidance_v2_enabled': ParameterValue(
                LaunchConfiguration('channel_avoidance_v2_enabled'), value_type=bool),
            'channel_v2.asset_directory': LaunchConfiguration(
                'channel_v2_asset_directory'),
            'channel_v2.admission_sha256': LaunchConfiguration(
                'channel_v2_admission_sha256'),
            'channel.speed.straight_ceiling_mps': ParameterValue(
                LaunchConfiguration('channel_straight_ceiling'), value_type=float),
            'channel.speed.corner_ceiling_mps': ParameterValue(
                LaunchConfiguration('channel_corner_ceiling'), value_type=float),
            'channel.speed.lane_change_ceiling_mps': ParameterValue(
                LaunchConfiguration('channel_lane_change_ceiling'), value_type=float),
            'channel.corner_preview_distance_m': ParameterValue(
                LaunchConfiguration('channel_corner_preview_distance'), value_type=float),
            'channel.corner_exit_hold_distance_m': ParameterValue(
                LaunchConfiguration('channel_corner_exit_hold_distance'), value_type=float),
            'channel.corner_curvature_threshold_1pm': ParameterValue(
                LaunchConfiguration('channel_corner_curvature_threshold'), value_type=float),
            'channel.cone_lookahead_m': ParameterValue(
                LaunchConfiguration('channel_cone_lookahead'), value_type=float),
            'channel.cone_passed_margin_m': ParameterValue(
                LaunchConfiguration('channel_cone_passed_margin'), value_type=float),
            'channel.cone_lateral_relevance_m': ParameterValue(
                LaunchConfiguration('channel_cone_lateral_relevance'), value_type=float),
            # Keep path-shape and detailed Direct-Gate age diagnostics out of
            # the normal competition hot path.  They are enabled only for an
            # explicitly requested diagnostic run/bag.
            'diagnostics_path_geometry_enabled': ParameterValue(
                capture_diagnostics_enabled, value_type=bool),
            # Readiness always verifies the real safety-chain output. The
            # chassis may deliberately subscribe to /cmd_vel_disabled during
            # a stationary debug run without changing this health contract.
            'safe_cmd_topic': '/cmd_vel_safe',
            'race_continue_enabled': ParameterValue(
                LaunchConfiguration('race_continue_enabled'), value_type=bool),
            'competition_quiet_mode': ParameterValue(
                LaunchConfiguration('competition_quiet_mode'), value_type=bool),
            'direct_reverse_to_channel_entry_enabled': ParameterValue(
                LaunchConfiguration('direct_reverse_to_channel_entry_enabled'),
                value_type=bool),
            'bev_correction_enabled_on_start': ParameterValue(
                LaunchConfiguration('bev_correction_enabled'), value_type=bool),
            # Formal P->QR uses only these two fixed map goals.  Their yaw is
            # derived by the supervisor from the calibrated camera TF and QR
            # board pose; the legacy lane-bias/replan path is disabled.
            'qr.primary_x': 3.60,
            'qr.primary_y': 1.20,
            'qr.backup_x': 3.60,
            'qr.backup_y': 0.80,
            'qr_midcourse_replan_x': ParameterValue(
                LaunchConfiguration('qr_midcourse_replan_x'), value_type=float),
            'qr_lane_bias_enabled': False,
            'qr_stage_timeout_sec': 30.0,
            'reverse_speed_limit': ParameterValue(
                LaunchConfiguration('reverse_speed_limit'), value_type=float),
            'base_frame': 'base_link',
            'camera_frame': LaunchConfiguration('camera_frame'),
            'map_frame': 'map',
            'vehicle.wheelbase_m': 0.144,
            'vehicle.footprint_front_m': 0.28,
            'vehicle.footprint_rear_m': 0.11,
            'vehicle.footprint_half_width_m': 0.13,
            'vehicle.footprint_padding_m': 0.0,
            'channel.heading_limit_deg': 8.0,
            'vehicle.right_mechanical_limit_deg': 22.3,
            'stationary.linear_threshold_mps': 0.02,
            'stationary.angular_threshold_radps': 0.05,
            'stationary.minimum_duration_sec': ParameterValue(
                LaunchConfiguration('stationary_minimum_duration_sec'),
                value_type=float),
            'stationary.minimum_samples': ParameterValue(
                LaunchConfiguration('stationary_minimum_samples'),
                value_type=int),
            'stationary.maximum_odom_age_sec': ParameterValue(
                LaunchConfiguration('stationary_maximum_odom_age_sec'),
                value_type=float),
            'parking_region.footprint_fraction_min': 0.50,
            'parking_region.x_min': 0.0,
            'parking_region.x_max': 1.0,
            'parking_region.y_min': 0.0,
            'parking_region.y_max': 0.4,
            'gate_region.x_min': 2.30,
            'gate_region.x_max': 2.70,
            'gate_region.y_min': 2.25,
            'gate_region.y_max': 2.40,
            'reverse.search_step_m': 0.10,
            'reverse.max_search_distance_m': 1.50,
            'snapshot_freshness.odom_warn_age_sec': 0.30,
            'snapshot_freshness.tf_warn_age_sec': 0.30,
            'snapshot_freshness.local_costmap_warn_age_sec': 0.60,
            'snapshot_freshness.global_costmap_warn_age_sec': 1.00,
            'snapshot_freshness.odom_hard_age_sec': 0.80,
            'snapshot_freshness.tf_hard_age_sec': 0.80,
            'snapshot_freshness.local_costmap_hard_age_sec': 1.50,
            'snapshot_freshness.global_costmap_hard_age_sec': 2.00,
            'action_deadlines.goal_response_hard_sec': 1.0,
            'action_deadlines.compute_path_warning_sec': 1.0,
            'action_deadlines.compute_path_hard_sec': 3.0,
            'action_deadlines.cancel_terminal_hard_sec': 1.5,
            'action_deadlines.forward_speed_limit_mps': ParameterValue(
                LaunchConfiguration('linear_speed_limit'), value_type=float),
            'qr_board.center_x': ParameterValue(
                LaunchConfiguration('qr_board_center_x'), value_type=float),
            'qr_board.center_y': ParameterValue(
                LaunchConfiguration('qr_board_center_y'), value_type=float),
            'qr_board.center_z': ParameterValue(
                LaunchConfiguration('qr_board_center_z'), value_type=float),
            'qr_board.normal_yaw_deg': ParameterValue(
                LaunchConfiguration('qr_board_normal_yaw_deg'),
                value_type=float),
            'qr_board.width_m': 0.210,
            'qr_board.height_m': 0.148,
            'qr_camera.fx': 682.7522450473028,
            'qr_camera.fy': 684.6825030858382,
            'qr_camera.cx': 948.0233927822394,
            'qr_camera.cy': 552.5465008451092,
            'qr_camera.image_width': 1920,
            'qr_camera.image_height': 1080,
            'qr_camera.minimum_projected_width_px': 70.0,
            'qr_camera.image_border_fraction': 0.08,
            'qr_camera.maximum_slant_angle_deg': 60.0,
            # The QR publisher carries the camera-frame timestamp.  X5
            # decode/DDS latency measured on-car is about 0.85 s; retain a
            # bounded freshness gate without rejecting the decoder's one-shot
            # latched result before the handover safety checks run.
        }],
        remappings=[
            ('/code', code_topic),
            ('/ollama', ollama_topic),
            ('/vlm_trigger', vlm_trigger_topic),
            ('/race/display_text', display_topic),
        ],
    )

    foxglove_node = Node(
        condition=IfCondition(LaunchConfiguration('launch_foxglove_bridge')),
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge_competition',
        output='screen',
        additional_env=nav_udp_env,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'port': 8765,
            'address': '0.0.0.0',
            'send_buffer_limit': 20000000,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_mission',
            default_value='false',
            description=(
                'Safety gate. true executes the complete competition mission; '
                'the vehicle must physically be at P first.'),
        ),
        DeclareLaunchArgument(
            'expected_ros_domain_id', default_value='231',
            description=(
                'Startup ownership guard. The shell ROS_DOMAIN_ID must match '
                'this isolated competition domain.')),
        DeclareLaunchArgument(
            'launch_foxglove_bridge', default_value='true',
            description=(
                'Own one in-domain UDP-profile Foxglove bridge on port 8765 '
                'so map/TF/odom share the competition DDS graph.')),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('use_qr', default_value='true'),
        DeclareLaunchArgument('use_vlm', default_value='true'),
        DeclareLaunchArgument(
            'mission_scope', default_value='full',
            description='full, qr_reverse_gate, or channel_validation'),
        DeclareLaunchArgument(
            'use_hdmi', default_value='false',
            description=(
                'Optional local display. Keep false for the navigation run; '
                'the mission result topics remain available without DRM.')),
        DeclareLaunchArgument('direction_override', default_value='auto'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument(
            'camera_tf_file',
            default_value='/root/calib/origincar_pro/camera_tf.yaml'),
        DeclareLaunchArgument(
            'usb_camera_calibration_file_path',
            default_value=(
                '/root/calib/origincar_pro/usb_camera_calibration.yaml')),
        DeclareLaunchArgument('qr_board_center_x', default_value='4.75'),
        DeclareLaunchArgument('qr_board_center_y', default_value='1.80'),
        DeclareLaunchArgument('qr_board_center_z', default_value='0.18'),
        DeclareLaunchArgument(
            'qr_board_normal_yaw_deg', default_value='180.0'),
        DeclareLaunchArgument(
            'qr_midcourse_replan_x', default_value='1.6',
            description=(
                'Map-frame x threshold for the one-shot motion-preserving '
                'P-to-QR Smac replan.')),
        DeclareLaunchArgument(
            'channel_templates_runtime_allowed',
            default_value='false',
            description=(
                'Fail-closed field admission switch. Enable only after the '
                'generated CW/CCW template has passed the on-car corridor, '
                'curvature, TF-version and suffix tests.')),
        DeclareLaunchArgument(
            'wait_for_start_signal', default_value='true',
            description=(
                'When start_mission=true, arm the complete stack and wait for '
                'a std_msgs/String start command on /race/start.')),
        DeclareLaunchArgument(
            'race_continue_enabled', default_value='true',
            description=(
                'Keep startup readiness strict but continue through non-fatal '
                'runtime freshness/planning/avoidance faults. Set false to '
                'restore the previous fail-closed mission behavior.')),
        DeclareLaunchArgument('channel_avoidance_v2_enabled', default_value='false'),
        DeclareLaunchArgument('channel_v2_asset_directory', default_value=''),
        DeclareLaunchArgument('channel_v2_admission_sha256', default_value=''),
        DeclareLaunchArgument(
            'channel_runtime_tuning_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'config', 'channel_tuning',
                'channel_runtime_tuning.yaml',
            ])),
        DeclareLaunchArgument('channel_straight_ceiling', default_value='0.60'),
        DeclareLaunchArgument('channel_corner_ceiling', default_value='0.42'),
        DeclareLaunchArgument('channel_lane_change_ceiling', default_value='0.42'),
        DeclareLaunchArgument('channel_corner_preview_distance', default_value='0.45'),
        DeclareLaunchArgument('channel_corner_exit_hold_distance', default_value='0.0'),
        DeclareLaunchArgument('channel_corner_curvature_threshold', default_value='0.15'),
        DeclareLaunchArgument('channel_cone_lookahead', default_value='1.25'),
        DeclareLaunchArgument('channel_cone_passed_margin', default_value='0.45'),
        DeclareLaunchArgument('channel_cone_lateral_relevance', default_value='0.27'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('competition_log_level', default_value='info'),
        DeclareLaunchArgument('qr_log_level', default_value='info'),
        DeclareLaunchArgument('camera_log_level', default_value='error'),
        DeclareLaunchArgument(
            'competition_quiet_mode', default_value='false',
            description=(
                'Race-only low-overhead mode: force node log levels to fatal, '
                'disable diagnostics, and suppress mission/action audit topics.')),
        DeclareLaunchArgument(
            'direct_reverse_to_channel_entry_enabled', default_value='true',
            description=(
                'After Reverse stops, try current-pose Smac plans directly to '
                'Channel Entry S1/S2; retain the existing Gate flow as fallback.')),
        DeclareLaunchArgument(
            'capture_diagnostics_enabled',
            default_value='false',
            description=(
                'Enable low-rate QR reverse audit topics for debug bags.')),
        DeclareLaunchArgument(
            'enable_cone_detector',
            default_value='true',
            description=(
                'Competition cone clustering. Costmaps consume only the '
                'fixed-radius cone cloud; raw scan remains available to the '
                'perception/diagnostic stack.')),
        DeclareLaunchArgument(
            'enable_lidar_odom', default_value='false',
            description='Debug-only; formal competition must keep false.'),
        DeclareLaunchArgument('base_cmd_vel_topic', default_value='/cmd_vel_safe'),
        DeclareLaunchArgument('speed_profile', default_value='safe_05'),
        DeclareLaunchArgument('linear_speed_limit', default_value='0.50'),
        DeclareLaunchArgument(
            'allow_high_speed_navigation', default_value='false'),
        DeclareLaunchArgument(
            'reverse_speed_limit', default_value='0.35'),
        DeclareLaunchArgument(
            'stationary_minimum_duration_sec', default_value='0.15'),
        DeclareLaunchArgument(
            'stationary_minimum_samples', default_value='4'),
        DeclareLaunchArgument(
            'stationary_maximum_odom_age_sec', default_value='0.10'),
        DeclareLaunchArgument('readiness_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'launch_bev_stack', default_value='true',
            description=(
                'Start the BPU BEV segmenter and matcher/backend processes. '
                'This is stack ownership only; compute and correction are '
                'separately gated by the Supervisor.')),
        DeclareLaunchArgument(
            'bev_compute_enabled', default_value='false',
            description=(
                'Initial BEV BPU inference state. Keep false through QR, '
                'Direct and Reverse; the Supervisor opens it at Gate.')),
        DeclareLaunchArgument(
            'bev_correction_enabled', default_value='false',
            description=(
                'Initial map->odom correction state. The Supervisor publishes '
                'the transient-local stage gate; this remains false at startup.')),
        DeclareLaunchArgument(
            'localization_pose_x', default_value='0.41'),
        DeclareLaunchArgument(
            'localization_pose_y', default_value='0.21'),
        DeclareLaunchArgument(
            'localization_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'bev_localization_id_map_topic',
            default_value='/bev_seg/id_map'),
        DeclareLaunchArgument(
            'bev_localization_model_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_bev_seg'),
                'models',
                'yolo26s-sem.bin',
            ])),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'maps',
                'rdk_2026_hospital_static_1cm.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'usb_video_device',
            default_value=(
                '/dev/v4l/by-id/usb-Alcorlink_Corp._USB_2.0_Camera-video-index0')),
        DeclareLaunchArgument('usb_framerate', default_value='30'),
        DeclareLaunchArgument('usb_image_width', default_value='1920'),
        DeclareLaunchArgument('usb_image_height', default_value='1080'),
        DeclareLaunchArgument('image_topic', default_value='/hbmem_img'),
        DeclareLaunchArgument('code_topic', default_value='/code'),
        DeclareLaunchArgument('code_sign_topic', default_value='/code_sign'),
        DeclareLaunchArgument('vlm_trigger_topic', default_value='/vlm_trigger'),
        DeclareLaunchArgument('ollama_topic', default_value='/ollama'),
        DeclareLaunchArgument('display_topic', default_value='/race/display_text'),
        DeclareLaunchArgument('vlm_server_ip', default_value='192.168.31.196'),
        DeclareLaunchArgument('vlm_server_port', default_value='11434'),
        DeclareLaunchArgument('vlm_model', default_value='gemma3:4b'),
        DeclareLaunchArgument(
            'vlm_prompt',
            default_value=(
                '这是一张医疗场景的图片，请描述这张图。请用中文分三行回答：'
                '第一行描述画面中有什么样的人；第二行说明人物在哪里；'
                '第三行说明人物正在做什么。不要只输出数字或类别。')),
        DeclareLaunchArgument('vlm_http_timeout_sec', default_value='12'),
        DeclareLaunchArgument(
            'points_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'waypoints',
                'fixed_start_points.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'competition_points_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'waypoints',
                'rdk_2026_competition_points.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'navigate_to_pose_behavior_tree',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'behavior_trees',
                'navigate_to_pose_fixed_start.xml',
            ]),
        ),
        DeclareLaunchArgument(
            'route_pose_behavior_tree',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'behavior_trees',
                'navigate_to_pose_competition.xml',
            ]),
        ),
        DeclareLaunchArgument(
            'through_poses_behavior_tree',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'),
                'behavior_trees',
                'navigate_through_poses_competition.xml',
            ]),
        ),

        # Validate both point files before starting hardware or vision.
        OpaqueFunction(function=_validate_competition),

        # _validate_competition snapshots the outer start request and whether
        # the supervisor should begin armed or immediately active before the
        # nested launch sets its own start_mission=false safety gate.

        # Keep the navigation/safety chain in a non-loaned environment.  The
        # large SHM profile is injected only on map_server and planner_server
        # themselves; do not put it on this parent GroupAction, otherwise it
        # would leak into lidar, EKF, cone, controller and base processes.
        GroupAction(
            scoped=True,
            actions=[
                SetEnvironmentVariable('ROS_DISABLE_LOANED_MESSAGES', '1'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(PathJoinSubstitution([
                        FindPackageShare('hobot_nav'),
                        'launch',
                        'fixed_start_navigation.launch.py',
                    ])),
                    launch_arguments={
                        'start_mission': 'false',
                        'points_file': points_file,
                        'serial_port': LaunchConfiguration('serial_port'),
                        'camera_tf_file': LaunchConfiguration('camera_tf_file'),
                        'log_level': log_level,
                        'capture_diagnostics_enabled':
                            capture_diagnostics_enabled,
                        'enable_cone_detector': LaunchConfiguration(
                            'enable_cone_detector'),
                        'enable_lidar_odom': LaunchConfiguration(
                            'enable_lidar_odom'),
                        'base_cmd_vel_topic': LaunchConfiguration(
                            'base_cmd_vel_topic'),
                        'controller_mode': 'mppi',
                        'speed_profile': LaunchConfiguration('speed_profile'),
                        'linear_speed_limit': LaunchConfiguration(
                            'linear_speed_limit'),
                        'allow_high_speed_navigation': LaunchConfiguration(
                            'allow_high_speed_navigation'),
                        'reverse_speed_limit': LaunchConfiguration(
                            'reverse_speed_limit'),
                        'allow_reverse': 'true',
                        # The C++ CompetitionSupervisor is the high-level
                        # task coordinator; do not start BT/behavior/waypoint
                        # servers in the competition action stack.
                        'minimal_action_stack': 'true',
                        'channel_tuning_v2_enabled': LaunchConfiguration(
                            'channel_avoidance_v2_enabled'),
                        'channel_runtime_tuning_file': LaunchConfiguration(
                            'channel_runtime_tuning_file'),
                        'use_bev_localizer': LaunchConfiguration(
                            'launch_bev_stack'),
                        'bev_compute_enabled': LaunchConfiguration(
                            'bev_compute_enabled'),
                        'bev_correction_enabled': LaunchConfiguration(
                            'bev_correction_enabled'),
                        'localization_pose_x': LaunchConfiguration(
                            'localization_pose_x'),
                        'localization_pose_y': LaunchConfiguration(
                            'localization_pose_y'),
                        'localization_pose_yaw': LaunchConfiguration(
                            'localization_pose_yaw'),
                        'bev_localization_id_map_topic': LaunchConfiguration(
                            'bev_localization_id_map_topic'),
                        'bev_localization_model_file': LaunchConfiguration(
                            'bev_localization_model_file'),
                        'readiness_timeout_sec': LaunchConfiguration(
                            'readiness_timeout_sec'),
                        'fixed_start_bt': LaunchConfiguration(
                            'navigate_to_pose_behavior_tree'),
                        'fixed_start_through_poses_bt': through_poses_bt,
                        'use_rviz': LaunchConfiguration('use_rviz'),
                        'map': LaunchConfiguration('map'),
                    }.items(),
                ),
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_usb_cam'),
                'launch',
                'hobot_usb_cam.launch.py',
            ])),
            condition=IfCondition(start_camera),
            launch_arguments={
                'usb_video_device': LaunchConfiguration('usb_video_device'),
                'usb_framerate': LaunchConfiguration('usb_framerate'),
                'usb_image_width': LaunchConfiguration('usb_image_width'),
                'usb_image_height': LaunchConfiguration('usb_image_height'),
                'usb_pixel_format': 'mjpeg',
                'usb_zero_copy': 'True',
                'usb_frame_id': LaunchConfiguration('camera_frame'),
                'usb_camera_calibration_file_path': LaunchConfiguration(
                    'usb_camera_calibration_file_path'),
                # The camera emits two INFO records for every frame. Keep this
                # high-rate logger isolated from the mission/Nav2 log switch.
                'log_level': camera_log_level,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hbmem_wechatcode'),
                'launch',
                'hbmem_wechatcode.launch.py',
            ])),
            condition=IfCondition(use_qr),
            launch_arguments={
                'image_topic': image_topic,
                'code_topic': code_topic,
                'code_sign_topic': LaunchConfiguration('code_sign_topic'),
                # The camera/worker stays alive independently of navigation.
                # QR publishes its first legal value independently; Supervisor
                # decides only when motion may start or be interrupted.
                'enable_topic': '',
                'enabled_on_start': 'true',
                'decode_hz': '5.0',
                # Keep QR off the controller/EKF cores while preserving the
                # single-worker/latest-only contract.
                'process_prefix': 'nice -n 8 taskset -c 4,5',
                'log_level': qr_log_level,
            }.items(),
        ),
        Node(
            package='hobot_codec',
            executable='hobot_codec_republish',
            name='competition_jpeg_shm_to_nv12_shm',
            output='screen',
            condition=IfCondition(start_camera),
            additional_env=vision_shm_env,
            parameters=[
                {'channel': 1},
                {'in_mode': 'shared_mem'},
                {'in_format': 'jpeg'},
                {'out_mode': 'shared_mem'},
                {'out_format': 'nv12'},
                {'sub_topic': image_topic},
                {'pub_topic': '/hbmem_img_nv12'},
                {'input_framerate': 30},
                {'output_framerate': -1},
                {'dump_output': False},
            ],
            arguments=['--ros-args', '--log-level', 'error'],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('board_crop'),
                'launch',
                'lipai_vlm_pipeline.launch.py',
            ])),
            condition=IfCondition(use_vlm),
            launch_arguments={
                'server_ip': LaunchConfiguration('vlm_server_ip'),
                'server_port': LaunchConfiguration('vlm_server_port'),
                'model': LaunchConfiguration('vlm_model'),
                'prompt': LaunchConfiguration('vlm_prompt'),
                'http_timeout_sec': LaunchConfiguration('vlm_http_timeout_sec'),
                'image_topic': '/hbmem_img_nv12',
                'trigger_topic': vlm_trigger_topic,
                'capture_enable_topic': '/race/vlm_detector_enabled',
                'cache_enable_topic': '/race/vlm_image_cache_enabled',
                'cache_enabled_on_start': 'false',
                'publish_tts': 'false',
            }.items(),
        ),
        Node(
            package='hobot_hdmi',
            executable='hobot_hdmi',
            name='hobot_hdmi_competition',
            output='screen',
            condition=IfCondition(use_hdmi),
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{
                'display_mode': 'text',
                'text_sub_topic_name': display_topic,
                'text_font_path':
                    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
                'text_font_size': 28,
                'display_width': 1024,
                'display_height': 600,
            }],
        ),
        competition_node,
        foxglove_node,
        RegisterEventHandler(
            condition=IfCondition(competition_start_requested),
            event_handler=OnProcessExit(
                target_action=competition_node,
                on_exit=EmitEvent(event=Shutdown(
                    reason='complete competition mission finished')),
            ),
        ),
    ])
