import copy
import math
from pathlib import Path

import yaml
from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare


SPEED_PROFILES = {
    'safe_05': {
        'file': 'fixed_start_mppi_safe_05.yaml',
        'forward': 0.50,
        'reverse': 0.25,
        'turning_radius': 0.35,
    },
    'race_06': {
        'file': 'fixed_start_mppi_race_06.yaml',
        'forward': 0.60,
        'reverse': 0.25,
        'turning_radius': 0.35,
    },
    'race_07': {
        'file': 'fixed_start_mppi_race_07.yaml',
        'forward': 0.70,
        'reverse': 0.25,
        'turning_radius': 0.35,
    },
    'race_08': {
        'file': 'fixed_start_mppi_race_08.yaml',
        'forward': 0.80,
        'reverse': 0.25,
        'turning_radius': 0.35,
    },
    'race_09': {
        'file': 'fixed_start_mppi_race_09.yaml',
        'forward': 0.90,
        'reverse': 0.25,
        'turning_radius': 0.35,
    },
}

# Keep commands from the already field-tested sessions working while logging
# and generating only the canonical profile names above.
LEGACY_SPEED_PROFILE_ALIASES = {
    'baseline_05': 'safe_05',
    'safe_06': 'race_06',
}


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _validate_configuration(context, *args, **kwargs):
    del args, kwargs
    points_path = Path(LaunchConfiguration('points_file').perform(context)).expanduser()
    if not points_path.is_file():
        raise RuntimeError(f'Fixed-start points file not found: {points_path}')
    with points_path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    params = data.get('mission_points', {}).get('ros__parameters', {})
    if params.get('calibrated') is not True:
        raise RuntimeError(
            'Fixed-start points are not approved. Record P and QR_NAV, verify them, '
            'then set calibrated: true.')
    if params.get('frame_id') != 'map':
        raise RuntimeError('Fixed-start points must use frame_id: map.')
    points = params.get('points', {})
    for name in ('P', 'QR_NAV'):
        values = points.get(name)
        if not isinstance(values, list) or len(values) != 3:
            raise RuntimeError(f'Point {name} must be [x, y, yaw].')
        try:
            numeric = [float(value) for value in values]
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f'Point {name} contains an unset value.') from exc
        if not all(math.isfinite(value) for value in numeric):
            raise RuntimeError(f'Point {name} contains a non-finite value.')
    if params.get('routes', {}).get('fixed_start') != ['QR_NAV']:
        raise RuntimeError('fixed_start route must contain only QR_NAV.')

    base_cmd_vel_topic = LaunchConfiguration('base_cmd_vel_topic').perform(context)
    if base_cmd_vel_topic not in ('/cmd_vel_safe', '/cmd_vel_disabled'):
        raise RuntimeError(
            'base_cmd_vel_topic must be /cmd_vel_safe for vehicle runs or '
            '/cmd_vel_disabled for no-motion command-chain diagnostics.')

    controller_mode = LaunchConfiguration('controller_mode').perform(context)
    requested_speed_profile = LaunchConfiguration('speed_profile').perform(context)
    speed_limit_text = LaunchConfiguration('linear_speed_limit').perform(context)
    reverse_speed_limit_text = LaunchConfiguration(
        'reverse_speed_limit').perform(context)
    allow_high_speed = _as_bool(
        LaunchConfiguration('allow_high_speed_navigation').perform(context))
    allow_reverse = _as_bool(
        LaunchConfiguration('allow_reverse').perform(context))
    channel_tuning_v2 = _as_bool(
        LaunchConfiguration('channel_tuning_v2_enabled').perform(context))
    channel_tuning = None
    if channel_tuning_v2:
        tuning_path = Path(
            LaunchConfiguration('channel_runtime_tuning_file').perform(context)
        ).expanduser()
        if not tuning_path.is_file():
            raise RuntimeError(f'Channel runtime tuning file not found: {tuning_path}')
        channel_tuning = yaml.safe_load(tuning_path.read_text(encoding='utf-8')) or {}
        if channel_tuning.get('schema_version') != 2:
            raise RuntimeError('Channel runtime tuning schema_version must be 2.')
        if channel_tuning.get('speed', {}).get('straight_mode') != 'selected_profile':
            raise RuntimeError('Channel straight speed must be owned by selected_profile.')
        if channel_tuning.get('contracts', {}).get('hard_rmin_m') != 0.35:
            raise RuntimeError('Channel hard_rmin_m must remain 0.35 m.')
    if controller_mode != 'mppi':
        raise RuntimeError(
            'Stage13 fixed-start navigation supports controller_mode:=mppi only. '
            'Use a legacy RPP launch/profile explicitly for controller diagnostics.')
    valid_profile_names = {
        'auto', *SPEED_PROFILES, *LEGACY_SPEED_PROFILE_ALIASES}
    if requested_speed_profile not in valid_profile_names:
        raise RuntimeError(
            'speed_profile must be auto, safe_05, race_06, race_07, race_08, '
            'or race_09.')
    try:
        speed_limit = float(speed_limit_text)
    except ValueError as exc:
        raise RuntimeError('linear_speed_limit must be a number.') from exc
    if not math.isfinite(speed_limit):
        raise RuntimeError('linear_speed_limit must be finite.')
    exact_speeds = (0.0, 0.50, 0.60, 0.70, 0.80, 0.90, 1.00)
    if not any(abs(speed_limit - value) < 1e-6 for value in exact_speeds):
        raise RuntimeError(
            'Stage13 recording speeds must be exactly 0.50, 0.60, 0.70, '
            '0.80, 0.90, or 1.00 m/s.')
    if speed_limit > 0.70 and not allow_high_speed:
        raise RuntimeError(
            'linear_speed_limit above 0.70 m/s requires '
            'allow_high_speed_navigation:=true.')
    try:
        reverse_speed_limit = float(reverse_speed_limit_text)
    except ValueError as exc:
        raise RuntimeError('reverse_speed_limit must be a number.') from exc
    if not math.isfinite(reverse_speed_limit):
        raise RuntimeError('reverse_speed_limit must be finite.')
    supported_reverse_speeds = (0.0, 0.25, 0.35, 0.45)
    if not any(abs(reverse_speed_limit - value) < 1e-6
               for value in supported_reverse_speeds):
        raise RuntimeError(
            'reverse_speed_limit must be 0.0 (use the base profile) or one '
            'of the bounded A/B values 0.25, 0.35, 0.45 m/s.')

    for package_name in ('nav2_mppi_controller', 'hobot_nav'):
        try:
            get_package_prefix(package_name)
        except PackageNotFoundError as exc:
            raise RuntimeError(
                f'Required package {package_name} is not installed. Refusing '
                'to start hardware with an incomplete command chain.') from exc

    if requested_speed_profile == 'auto':
        if abs(speed_limit - 0.50) < 1e-6:
            selected_profile = 'safe_05'
        elif speed_limit <= 0.60:
            selected_profile = 'race_06'
        elif speed_limit <= 0.70:
            selected_profile = 'race_07'
        elif speed_limit <= 0.80:
            selected_profile = 'race_08'
        else:
            selected_profile = 'race_09'
    else:
        selected_profile = LEGACY_SPEED_PROFILE_ALIASES.get(
            requested_speed_profile, requested_speed_profile)

    if speed_limit > 0.0:
        expected_profile = (
            'safe_05' if speed_limit <= 0.50
            else 'race_06' if speed_limit <= 0.60
            else 'race_07' if speed_limit <= 0.70
            else 'race_08' if speed_limit <= 0.80
            else 'race_09'
        )
        if selected_profile != expected_profile:
            raise RuntimeError(
                f'linear_speed_limit={speed_limit:.2f} m/s must use '
                f'speed_profile:={expected_profile}; '
                f'{selected_profile} was requested.')

    selected_spec = SPEED_PROFILES[selected_profile]
    selected_file = selected_spec['file']
    profile_speed = selected_spec['forward']
    selected_speed = speed_limit if speed_limit > 0.0 else profile_speed
    reverse_speed = (
        reverse_speed_limit
        if reverse_speed_limit > 0.0
        else selected_spec['reverse']
    )
    selected_turning_radius = selected_spec['turning_radius']
    if selected_speed > 0.90 + 1e-6:
        raise RuntimeError(
            'linear_speed_limit above 0.90 m/s is outside the frozen '
            'Phase223 profile set.')

    profile_path = LaunchConfiguration('vehicle_profile_file').perform(context)
    if profile_path == 'auto':
        profile_path = str(
            Path(get_package_share_directory('hobot_nav'))
            / 'config'
            / selected_file
        )
    profile_path = Path(profile_path).expanduser()
    if not profile_path.is_file():
        raise RuntimeError(f'Fixed-start vehicle profile not found: {profile_path}')
    with profile_path.open(encoding='utf-8') as stream:
        runtime_profile = yaml.safe_load(stream) or {}
    try:
        controller_params = runtime_profile[
            'controller_server']['ros__parameters']
        follow_path = controller_params['FollowPath']
        planner = runtime_profile[
            'planner_server']['ros__parameters']['GridBased']
        smoother = runtime_profile['velocity_smoother']['ros__parameters']
        limiter = runtime_profile[
            'competition_command_limiter']['ros__parameters']
        local_costmap = runtime_profile[
            'local_costmap']['local_costmap']['ros__parameters']
        global_costmap = runtime_profile[
            'global_costmap']['global_costmap']['ros__parameters']
        coupled_values = {
            'MPPI vx_max': float(follow_path['vx_max']),
            'smoother max_velocity': float(smoother['max_velocity'][0]),
            'limiter profile_max_speed': float(limiter['profile_max_speed']),
        }
        turning_radii = {
            'Smac minimum_turning_radius': float(
                planner['minimum_turning_radius']),
            'MPPI min_turning_r': float(
                follow_path['AckermannConstraints']['min_turning_r']),
            'limiter min_turning_radius': float(
                limiter['min_turning_radius']),
        }
        footprint_paddings = {
            'local costmap footprint_padding':
                float(local_costmap['footprint_padding']),
            'global costmap footprint_padding':
                float(global_costmap['footprint_padding']),
        }
    except (KeyError, TypeError, ValueError, IndexError) as exc:
        raise RuntimeError(
            f'Incomplete Stage13 coupled profile: {profile_path}') from exc
    for label, value in coupled_values.items():
        if abs(value - profile_speed) >= 1e-6:
            raise RuntimeError(
                f'{label}={value:.3f} does not match selected profile '
                f'base speed {profile_speed:.3f}: {profile_path}')
    for label, value in turning_radii.items():
        if abs(value - selected_turning_radius) >= 1e-6:
            raise RuntimeError(
                f'{label}={value:.3f} does not match {selected_profile} '
                f'radius {selected_turning_radius:.3f}: '
                f'{profile_path}')
    for label, value in footprint_paddings.items():
        if abs(value) >= 1e-6:
            raise RuntimeError(
                f'{label}={value:.3f} does not match the formal zero '
                f'footprint padding: {profile_path}')

    # The tuned YAMLs retain speed-specific controller and braking parameters.
    # The launch-time profile table above is the single source of truth for the
    # three signed longitudinal caps applied by every command-chain component.
    follow_path['vx_max'] = selected_speed
    smoother['max_velocity'][0] = selected_speed
    smoother['velocity_timeout'] = 0.30
    limiter['profile_max_speed'] = selected_speed
    # The smoother is the only temporal command filter.  The limiter is the
    # final vehicle-constraint layer immediately before the chassis.
    limiter['input_topic'] = '/cmd_vel_nav'
    limiter['output_topic'] = '/cmd_vel_safe'
    controller_params['progress_checker'][
        'required_movement_radius'] = 0.02
    controller_params['progress_checker'][
        'movement_time_allowance'] = 120.0

    if allow_reverse:
        # Return-to-P has a valid parking region rather than a single required
        # heading.  Keep this checker scoped to Return only; QR, Gate and
        # ordinary MPPI paths retain their calibrated yaw tolerances.
        goal_checker_plugins = controller_params.setdefault(
            'goal_checker_plugins', [])
        if 'ReturnPGoalChecker' not in goal_checker_plugins:
            goal_checker_plugins.append('ReturnPGoalChecker')
        controller_params['ReturnPGoalChecker'] = {
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance': 0.20,
            'yaw_goal_tolerance': math.pi,
            'stateful': False,
        }
        # The calibrated STM32 uses L=0.144 m, T=0.162 m and clamps the
        # asymmetric wheel angles to +0.32/-0.49 rad. The entire software
        # chain uses the required 0.35 m minimum radius; physical clearance is
        # an admission item and is not hidden in a larger planner radius.
        competition_turning_radius = 0.35
        planner['minimum_turning_radius'] = competition_turning_radius
        follow_path['AckermannConstraints'][
            'min_turning_r'] = competition_turning_radius
        limiter['min_turning_radius'] = competition_turning_radius
        planner_params = runtime_profile[
            'planner_server']['ros__parameters']
        # Reverse candidate generation and safety auditing are owned by the
        # competition supervisor.  Keep one deterministic Smac plugin so the
        # planner server does not construct unused reverse search instances.
        planner_params['planner_plugins'] = ['GridBased']
        # The full competition stack measured only 7-8 Hz with the safe_05
        # batch of 900 samples while the controller declared 20 Hz. Bound the
        # competition-only workload so obstacle control has a stable 10 Hz
        # deadline instead of missing every nominal cycle.
        controller_params['controller_frequency'] = 10.0
        # The fixed Gate pose remains a Smac heading hint. Competition
        # completion is owned by the Supervisor's XY box, so the Nav2 checker
        # must not continue low-speed yaw repair after entering that box.
        controller_params['GateEntryGoalChecker'][
            'yaw_goal_tolerance'] = math.pi
        # Humble MPPI requires model_dt to be at least the controller period.
        # Keep both at 0.10 s and use 22 steps for the original ~2.2 s
        # prediction horizon.
        follow_path['model_dt'] = 0.10
        follow_path['time_steps'] = 22
        follow_path['batch_size'] = min(
            int(follow_path['batch_size']), 600)
        # Keep the ordinary course controller strictly forward. Sharing one
        # bidirectional MPPI instance with the QR escape made sharp channel
        # turns alternate between positive and negative vx instead of
        # committing to the turn. Only the dedicated QR controller may sample
        # reverse trajectories; both controllers still feed the same smoother,
        # limiter command chain.
        follow_path['vx_min'] = 0.0
        reverse_follow_path = copy.deepcopy(follow_path)
        reverse_follow_path['vx_min'] = -reverse_speed
        # Do not leave the copied +vx cap enabled here. The mission's
        # reverse-only limiter is a final safety interlock, not a trajectory
        # selector: if MPPI is allowed to prefer a forward sample, the
        # interlock correctly suppresses it and the NavigateThroughPoses goal
        # can remain active without any vehicle motion. Constrain this
        # controller at the sampler itself so every feasible command is a
        # planned stop or reverse command; the ordinary FollowPath controller
        # above remains [0, selected_speed] for the rest of the course.
        reverse_follow_path['vx_max'] = 0.0
        reverse_follow_path[
            'PathAngleCritic']['forward_preference'] = False
        reverse_follow_path['critics'] = [
            critic for critic in reverse_follow_path['critics']
            if critic != 'PreferForwardCritic'
        ]
        reverse_follow_path['PreferForwardCritic']['enabled'] = False
        legacy_channel_speed = min(selected_speed, 0.60)
        rpp_tuning = channel_tuning['rpp'] if channel_tuning_v2 else {
            'use_velocity_scaled_lookahead_dist': True,
            'lookahead_time_sec': 0.78,
            'lookahead_dist_m': 0.28,
            'min_lookahead_dist_m': 0.22,
            'max_lookahead_dist_m': 0.46,
            'regulated_linear_scaling_min_radius_m': 0.40,
            'regulated_linear_scaling_min_speed_mps': 0.15,
            'max_allowed_time_to_collision_sec': 0.80,
        }
        numeric_rpp_keys = (
            'lookahead_time_sec', 'lookahead_dist_m', 'min_lookahead_dist_m',
            'max_lookahead_dist_m', 'regulated_linear_scaling_min_radius_m',
            'regulated_linear_scaling_min_speed_mps',
            'max_allowed_time_to_collision_sec',
        )
        if not all(float(rpp_tuning[key]) > 0.0 for key in numeric_rpp_keys):
            raise RuntimeError('Channel RPP tuning values must be positive.')
        if float(rpp_tuning['min_lookahead_dist_m']) > float(
                rpp_tuning['max_lookahead_dist_m']):
            raise RuntimeError('Channel min lookahead exceeds max lookahead.')
        channel_follow_path = {
            'plugin': (
                'nav2_regulated_pure_pursuit_controller::'
                'RegulatedPurePursuitController'),
            # The six verified Tube assets are designed at Rmin=0.40 m.
            # Preserve the selected launch profile (safe_05 remains 0.50)
            # while using race06=0.60 as the initial Channel tuning ceiling.
            # V2 may expose the selected profile ceiling only after explicit
            # admission.  The legacy default remains byte-for-byte 0.60-capped.
            'desired_linear_vel': (
                selected_speed if channel_tuning_v2
                else legacy_channel_speed),
            'use_velocity_scaled_lookahead_dist': bool(
                rpp_tuning['use_velocity_scaled_lookahead_dist']),
            'lookahead_time': float(rpp_tuning['lookahead_time_sec']),
            'lookahead_dist': float(rpp_tuning['lookahead_dist_m']),
            'min_lookahead_dist': float(rpp_tuning['min_lookahead_dist_m']),
            'max_lookahead_dist': float(rpp_tuning['max_lookahead_dist_m']),
            'use_regulated_linear_velocity_scaling': True,
            'regulated_linear_scaling_min_radius': float(
                rpp_tuning['regulated_linear_scaling_min_radius_m']),
            'regulated_linear_scaling_min_speed': float(
                rpp_tuning['regulated_linear_scaling_min_speed_mps']),
            'use_cost_regulated_linear_velocity_scaling': False,
            # In the admitted race-v2 stack, cone avoidance is owned by the
            # Supervisor's Center/Side Tube selection. RPP collision-ahead
            # exceptions otherwise stop the action before a legal connector
            # can be reached on the next straight. Static Tube legality and
            # the final command/radius limiter remain active.
            'use_collision_detection': not channel_tuning_v2,
            'max_allowed_time_to_collision_up_to_carrot': float(
                rpp_tuning['max_allowed_time_to_collision_sec']),
            'allow_reversing': False,
            'use_rotate_to_heading': False,
            'transform_tolerance': 0.20,
            'max_robot_pose_search_dist': 1.0,
            'use_interpolation': True,
        }
        controller_params['controller_plugins'] = [
            'FollowPath', 'FollowPathReverse', 'FollowPathChannel']
        controller_params['FollowPathReverse'] = reverse_follow_path
        controller_params['FollowPathChannel'] = channel_follow_path
        smoother['min_velocity'][0] = -reverse_speed
        limiter['profile_max_reverse_speed'] = reverse_speed
        # Keep the speed-profile point threshold.  Lowering it to two made
        # residual returns at the padded footprint boundary look like an
        # The command limiter is deliberately the only final command layer;
        # obstacle geometry remains the responsibility of costmaps and MPPI.
        behavior_params = runtime_profile[
            'behavior_server']['ros__parameters']
        behavior_params['behavior_plugins'] = ['backup', 'wait']
        behavior_params['backup'] = {
            'plugin': 'nav2_behaviors/BackUp',
        }
    else:
        follow_path['vx_min'] = 0.0
        smoother['min_velocity'][0] = 0.0
        limiter['profile_max_reverse_speed'] = 0.0

    runtime_dir = Path('/tmp/hobot_nav_runtime')
    runtime_dir.mkdir(parents=True, exist_ok=True)
    runtime_path = runtime_dir / 'fixed_start_vehicle_profile.yaml'
    pending_path = runtime_dir / 'fixed_start_vehicle_profile.yaml.tmp'
    pending_path.write_text(
        yaml.safe_dump(runtime_profile, sort_keys=False), encoding='utf-8')
    pending_path.replace(runtime_path)

    return [
        SetLaunchConfiguration('vehicle_profile_file', str(runtime_path)),
        LogInfo(msg=(
            f'Stage13 fixed-start profile: {selected_profile} '
            f'(base={profile_speed:.2f}, forward_limit={selected_speed:.2f} m/s); '
            f'reverse_limit={reverse_speed if allow_reverse else 0.0:.2f} m/s; '
            f'min_turning_radius='
            f'{0.35 if allow_reverse else selected_turning_radius:.2f} m; '
            f'runtime profile: {runtime_path}')),
    ]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    points_file = LaunchConfiguration('points_file')
    map_yaml_file = LaunchConfiguration('map')
    serial_port = LaunchConfiguration('serial_port')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    vehicle_profile_file = LaunchConfiguration('vehicle_profile_file')
    minimal_action_stack = LaunchConfiguration('minimal_action_stack')
    fixed_start_bt = LaunchConfiguration('fixed_start_bt')
    fixed_start_through_poses_bt = LaunchConfiguration('fixed_start_through_poses_bt')
    start_mission = LaunchConfiguration('start_mission')
    use_bev_localizer = LaunchConfiguration('use_bev_localizer')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    log_level = LaunchConfiguration('log_level')
    capture_diagnostics_enabled = LaunchConfiguration(
        'capture_diagnostics_enabled')

    # Keep only map/costmap publishers and the supervisor on the deterministic
    # UDP-only navigation profile.  Other navigation, perception and camera
    # processes retain the normal hobot_shm profile.
    nav_udp_env = {
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': str(
            Path(get_package_share_directory('hobot_nav')) /
            'config' / 'nav_udp_fastdds.xml'),
        'RMW_FASTRTPS_USE_QOS_FROM_XML': '1',
        'ROS_DISABLE_LOANED_MESSAGES': '1',
    }

    fixed_tf_node = Node(
        condition=UnlessCondition(use_bev_localizer),
        package='hobot_nav',
        executable='fixed_start_tf.py',
        name='fixed_start_tf',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'points_file': points_file,
            'init_timeout_sec': ParameterValue(
                LaunchConfiguration('init_timeout_sec'), value_type=float),
            'stable_sample_count': ParameterValue(
                LaunchConfiguration('stable_sample_count'), value_type=int),
            'stable_min_duration_sec': ParameterValue(
                LaunchConfiguration('stable_min_duration_sec'), value_type=float),
        }],
    )
    bev_localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('mycar_localization'),
            'launch',
            'bev_cpu_relocalization.launch.py',
        ])),
        condition=IfCondition(use_bev_localizer),
        launch_arguments={
            'config_file': LaunchConfiguration(
                'bev_localizer_params_file'),
            'semantic_map_file': LaunchConfiguration(
                'bev_semantic_map_file'),
            'model_file': LaunchConfiguration(
                'bev_localization_model_file'),
            'vehicle_id': LaunchConfiguration('bev_vehicle_id'),
            'calibration_root': LaunchConfiguration('bev_calibration_root'),
            'bev_id_map_topic': LaunchConfiguration(
                'bev_localization_id_map_topic'),
            'initial_map_x': LaunchConfiguration('localization_pose_x'),
            'initial_map_y': LaunchConfiguration('localization_pose_y'),
            'initial_map_yaw': LaunchConfiguration(
                'localization_pose_yaw'),
            'compute_enabled_on_start': LaunchConfiguration(
                'bev_compute_enabled'),
            'compute_enable_topic': '/race/bev_compute_enabled',
            'correction_enable_topic': '/race/bev_localization_corrections_enabled',
            'corrections_enabled_on_start': LaunchConfiguration(
                'bev_correction_enabled'),
        }.items(),
    )
    goal_node = Node(
        condition=IfCondition(start_mission),
        package='hobot_nav',
        executable='fixed_start_goal.py',
        name='fixed_start_goal',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'points_file': points_file,
            'readiness_timeout_sec': ParameterValue(
                LaunchConfiguration('readiness_timeout_sec'), value_type=float),
            'navigation_timeout_sec': ParameterValue(
                LaunchConfiguration('navigation_timeout_sec'), value_type=float),
            'behavior_tree': fixed_start_bt,
        }],
    )
    competition_command_limiter_node = Node(
        package='hobot_nav',
        executable='competition_command_limiter',
        name='competition_command_limiter',
        output='screen',
        emulate_tty=True,
        prefix='nice -n -4',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[
            ParameterFile(vehicle_profile_file, allow_substs=True),
            {
                'use_sim_time': use_sim_time,
                'capture_diagnostics_enabled': ParameterValue(
                    capture_diagnostics_enabled, value_type=bool),
            },
        ],
    )

    # Fast DDS UDP transport does not reliably deliver the single transient
    # 500x500 /map sample to a global StaticLayer that joins after map_server
    # activation.  Keep map_server configured but delay its lifecycle manager
    # long enough for planner/global_costmap to create the subscription; the
    # first activation publish is then received by both local and global maps.
    #
    # Resolve substitutions in the parent launch context before constructing
    # the TimerAction.  On Humble a child action created directly in
    # TimerAction may otherwise lose the parent's launch configuration scope
    # when it fires several seconds later.
    def schedule_map_lifecycle_manager(context, *args, **kwargs):
        del args, kwargs
        raw_use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
        use_sim_time_value = _as_bool(raw_use_sim_time)
        log_level_value = LaunchConfiguration('log_level').perform(context)
        return [TimerAction(
            period=8.0,
            actions=[Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_fixed_start_map',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level_value],
                parameters=[{
                    'use_sim_time': use_sim_time_value,
                    'autostart': True,
                    'bond_timeout': 12.0,
                    'node_names': ['map_server'],
                }],
            )],
        )]

    delayed_map_lifecycle_manager = OpaqueFunction(
        function=schedule_map_lifecycle_manager)

    # Start Nav2 after the delayed map lifecycle manager has had a chance to
    # publish the static map.  Starting planner/controller at t=0 makes their
    # costmap lifecycle race the one-shot map publication on the X5 DDS
    # transport; the nodes may then remain alive but never become active.
    # Base/EKF, fixed TF, camera and QR are intentionally started before this
    # delay so perception can prewarm without being part of the Nav2 race.
    def schedule_navigation_stack(context, *args, **kwargs):
        del args, kwargs
        resolved = {
            'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
            'autostart': 'true',
            'use_composition': 'false',
            'params_file': LaunchConfiguration('nav2_params_file').perform(context),
            'vehicle_profile_file': LaunchConfiguration(
                'vehicle_profile_file').perform(context),
            'nav_to_pose_bt_xml': LaunchConfiguration(
                'fixed_start_bt').perform(context),
            'nav_through_poses_bt_xml': LaunchConfiguration(
                'fixed_start_through_poses_bt').perform(context),
            'controller_cmd_vel_topic': '/cmd_vel_nav_raw',
            'minimal_action_stack': LaunchConfiguration(
                'minimal_action_stack').perform(context),
            'log_level': LaunchConfiguration('log_level').perform(context),
        }
        return [TimerAction(
            period=12.0,
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(PathJoinSubstitution([
                    FindPackageShare('hobot_nav'), 'launch',
                    'competition_nav2_navigation.launch.py'])),
                launch_arguments=resolved.items(),
            )],
        )]

    delayed_navigation_stack = OpaqueFunction(
        function=schedule_navigation_stack)

    # Humble's LifecycleManager gives each post-transition get_state response
    # a fixed two-second window and abandons the remaining node list after one
    # timeout. Under full camera/QR/Foxglove load this can leave Controller
    # active while Planner and VelocitySmoother remain inactive forever. Start
    # a bounded, state-aware repair only after the ordinary manager has had
    # ample time to finish. It skips active nodes and exits before mission
    # runtime, so it cannot become a mid-race map or motion gate.
    def schedule_nav2_startup_guard(context, *args, **kwargs):
        del args, kwargs
        log_level_value = LaunchConfiguration('log_level').perform(context)
        return [TimerAction(
            period=35.0,
            actions=[Node(
                package='hobot_nav',
                executable='nav2_startup_guard.py',
                name='nav2_startup_guard',
                output='screen',
                additional_env=nav_udp_env,
                arguments=['--ros-args', '--log-level', log_level_value],
                parameters=[{
                    'node_names': [
                        '/controller_server',
                        '/planner_server',
                        '/velocity_smoother',
                    ],
                    'repair_timeout_sec': 45.0,
                    'request_timeout_sec': 3.0,
                    'retry_period_sec': 1.0,
                }],
            )],
        )]

    delayed_nav2_startup_guard = OpaqueFunction(
        function=schedule_nav2_startup_guard)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS1'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument(
            'capture_diagnostics_enabled',
            default_value='false',
            description=(
                'Publish low-rate QR/reverse performance diagnostics. '
                'Keep false for competition runs without a debug bag.')),
        DeclareLaunchArgument(
            'camera_tf_file',
            default_value='/root/calib/origincar_pro/camera_tf.yaml',
            description=(
                'Measured base_link->camera_link and optical TF. The launch '
                'fails closed when this vehicle calibration is absent.')),
        DeclareLaunchArgument(
            'enable_cone_detector',
            default_value='true',
            description=(
                'Keep semantic cone outputs enabled outside competition; '
                'Nav2 obstacle avoidance always uses filtered /scan.')),
        DeclareLaunchArgument(
            'enable_lidar_odom', default_value='false',
            description='Debug-only LiDAR odometry switch; competition default is false.'),
        DeclareLaunchArgument(
            'start_mission', default_value='false',
            description='Safety gate. true sends QR_NAV after all readiness checks pass.'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_bev_localizer',
            default_value='false',
            description=(
                'Replace fixed_start_tf with the final BPU BEV '
                'map->odom localizer.')),
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
            'bev_vehicle_id', default_value='origincar_pro'),
        DeclareLaunchArgument(
            'bev_calibration_root', default_value='/root/calib'),
        DeclareLaunchArgument(
            'bev_compute_enabled', default_value='false',
            description=(
                'Initial BEV inference gate. The competition supervisor opens '
                'it only at Gate; keep false during QR/Direct/Reverse.')),
        DeclareLaunchArgument(
            'bev_correction_enabled', default_value='false',
            description=(
                'Initial matcher/map->odom correction state. The competition '
                'Supervisor owns the transient-local runtime gate.')),
        DeclareLaunchArgument(
            'bev_localizer_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mycar_localization'),
                'config',
                'bev_pose_graph_localizer.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'bev_semantic_map_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mycar_localization'),
                'maps',
                'semantic_map_1000x1000.png',
            ]),
        ),
        DeclareLaunchArgument(
            'base_cmd_vel_topic',
            default_value='/cmd_vel_safe',
            description=(
                'Chassis command input. Keep /cmd_vel_safe for vehicle runs; '
                'use /cmd_vel_disabled only for no-motion command-chain diagnostics.'),
        ),
        DeclareLaunchArgument(
            'controller_mode', default_value='mppi',
            description='Stage13 fixed-start controller; only mppi is accepted.'),
        DeclareLaunchArgument(
            'speed_profile', default_value='auto',
            description=(
                'Frozen Phase223 profile: auto, safe_05, race_06, race_07, '
                'race_08, or race_09.')),
        DeclareLaunchArgument(
            'linear_speed_limit', default_value='0.0',
            description=(
                'Navigation limit: 0 uses the base profile; recording '
                'steps are 0.50 through 1.00 m/s.')),
        DeclareLaunchArgument(
            'allow_high_speed_navigation', default_value='false',
            description=(
                'Required acknowledgement when reusing race_07 above '
                '0.70 m/s; required for race_08 and race_09 as well.')),
        DeclareLaunchArgument(
            'reverse_speed_limit', default_value='0.0',
            description=(
                'Planned reverse cap; 0 uses the base profile, otherwise '
                'one of 0.25, 0.35, or 0.45 m/s.')),
        DeclareLaunchArgument(
            'allow_reverse', default_value='false',
            description=(
                'Enable the competition Reeds-Shepp planner and a strictly '
                'profile-limited reverse command through the safety chain.')),
        DeclareLaunchArgument(
            'minimal_action_stack', default_value='false',
            description=(
                'Launch only controller/planner/velocity smoother; the '
                'competition supervisor owns high-level task orchestration.')),
        DeclareLaunchArgument(
            'channel_tuning_v2_enabled', default_value='false',
            description=(
                'Candidate-only RPP speed architecture. false preserves the '
                'legacy 0.60 m/s Channel cap.')),
        DeclareLaunchArgument(
            'channel_runtime_tuning_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'config', 'channel_tuning',
                'channel_runtime_tuning.yaml',
            ])),
        DeclareLaunchArgument('init_timeout_sec', default_value='45.0'),
        DeclareLaunchArgument('stable_sample_count', default_value='30'),
        DeclareLaunchArgument('stable_min_duration_sec', default_value='0.5'),
        DeclareLaunchArgument('readiness_timeout_sec', default_value='60.0'),
        DeclareLaunchArgument('navigation_timeout_sec', default_value='300.0'),
        DeclareLaunchArgument(
            'points_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'waypoints', 'fixed_start_points.yaml']),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'maps',
                'rdk_2026_hospital_static_1cm.yaml']),
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'config', 'competition_nav2_params.yaml']),
        ),
        DeclareLaunchArgument(
            'vehicle_profile_file',
            default_value='auto',
            description='auto selects controller_mode + speed_profile; path overrides it.'),
        DeclareLaunchArgument(
            'fixed_start_bt',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'behavior_trees',
                'navigate_to_pose_fixed_start.xml']),
        ),
        DeclareLaunchArgument(
            'fixed_start_through_poses_bt',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'behavior_trees',
                'navigate_through_poses_fixed_start.xml']),
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'rviz', 'slam_nav_debug.rviz']),
        ),
        # Validation runs before any hardware or navigation process is started.
        OpaqueFunction(function=_validate_configuration),
        competition_command_limiter_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'multimodal_odom.launch.py'])),
            launch_arguments={
                'mode': 'final',
                'enable_lidar_odom': LaunchConfiguration('enable_lidar_odom'),
                'launch_base': 'true',
                'base_cmd_vel_topic': LaunchConfiguration('base_cmd_vel_topic'),
                'use_sim_time': use_sim_time,
                'serial_port': serial_port,
                'enable_cone_detector': LaunchConfiguration(
                    'enable_cone_detector'),
                'log_level': log_level,
                # A loaded X5 can delay the 50 Hz base/EKF callbacks behind
                # MPPI. Priority protects odom->base_link timestamp
                # availability; it does not alter the navigation algorithm.
                'critical_nice_level': '-5',
                'publish_static_frames': 'true',
                'vehicle_profile_file': vehicle_profile_file,
                'camera_tf_file': LaunchConfiguration('camera_tf_file'),
            }.items(),
        ),
        bev_localizer,
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            additional_env=nav_udp_env,
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': use_sim_time,
            }],
        ),
        delayed_map_lifecycle_manager,
        delayed_navigation_stack,
        delayed_nav2_startup_guard,
        fixed_tf_node,
        goal_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hobot_nav'), 'launch', 'rviz_debug.launch.py'])),
            condition=IfCondition(use_rviz),
            launch_arguments={'rviz_config': rviz_config}.items(),
        ),
        RegisterEventHandler(
            condition=UnlessCondition(use_bev_localizer),
            event_handler=OnProcessExit(
                target_action=fixed_tf_node,
                on_exit=EmitEvent(event=Shutdown(
                    reason='fixed-start TF initializer exited')),
            ),
        ),
        RegisterEventHandler(
            condition=IfCondition(start_mission),
            event_handler=OnProcessExit(
                target_action=goal_node,
                on_exit=EmitEvent(event=Shutdown(
                    reason='fixed-start single-goal task finished')),
            ),
        ),
    ])
