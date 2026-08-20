#!/usr/bin/env python3
"""Run the vehicle's native Nav2 MPPI chain against a virtual Ackermann plant.

This script is intentionally a *test harness*, not another controller.  The
controller, smoother, limiter, planner and costmap nodes are the X5 binaries;
the only replacement is the chassis/odom feedback.  All command topics are
shadow-remapped by the launch command, so this process never touches the base
driver or /cmd_vel_safe.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import pathlib
import struct
import threading
import time
from collections import deque

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.msg import Costmap, SpeedLimit
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import MarkerArray

from ackermann_shadow_plant import AckermannPlant


def yaw_quat(yaw: float):
    from geometry_msgs.msg import Quaternion
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wrap(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


class NativeReplay:
    def __init__(self, scenario: dict, output: pathlib.Path, timeout: float,
                 use_sim_time: bool, plant_v_max: float,
                 command_delay: float, tau_v: float, tau_w: float,
                 max_accel: float, max_decel: float,
                 max_w_accel: float, plant_speed_gain: float,
                 plant_steering_left_tau: float,
                 plant_steering_right_tau: float,
                 plant_steering_left_gain: float,
                 plant_steering_right_gain: float,
                 path_align_offset: int,
                 path_align_goal_threshold: float,
                 path_align_max_occupancy_ratio: float,
                 profile_speed_limit: float,
                 preview_lateral_accel: float,
                 curve_preview_distance: float,
                 terminal_speed_limit: float,
                 terminal_slowdown_distance: float,
                 command_watchdog_sec: float,
                 post_result_settle_timeout_sec: float):
        self.scenario = scenario
        self.output = output
        self.timeout = timeout
        self.plant_v_max = plant_v_max
        self.command_delay = command_delay
        self.tau_v = tau_v
        self.tau_w = tau_w
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_w_accel = max_w_accel
        self.path_align_offset = path_align_offset
        self.path_align_goal_threshold = path_align_goal_threshold
        self.path_align_max_occupancy_ratio = path_align_max_occupancy_ratio
        self.profile_speed_limit = profile_speed_limit
        self.preview_lateral_accel = preview_lateral_accel
        self.curve_preview_distance = curve_preview_distance
        self.terminal_speed_limit = terminal_speed_limit
        self.terminal_slowdown_distance = terminal_slowdown_distance
        self.command_watchdog_sec = command_watchdog_sec
        self.post_result_settle_timeout_sec = post_result_settle_timeout_sec
        # Apply use_sim_time at node construction.  Setting it after creation
        # lets the first wall-clock samples leak into TF before the bag clock
        # arrives, which causes a destructive time-source jump in Nav2.
        self.node = rclpy.create_node(
            'native_mppi_shadow_replay',
            parameter_overrides=[rclpy.parameter.Parameter(
                'use_sim_time', rclpy.Parameter.Type.BOOL, use_sim_time)])
        self.tf_pub = self.node.create_publisher(TFMessage, '/tf', 10)
        self.odom_pub = self.node.create_publisher(Odometry, '/odom', 10)
        self.speed_limit_pub = self.node.create_publisher(
            SpeedLimit, '/speed_limit', 10)
        self.cone_pub = self.node.create_publisher(
            PointCloud2, '/cones/points', qos_profile_sensor_data)
        self.cmd_sub = self.node.create_subscription(
            Twist, '/shadow_cmd_vel_safe', self.on_command, 10)
        self.raw_sub = self.node.create_subscription(
            Twist, '/shadow_cmd_vel_raw', self.on_raw, 10)
        self.nav_sub = self.node.create_subscription(
            Twist, '/cmd_vel_nav', self.on_nav, 10)
        # These are native MPPI diagnostic outputs.  They are silent in the
        # formal baseline (visualize=false).  A diagnostic-only run enables
        # visualization and sets trajectory_step=1,time_step=21 so the marker
        # array contains the start and final point of every one of the 600
        # sampled trajectories.  That is sufficient to reproduce Humble
        # 1.1.20 findPathFurthestReachedPoint exactly without altering scores.
        self.trajectories_sub = self.node.create_subscription(
            MarkerArray, '/trajectories', self.on_trajectories, 10)
        self.transformed_path_sub = self.node.create_subscription(
            Path, '/transformed_global_plan', self.on_transformed_path, 10)
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.local_costmap_sub = self.node.create_subscription(
            Costmap, '/local_costmap/costmap_raw', self.on_local_costmap,
            costmap_qos)
        self.compute = ActionClient(self.node, ComputePathToPose,
                                    '/compute_path_to_pose')
        self.follow = ActionClient(self.node, FollowPath, '/follow_path')
        self.timer = self.node.create_timer(0.02, self.tick)
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.last_cmd_stamp = None
        self.raw = (0.0, 0.0)
        self.nav = (0.0, 0.0)
        self.queue: deque[tuple[float, float, float]] = deque()
        self.last_tick = None
        self.last_sim_time = None
        fixed_cones = scenario.get('fixed_cones_map', [])
        if not isinstance(fixed_cones, list):
            raise ValueError('fixed_cones_map must be a list of [x,y] pairs')
        self.fixed_cones = []
        for index, cone in enumerate(fixed_cones):
            if (not isinstance(cone, list) or len(cone) != 2 or
                    not all(math.isfinite(float(value)) for value in cone)):
                raise ValueError(
                    f'fixed_cones_map[{index}] must be a finite [x,y] pair')
            self.fixed_cones.append((float(cone[0]), float(cone[1])))
        self.last_cone_publish_sec = None
        self.plant = AckermannPlant(
            wheelbase_m=0.144,
            min_turning_radius_m=0.35,
            speed_limit_mps=self.plant_v_max,
            tau_speed_sec=self.tau_v,
            tau_steering_sec=self.tau_w,
            max_accel_mps2=self.max_accel,
            max_decel_mps2=self.max_decel,
            legacy_max_yaw_accel_radps2=self.max_w_accel,
            speed_gain=float(scenario.get(
                'plant_speed_gain', plant_speed_gain)),
            tau_steering_left_sec=float(
                scenario.get(
                    'plant_tau_steering_left_sec', plant_steering_left_tau)),
            tau_steering_right_sec=float(
                scenario.get(
                    'plant_tau_steering_right_sec', plant_steering_right_tau)),
            steering_gain_left=float(
                scenario.get(
                    'plant_steering_gain_left', plant_steering_left_gain)),
            # R3 did not excite clean right turns enough to identify a
            # separate gain.  Keep the left-derived gain as the provisional
            # value while the asymmetric STM32 PWM table remains exact.
            steering_gain_right=float(
                scenario.get(
                    'plant_steering_gain_right', plant_steering_right_gain)))
        self.v = 0.0
        self.w = 0.0
        self.steering = 0.0
        self.x = float(scenario['start'][0])
        self.y = float(scenario['start'][1])
        self.yaw = math.radians(float(scenario['start'][2]))
        self.rows: list[dict[str, float]] = []
        self.path_align_probe_rows: list[dict] = []
        self._probe_markers: deque[tuple[float, list]] = deque(maxlen=4)
        self._probe_paths: deque[tuple[float, Path]] = deque(maxlen=4)
        self._local_costmap = None
        self.path = None
        self.speed_path = None
        self.speed_path_nearest = 0
        self.active_speed_limit = profile_speed_limit
        self.preview_max_curvature = 0.0
        self.remaining_path_distance = float('nan')
        self.result = 'not_started'
        self.action_result_snapshot = None
        self.settled_snapshot = None
        self.settle_reason = 'not_started'
        self.action_result_t = None
        self.plan_request_t = None
        self.plan_result_t = None
        self.planning_duration_sec = None
        self._done = threading.Event()

    def now_sec(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1.0e-9

    def on_command(self, msg: Twist):
        with self.lock:
            self.cmd = (float(msg.linear.x), float(msg.angular.z))
            self.last_cmd_stamp = self.now_sec()

    def on_raw(self, msg: Twist):
        with self.lock:
            self.raw = (float(msg.linear.x), float(msg.angular.z))

    def on_nav(self, msg: Twist):
        with self.lock:
            self.nav = (float(msg.linear.x), float(msg.angular.z))

    def on_local_costmap(self, msg: Costmap):
        with self.lock:
            self._local_costmap = msg

    def on_trajectories(self, msg: MarkerArray):
        now = self.now_sec()
        candidates = [
            m for m in msg.markers if m.ns == 'Candidate Trajectories']
        if not candidates:
            return
        with self.lock:
            self._probe_markers.append((now, candidates))
            self._pair_path_align_probe_locked()

    def on_transformed_path(self, msg: Path):
        now = self.now_sec()
        with self.lock:
            self._probe_paths.append((now, msg))
            self._pair_path_align_probe_locked()

    def prepare_speed_path(self):
        if self.path is None or len(self.path.poses) < 2:
            self.speed_path = None
            return
        x = [float(p.pose.position.x) for p in self.path.poses]
        y = [float(p.pose.position.y) for p in self.path.poses]
        yaw = []
        for pose_stamped in self.path.poses:
            q = pose_stamped.pose.orientation
            yaw.append(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        cumulative = [0.0] * len(x)
        for i in range(1, len(x)):
            cumulative[i] = cumulative[i - 1] + math.hypot(
                x[i] - x[i - 1], y[i] - y[i - 1])
        curvature = [0.0] * len(x)
        for i in range(1, len(x) - 1):
            ds = cumulative[i + 1] - cumulative[i - 1]
            curvature[i] = (
                abs(wrap(yaw[i + 1] - yaw[i - 1])) / ds
                if ds > 1.0e-6 else 0.0)
        if len(curvature) >= 2:
            curvature[0] = curvature[1]
            curvature[-1] = curvature[-2]
        self.speed_path = {
            'x': x, 'y': y, 'yaw': yaw, 'cumulative': cumulative,
            'curvature': curvature,
        }
        self.speed_path_nearest = 0

    def path_diagnostics(self, raw_v: float, raw_w: float) -> dict[str, float]:
        """Return the curvature-chain quantities used for stress attribution.

        These are deliberately logged from the same shadow tick as the plant
        state.  ``commanded_kappa`` is the curvature requested by the native
        controller, while ``actual_kappa`` is the curvature actually produced
        by the delayed Ackermann state.  A zero-speed command has no defined
        curvature and is recorded as zero rather than creating an artificial
        spike in the report.
        """
        path_kappa = 0.0
        heading_error = 0.0
        if self.speed_path:
            i = max(0, min(self.speed_path_nearest,
                           len(self.speed_path['x']) - 1))
            path_kappa = float(self.speed_path['curvature'][i])
            heading_error = wrap(float(self.speed_path['yaw'][i]) - self.yaw)
        commanded_kappa = raw_w / raw_v if abs(raw_v) > 1.0e-4 else 0.0
        actual_kappa = self.w / self.v if abs(self.v) > 1.0e-4 else 0.0
        return {
            'path_kappa_1pm': path_kappa,
            'commanded_kappa_1pm': commanded_kappa,
            'actual_kappa_1pm': actual_kappa,
            'effective_delta_rad': self.steering,
            'heading_error_rad': heading_error,
        }

    def publish_preview_speed_limit(self):
        limit = self.profile_speed_limit
        max_curvature = 0.0
        path = self.speed_path
        if path is not None:
            begin = max(0, self.speed_path_nearest - 4)
            end = min(len(path['x']), self.speed_path_nearest + 80)
            nearest = min(
                range(begin, end),
                key=lambda i: ((path['x'][i] - self.x) ** 2 +
                               (path['y'][i] - self.y) ** 2))
            self.speed_path_nearest = max(self.speed_path_nearest, nearest)
        if (path is not None and self.preview_lateral_accel > 0.0 and
                self.curve_preview_distance > 0.0):
            preview_end = (
                path['cumulative'][self.speed_path_nearest] +
                self.curve_preview_distance)
            for i in range(self.speed_path_nearest, len(path['x'])):
                if path['cumulative'][i] > preview_end:
                    break
                max_curvature = max(max_curvature, path['curvature'][i])
            if max_curvature > 1.0e-6:
                limit = min(
                    limit,
                    math.sqrt(self.preview_lateral_accel / max_curvature))
        remaining = float('nan')
        if path is not None:
            remaining = max(
                0.0,
                path['cumulative'][-1] -
                path['cumulative'][self.speed_path_nearest])
            if (self.terminal_speed_limit > 0.0 and
                    self.terminal_slowdown_distance > 0.0 and
                    remaining <= self.terminal_slowdown_distance):
                limit = min(limit, self.terminal_speed_limit)
        self.active_speed_limit = limit
        self.preview_max_curvature = max_curvature
        self.remaining_path_distance = remaining
        msg = SpeedLimit()
        msg.percentage = False
        msg.speed_limit = float(limit)
        self.speed_limit_pub.publish(msg)

    @staticmethod
    def _world_to_map(costmap: Costmap, x: float, y: float):
        meta = costmap.metadata
        resolution = float(meta.resolution)
        origin_x = float(meta.origin.position.x)
        origin_y = float(meta.origin.position.y)
        mx = math.floor((x - origin_x) / resolution)
        my = math.floor((y - origin_y) / resolution)
        if mx < 0 or my < 0 or mx >= int(meta.size_x) or my >= int(meta.size_y):
            return None
        return int(mx), int(my)

    def _path_align_probe(self, marker_time: float, markers: list,
                          path_time: float, path_msg: Path) -> dict:
        # With diagnostic time_step=21 and native time_steps=22, green=0 is
        # j=0 and green=21/22 is the endpoint j=-1 used by the actual critic.
        starts = [m for m in markers if float(m.color.g) < 0.01]
        endpoints = [m for m in markers if float(m.color.g) > 0.90]
        path_xy = [
            [float(p.pose.position.x), float(p.pose.position.y)]
            for p in path_msg.poses]
        endpoints_xy = [
            [float(m.pose.position.x), float(m.pose.position.y)]
            for m in endpoints]
        exact_endpoint_set = len(starts) == 600 and len(endpoints) == 600
        sampled_endpoint_set = bool(starts and endpoints)
        furthest = None
        sampled_furthest = None
        closest_initial = None
        reason = 'insufficient_native_trajectory_markers'
        invalid_count = None
        occupancy_ratio = None
        path_align_active = False
        goal_distance = None
        cm = self._local_costmap

        if path_xy:
            goal_distance = math.hypot(
                self.x - path_xy[-1][0], self.y - path_xy[-1][1])
        all_path_invalid_count = None
        if sampled_endpoint_set and path_xy:
            nearest = []
            for ex, ey in endpoints_xy:
                nearest.append(min(
                    range(len(path_xy)),
                    key=lambda i: ((path_xy[i][0] - ex) ** 2 +
                                   (path_xy[i][1] - ey) ** 2)))
            sampled_furthest = max(nearest)
            if exact_endpoint_set:
                furthest = sampled_furthest
            sx = float(starts[0].pose.position.x)
            sy = float(starts[0].pose.position.y)
            closest_initial = min(
                range(len(path_xy)),
                key=lambda i: ((path_xy[i][0] - sx) ** 2 +
                               (path_xy[i][1] - sy) ** 2))
            if cm is not None:
                all_path_invalid_count = 0
                for px, py in path_xy[:-1]:
                    cell = self._world_to_map(cm, px, py)
                    valid = False
                    if cell is not None:
                        mx, my = cell
                        raw = int(cm.data[my * int(cm.metadata.size_x) + mx])
                        valid = raw not in (253, 254, 255)
                    if not valid:
                        all_path_invalid_count += 1
            if goal_distance is not None and goal_distance < self.path_align_goal_threshold:
                reason = 'within_goal_threshold'
            elif sampled_furthest < self.path_align_offset:
                reason = (
                    'furthest_below_offset'
                    if exact_endpoint_set else
                    'sampled_furthest_below_offset_not_conclusive')
            elif cm is None:
                reason = 'local_costmap_unavailable'
            else:
                invalid_count = 0
                occupancy_ratio = 0.0
                reason = 'active'
                # This is the exact order-dependent early-return loop used by
                # navigation2 tag 1.1.20 PathAlignCritic.
                segment_range = float(sampled_furthest - closest_initial)
                for i in range(closest_initial, sampled_furthest):
                    cell = self._world_to_map(cm, *path_xy[i])
                    valid = False
                    if cell is not None:
                        mx, my = cell
                        raw = int(cm.data[my * int(cm.metadata.size_x) + mx])
                        # Local costmap track_unknown_space is false in the
                        # audited formal runtime, so 255 is invalid here.
                        valid = raw not in (253, 254, 255)
                    if not valid:
                        invalid_count += 1
                    occupancy_ratio = (
                        invalid_count / segment_range
                        if segment_range > 0.0 else 0.0)
                    if (occupancy_ratio > self.path_align_max_occupancy_ratio and
                            invalid_count > 2):
                        reason = 'occupancy_ratio_exceeded'
                        break
                if reason == 'active' and not exact_endpoint_set:
                    if all_path_invalid_count == 0:
                        # The sampled maximum is a strict lower bound on the
                        # critic's maximum over all trajectories.  If that
                        # lower bound clears the offset and every transformed
                        # path point is valid, neither remaining gate can
                        # deactivate PathAlign: activity is therefore proven.
                        reason = 'active_proven_from_sampled_lower_bound'
                    else:
                        reason = 'sampled_lower_bound_active_occupancy_not_conclusive'
                path_align_active = reason in (
                    'active', 'active_proven_from_sampled_lower_bound')

        meta = cm.metadata if cm is not None else None
        return {
            't': marker_time,
            'path_receive_dt_sec': path_time - marker_time,
            'direct_native_observation': {
                'candidate_marker_count': len(markers),
                'candidate_start_count': len(starts),
                'candidate_endpoint_count': len(endpoints),
                'candidate_endpoints_xy': endpoints_xy,
                'transformed_path_xy': path_xy,
                'local_costmap': None if meta is None else {
                    'frame': cm.header.frame_id,
                    'resolution': float(meta.resolution),
                    'size_x': int(meta.size_x), 'size_y': int(meta.size_y),
                    'origin_x': float(meta.origin.position.x),
                    'origin_y': float(meta.origin.position.y),
                },
            },
            'derived_from_navigation2_1_1_20_algorithm': {
                'exact_all_600_endpoints': exact_endpoint_set,
                'furthest_path_index': furthest,
                'sampled_furthest_path_index_lower_bound': sampled_furthest,
                'closest_initial_path_index': closest_initial,
                'offset_from_furthest': self.path_align_offset,
                'goal_distance_m': goal_distance,
                'threshold_to_consider_m': self.path_align_goal_threshold,
                'invalid_path_points': invalid_count,
                'all_transformed_path_invalid_points': all_path_invalid_count,
                'occupancy_ratio': occupancy_ratio,
                'max_path_occupancy_ratio': self.path_align_max_occupancy_ratio,
                'path_align_active': path_align_active,
                'inactive_reason': None if path_align_active else reason,
            },
        }

    def _pair_path_align_probe_locked(self):
        while self._probe_markers and self._probe_paths:
            mt, markers = self._probe_markers[0]
            nearest_j = min(
                range(len(self._probe_paths)),
                key=lambda j: abs(self._probe_paths[j][0] - mt))
            pt, path_msg = self._probe_paths[nearest_j]
            if abs(pt - mt) > 0.08:
                # Drop whichever observation is older; cross-topic DDS order
                # can differ, but both are published in one controller cycle.
                if mt < pt:
                    self._probe_markers.popleft()
                else:
                    self._probe_paths.popleft()
                continue
            self._probe_markers.popleft()
            del self._probe_paths[nearest_j]
            self.path_align_probe_rows.append(
                self._path_align_probe(mt, markers, pt, path_msg))

    def publish_feedback(self, stamp):
        sec = stamp.to_msg()
        tf = []
        map_odom = TransformStamped()
        map_odom.header.stamp = sec
        map_odom.header.frame_id = 'map'
        map_odom.child_frame_id = 'odom'
        map_odom.transform.rotation.w = 1.0
        tf.append(map_odom)
        odom_base = TransformStamped()
        odom_base.header.stamp = sec
        odom_base.header.frame_id = 'odom'
        odom_base.child_frame_id = 'base_link'
        odom_base.transform.translation.x = self.x
        odom_base.transform.translation.y = self.y
        odom_base.transform.rotation = yaw_quat(self.yaw)
        tf.append(odom_base)
        self.tf_pub.publish(TFMessage(transforms=tf))
        msg = Odometry()
        msg.header.stamp = sec
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = yaw_quat(self.yaw)
        msg.twist.twist.linear.x = self.v
        msg.twist.twist.angular.z = self.w
        self.odom_pub.publish(msg)

    def publish_fixed_cones(self, stamp, now: float):
        """Feed field-derived stationary cones through the real ConeLayer.

        The points are already in the stable map frame.  This deliberately
        exercises the installed global/local ConeLayer and its hard/soft
        radii; the replay does not draw obstacles directly into either map.
        """
        if not self.fixed_cones:
            return
        if (self.last_cone_publish_sec is not None and
                now - self.last_cone_publish_sec < 0.10):
            return
        self.last_cone_publish_sec = now
        cloud = PointCloud2()
        cloud.header.stamp = stamp.to_msg()
        cloud.header.frame_id = 'map'
        cloud.height = 1
        cloud.width = len(self.fixed_cones)
        cloud.is_bigendian = False
        cloud.is_dense = True
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name='intensity', offset=12,
                datatype=PointField.FLOAT32, count=1),
        ]
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.data = b''.join(
            struct.pack('<ffff', x, y, 0.0, 1.0)
            for x, y in self.fixed_cones)
        self.cone_pub.publish(cloud)

    def tick(self):
        stamp = self.node.get_clock().now()
        now = stamp.nanoseconds * 1.0e-9
        if now <= 0.0:
            return
        if self.last_sim_time is None:
            self.last_sim_time = now
            self.publish_feedback(stamp)
            self.publish_fixed_cones(stamp, now)
            return
        dt = now - self.last_sim_time
        self.last_sim_time = now
        if dt <= 0.0 or dt > 0.20:
            dt = 0.02
        with self.lock:
            cv, cw = self.cmd
            cmd_stamp = self.last_cmd_stamp
        safe_cmd_age = (
            now - cmd_stamp if cmd_stamp is not None else float('inf'))
        watchdog_forced_zero = (
            self.command_watchdog_sec > 0.0 and
            safe_cmd_age > self.command_watchdog_sec)
        if watchdog_forced_zero:
            cv, cw = 0.0, 0.0
        self.queue.append((now, cv, cw))
        while len(self.queue) > 2 and self.queue[1][0] <= now - self.command_delay:
            self.queue.popleft()
        dv, dw = (
            self.queue[0][1:]
            if self.queue and self.queue[0][0] <= now - self.command_delay
            else (0.0, 0.0)
        )
        # Ackermann-consistent virtual chassis.  Steering is the lateral state;
        # yaw rate is derived from v*tan(delta)/wheelbase and is never an
        # independently lagged state.
        plant_state = self.plant.step(dv, dw, dt)
        self.v = plant_state.speed_mps
        self.steering = plant_state.steering_rad
        self.w = plant_state.yaw_rate_radps
        self.x += dt * self.v * math.cos(self.yaw)
        self.y += dt * self.v * math.sin(self.yaw)
        self.yaw = wrap(self.yaw + dt * self.w)
        self.publish_feedback(stamp)
        self.publish_fixed_cones(stamp, now)
        self.publish_preview_speed_limit()
        with self.lock:
            raw_v, raw_w = self.raw
            nav_v, nav_w = self.nav
        row = {
            't': now, 'x': self.x, 'y': self.y, 'yaw_deg': math.degrees(self.yaw),
            'v': self.v, 'w': self.w,
            'steering_deg': math.degrees(self.steering),
            'steering_target_deg': math.degrees(
                plant_state.steering_target_rad),
            'steering_target_pwm': plant_state.steering_target_pwm,
            'raw_v': raw_v, 'raw_w': raw_w,
            'nav_v': nav_v, 'nav_w': nav_w, 'safe_v': cv, 'safe_w': cw,
            'safe_cmd_age_sec': safe_cmd_age,
            'watchdog_forced_zero': 1.0 if watchdog_forced_zero else 0.0,
            'action_result_observed': (
                1.0 if self.action_result_t is not None else 0.0),
            'speed_limit': self.active_speed_limit,
            'preview_max_kappa': self.preview_max_curvature,
            'remaining_path_m': self.remaining_path_distance,
        }
        row.update(self.path_diagnostics(raw_v, raw_w))
        self.rows.append(row)

    def pose_stamped(self, xyz):
        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x = float(xyz[0])
        msg.pose.position.y = float(xyz[1])
        msg.pose.orientation = yaw_quat(math.radians(float(xyz[2])))
        return msg

    def wait_server(self, client, name):
        # Downsample-factor and CPU/BPU load variants can make lifecycle
        # activation exceed 15 s even though the stack is healthy. Keep this
        # diagnostic wait outside the measured action time and long enough to
        # avoid classifying startup jitter as a planner failure.
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f'{name} action server unavailable')

    def state_snapshot(self):
        return {
            't': self.now_sec(),
            'x': self.x,
            'y': self.y,
            'yaw_deg': math.degrees(self.yaw),
            'v': self.v,
            'w': self.w,
            'steering_deg': math.degrees(self.steering),
        }

    def wait_for_settle(self):
        """Keep the real command chain and virtual chassis alive after Action.

        ControllerServer publishes a terminal zero, the velocity smoother and
        limiter propagate it, and the emulated 0.30 s chassis watchdog is the
        final fallback.  A 0.50 s sustained window avoids labelling a single
        zero-crossing during steering reversal as a settled vehicle.
        """
        deadline = time.monotonic() + self.post_result_settle_timeout_sec
        settled_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            quiet = abs(self.v) < 0.02 and abs(self.w) < 0.05
            if quiet:
                settled_since = (
                    time.monotonic() if settled_since is None else settled_since)
                if time.monotonic() - settled_since >= 0.50:
                    self.settle_reason = 'sustained_v_lt_0p02_w_lt_0p05'
                    self.settled_snapshot = self.state_snapshot()
                    return
            else:
                settled_since = None
            time.sleep(0.02)
        self.settle_reason = 'post_result_settle_timeout'
        self.settled_snapshot = self.state_snapshot()

    def run_actions(self):
        self.wait_server(self.compute, 'ComputePathToPose')
        self.wait_server(self.follow, 'FollowPath')
        # The isolated launch still brings controller/planner lifecycle nodes
        # up asynchronously.  Keep publishing virtual odom/TF while the
        # lifecycle manager activates them; an action server can exist before
        # the planner is ACTIVE and would otherwise reject the goal.
        time.sleep(20.0)
        goal = ComputePathToPose.Goal()
        goal.start = self.pose_stamped(self.scenario['start'])
        goal.goal = self.pose_stamped(self.scenario['goal'])
        goal.planner_id = 'GridBased'
        self.plan_request_t = self.now_sec()
        future = self.compute.send_goal_async(goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('ComputePathToPose goal rejected')
        result = handle.get_result_async()
        while rclpy.ok() and not result.done():
            time.sleep(0.02)
        self.plan_result_t = self.now_sec()
        self.planning_duration_sec = self.plan_result_t - self.plan_request_t
        plan_result = result.result().result
        self.path = plan_result.path
        if not self.path.poses:
            raise RuntimeError(
                'planner returned empty path: '
                f"{getattr(plan_result, 'error_msg', 'no error message')}")
        self.output.parent.mkdir(parents=True, exist_ok=True)
        path_rows = []
        for pose_stamped in self.path.poses:
            q = pose_stamped.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            path_rows.append({
                'x': pose_stamped.pose.position.x,
                'y': pose_stamped.pose.position.y,
                'yaw_deg': math.degrees(yaw),
            })
        (self.output.parent / 'native_smacc_path.json').write_text(json.dumps({
            'poses': len(self.path.poses),
            'frame': self.path.header.frame_id,
            'start': self.scenario['start'], 'goal': self.scenario['goal'],
            'fixed_cones_map': self.fixed_cones,
            'path': path_rows,
        }, indent=2))
        self.prepare_speed_path()
        # Set the native ControllerServer limit before FollowPath is accepted,
        # so MPPI samples the first control sequence under the preview cap.
        for _ in range(3):
            self.publish_preview_speed_limit()
            time.sleep(0.05)
        follow_goal = FollowPath.Goal()
        follow_goal.path = self.path
        follow_goal.controller_id = str(
            self.scenario.get('controller_id', 'FollowPath'))
        follow_goal.goal_checker_id = str(
            self.scenario.get('goal_checker_id', 'GateEntryGoalChecker'))
        future = self.follow.send_goal_async(follow_goal)
        while rclpy.ok() and not future.done():
            time.sleep(0.02)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('FollowPath goal rejected')
        result = handle.get_result_async()
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and not result.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not result.done():
            self.result = 'timeout'
            cancel = handle.cancel_goal_async()
            while not cancel.done():
                time.sleep(0.01)
        else:
            self.result = f'status_{result.result().status}'
        self.action_result_t = self.now_sec()
        self.action_result_snapshot = self.state_snapshot()
        self.wait_for_settle()
        self._done.set()

    def write(self):
        self.output.parent.mkdir(parents=True, exist_ok=True)
        with self.output.open('w', newline='') as stream:
            fields = ['t', 'x', 'y', 'yaw_deg', 'v', 'w', 'steering_deg',
                      'steering_target_deg', 'steering_target_pwm',
                      'raw_v', 'raw_w',
                      'nav_v', 'nav_w', 'safe_v', 'safe_w',
                      'safe_cmd_age_sec', 'watchdog_forced_zero',
                      'action_result_observed', 'speed_limit',
                      'preview_max_kappa', 'remaining_path_m',
                      'path_kappa_1pm', 'commanded_kappa_1pm',
                      'actual_kappa_1pm', 'effective_delta_rad',
                      'heading_error_rad']
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader(); writer.writerows(self.rows)
        summary = {
            'result': self.result,
            'scenario': self.scenario,
            'samples': len(self.rows),
            'last_pose': self.rows[-1] if self.rows else None,
            'action_result_pose': self.action_result_snapshot,
            'settled_pose': self.settled_snapshot,
            'settle_reason': self.settle_reason,
            'post_result_settle_timeout_sec':
                self.post_result_settle_timeout_sec,
            'native_path_poses': len(self.path.poses) if self.path else 0,
            'planning': {
                'request_t': self.plan_request_t,
                'result_t': self.plan_result_t,
                'duration_sec': self.planning_duration_sec,
            },
            'preview_speed_policy': {
                'profile_speed_limit_mps': self.profile_speed_limit,
                'preview_lateral_accel_mps2': self.preview_lateral_accel,
                'curve_preview_distance_m': self.curve_preview_distance,
                'terminal_speed_limit_mps': self.terminal_speed_limit,
                'terminal_slowdown_distance_m':
                    self.terminal_slowdown_distance,
                'transport': 'nav2_speed_limit_absolute',
            },
            'plant': {
                'plant_v_max': self.plant_v_max,
                'command_delay_sec': self.command_delay,
                'tau_v_sec': self.tau_v,
                'tau_w_sec': self.tau_w,
                'max_accel_mps2': self.max_accel,
                'max_decel_mps2': self.max_decel,
                'max_w_accel_radps2': self.max_w_accel,
                'speed_gain': self.plant.speed_gain,
                'steering_left_tau_sec': self.plant.tau_steering_left_sec,
                'steering_right_tau_sec': self.plant.tau_steering_right_sec,
                'steering_left_gain': self.plant.steering_gain_left,
                'steering_right_gain': self.plant.steering_gain_right,
                'right_steering_identification':
                    'provisional_left_derived_dynamics',
                'lateral_state': 'steering_angle',
                'yaw_rate_semantics':
                    'actual_w=actual_v*tan(actual_steering)/wheelbase',
                'wheelbase_m': 0.144,
                'min_turning_radius_m': 0.35,
                'command_watchdog_sec': self.command_watchdog_sec,
            },
        }
        summary_path = self.output.with_name(self.output.stem + '_summary.json')
        summary_path.write_text(json.dumps(summary, indent=2))
        if self.path_align_probe_rows:
            probe_path = self.output.with_name(
                self.output.stem + '_pathalign_probe.jsonl')
            with probe_path.open('w', encoding='utf-8') as stream:
                for row in self.path_align_probe_rows:
                    stream.write(json.dumps(row, separators=(',', ':')) + '\n')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scenario', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    ap.add_argument('--timeout', type=float, default=30.0)
    ap.add_argument('--plant-v-max', type=float, default=0.50)
    ap.add_argument('--command-delay', type=float, default=0.20225)
    ap.add_argument('--tau-v', type=float, default=0.11461)
    ap.add_argument('--tau-w', type=float, default=0.08632)
    ap.add_argument('--max-accel', type=float, default=0.39950)
    ap.add_argument('--max-decel', type=float, default=1.03675)
    ap.add_argument('--max-w-accel', type=float, default=2.0)
    ap.add_argument('--plant-speed-gain', type=float, default=1.05268)
    ap.add_argument('--plant-steering-left-tau', type=float, default=0.08632)
    ap.add_argument('--plant-steering-right-tau', type=float, default=0.08632)
    ap.add_argument('--plant-steering-left-gain', type=float, default=0.96061)
    ap.add_argument('--plant-steering-right-gain', type=float, default=0.96061)
    ap.add_argument('--path-align-offset', type=int, default=15)
    ap.add_argument('--path-align-goal-threshold', type=float, default=0.30)
    ap.add_argument('--path-align-max-occupancy-ratio', type=float, default=0.05)
    ap.add_argument('--profile-speed-limit', type=float, default=0.50)
    ap.add_argument('--preview-lateral-accel', type=float, default=0.0)
    ap.add_argument('--curve-preview-distance', type=float, default=0.65)
    ap.add_argument('--terminal-speed-limit', type=float, default=0.0)
    ap.add_argument('--terminal-slowdown-distance', type=float, default=0.0)
    ap.add_argument('--command-watchdog-sec', type=float, default=0.30)
    ap.add_argument('--post-result-settle-timeout', type=float, default=4.0)
    ap.add_argument('--wall-time', action='store_true',
                    help='use current wall time; replay bag without /clock')
    args = ap.parse_args()
    scenario = json.loads(args.scenario.read_text())
    rclpy.init()
    replay = NativeReplay(
        scenario, args.out, args.timeout, not args.wall_time,
        args.plant_v_max, args.command_delay, args.tau_v, args.tau_w,
        args.max_accel, args.max_decel, args.max_w_accel,
        args.plant_speed_gain,
        args.plant_steering_left_tau, args.plant_steering_right_tau,
        args.plant_steering_left_gain, args.plant_steering_right_gain,
        args.path_align_offset, args.path_align_goal_threshold,
        args.path_align_max_occupancy_ratio, args.profile_speed_limit,
        args.preview_lateral_accel, args.curve_preview_distance,
        args.terminal_speed_limit, args.terminal_slowdown_distance,
        args.command_watchdog_sec, args.post_result_settle_timeout)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(replay.node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    error = None
    try:
        replay.run_actions()
    except Exception as exc:  # keep diagnostics on every failed scenario
        replay.result = 'error:' + str(exc)
        error = exc
    finally:
        replay.write()
        executor.shutdown()
        replay.node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=1.0)
    if error:
        raise error


if __name__ == '__main__':
    main()
