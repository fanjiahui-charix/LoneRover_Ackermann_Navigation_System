#!/usr/bin/env python3
"""Adaptive speed limiter for the fixed-start Ackermann navigation chain.

controller_server /cmd_vel_nav_raw
  -> adaptive_speed_limiter /cmd_vel_nav
  -> velocity_smoother /cmd_vel
  -> collision_monitor /cmd_vel_safe
  -> chassis
"""

import json
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener

from .logic import (
    apply_minimum_drive_speed,
    bounded_monotonic_closest_index,
    conservative_clearance,
    cost_to_clearance,
    goal_speed_limit,
    limited_signed_speed,
    rectangle_samples,
)


def yaw_from_quaternion(quaternion) -> float:
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


@dataclass
class Grid:
    frame: str
    receipt_monotonic: float
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: np.ndarray


class AdaptiveSpeedLimiter(Node):
    """Limit signed commands without bypassing the downstream safety chain."""

    def __init__(self) -> None:
        super().__init__('adaptive_speed_limiter')
        declare = self.declare_parameter

        self.input_topic = declare(
            'input_topic', '/cmd_vel_nav_raw').value
        self.output_topic = declare(
            'output_topic', '/cmd_vel_nav').value
        self.plan_topic = declare('plan_topic', '/plan').value
        self.costmap_topic = declare(
            'costmap_topic', '/local_costmap/costmap_raw').value
        self.scan_topic = declare('scan_topic', '/scan').value
        self.base_frame = declare('base_frame', 'base_link').value
        self.odom_frame = declare('odom_frame', 'odom').value
        self.odom_topic = declare('odom_topic', '/odom').value
        self.alignment_topic = declare(
            'alignment_topic', '/navigation/map_to_odom').value
        self.reverse_only_topic = declare(
            'reverse_only_topic', '/navigation/reverse_only').value
        self.channel_control_topic = declare(
            'channel_control_topic',
            '/navigation/channel_control_enabled').value

        self.max_speed = float(declare('profile_max_speed', 0.6).value)
        self.max_reverse_speed = float(
            declare('profile_max_reverse_speed', 0.0).value)
        self.minimum_drive_speed = float(
            declare('minimum_drive_speed', 0.0).value)
        self.lateral_accel_limit = float(
            declare('lateral_accel_limit', 0.33).value)
        self.curve_preview = float(
            declare('curve_preview_distance', 0.65).value)
        self.clearance_preview = float(
            declare('clearance_preview_distance', 0.30).value)
        self.goal_decel = float(declare('goal_decel', 0.88).value)
        self.reaction_time = float(declare('reaction_time', 0.27).value)
        self.goal_buffer = float(declare('goal_buffer', 0.065).value)
        self.goal_tolerance = float(
            declare('goal_checker_tolerance', 0.10).value)
        self.goal_creep_speed = float(
            declare('goal_creep_speed', 0.06).value)
        self.goal_creep_distance = float(
            declare('goal_creep_distance', 0.22).value)

        self.clearance_low = float(declare('clearance_low', 0.05).value)
        self.clearance_high = float(declare('clearance_high', 0.20).value)
        self.clearance_min_speed = float(
            declare('clearance_min_speed', 0.22).value)
        self.clearance_recovery_alpha = float(
            declare('clearance_recovery_alpha', 0.35).value)
        self.closest_search_forward_distance = float(
            declare('closest_search_forward_distance', 1.0).value)
        self.inflation_scale = float(
            declare('inflation_cost_scaling_factor', 17.0).value)
        self.inflation_radius = float(
            declare('inflation_radius', 0.25).value)
        self.inscribed_radius = float(
            declare('inscribed_radius', 0.11).value)

        front = float(declare('footprint_front', 0.27).value)
        rear = float(declare('footprint_rear', -0.10).value)
        half_width = float(declare('footprint_half_width', 0.12).value)
        padding = float(declare('footprint_padding', 0.01).value)
        sample_step = float(
            declare('footprint_sample_step', 0.03).value)
        self.footprint = rectangle_samples(
            front, rear, half_width, padding, sample_step)

        self.preserve_curvature = bool(
            declare('preserve_curvature', True).value)
        self.min_turning_radius = float(
            declare('min_turning_radius', 0.35).value)
        self.command_timeout = float(
            declare('command_timeout', 0.25).value)
        self.costmap_timeout = float(
            declare('costmap_timeout', 0.35).value)
        self.scan_receipt_timeout = float(
            declare('scan_receipt_timeout', 0.25).value)
        self.scan_header_timeout = float(
            declare('scan_header_timeout', 0.35).value)
        self.transform_timeout = float(
            declare('transform_timeout', 0.03).value)
        self.odom_timeout = float(
            declare('odom_timeout', 0.25).value)
        self.fail_closed = bool(declare('fail_closed', True).value)
        diagnostics_topic = declare(
            'diagnostics_topic',
            '/adaptive_speed_limiter/diagnostics',
        ).value
        active_limit_topic = declare(
            'active_limit_topic',
            '/adaptive_speed_limiter/active_limit',
        ).value
        watchdog_frequency = float(
            declare('watchdog_frequency', 20.0).value)
        self.capture_diagnostics_enabled = bool(
            declare('capture_diagnostics_enabled', False).value)

        self._validate_parameters(
            front, rear, half_width, padding, sample_step, watchdog_frequency)

        self._plan: Optional[np.ndarray] = None
        self._plan_frame = 'map'
        self._closest_index = 0
        self._filtered_clearance: Optional[float] = None
        self._grid: Optional[Grid] = None
        self._odom_pose: Optional[
            Tuple[float, float, float, float, float, float]] = None
        self._map_to_odom: Optional[
            Tuple[float, float, float]] = None
        self._last_command_stamp = 0.0
        self._last_scan_receipt = 0.0
        self._last_scan_stamp_ns: Optional[int] = None
        self._last_output_was_zero = True
        self._context_failure = 'startup'
        self._reverse_only = False
        self._channel_control_active = False
        self._perf_callback_count = 0
        self._perf_callback_total_sec = 0.0
        self._perf_callback_max_sec = 0.0
        self._perf_tf_count = 0
        self._perf_tf_total_sec = 0.0
        self._perf_tf_max_sec = 0.0
        self._perf_last_command_started = 0.0
        self._perf_loop_period_max_sec = 0.0
        self._perf_input_to_output_last_sec = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=3.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command_callback_group = MutuallyExclusiveCallbackGroup()
        self.context_callback_group = ReentrantCallbackGroup()

        costmap_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._subscriptions = [
            self.create_subscription(
                Path, self.plan_topic, self._on_plan, 10,
                callback_group=self.context_callback_group),
            self.create_subscription(
                Costmap, self.costmap_topic, self._on_costmap, costmap_qos,
                callback_group=self.context_callback_group),
            self.create_subscription(
                LaserScan, self.scan_topic, self._on_scan,
                qos_profile_sensor_data,
                callback_group=self.context_callback_group),
            self.create_subscription(
                Twist, self.input_topic, self._on_command, 10,
                callback_group=self.command_callback_group),
            self.create_subscription(
                Odometry, self.odom_topic, self._on_odom, 20,
                callback_group=self.context_callback_group),
            self.create_subscription(
                TransformStamped,
                self.alignment_topic,
                self._on_alignment,
                costmap_qos,
                callback_group=self.context_callback_group,
            ),
            self.create_subscription(
                Bool,
                self.reverse_only_topic,
                self._on_reverse_only,
                10,
                callback_group=self.context_callback_group,
            ),
            self.create_subscription(
                Bool,
                self.channel_control_topic,
                self._on_channel_control,
                10,
                callback_group=self.context_callback_group,
            ),
        ]
        self.command_publisher = self.create_publisher(
            Twist, self.output_topic, 10)
        self.diagnostics_publisher = self.create_publisher(
            Float32MultiArray, diagnostics_topic, 10)
        self.active_limit_publisher = self.create_publisher(
            String, active_limit_topic, 10)
        self.watchdog_timer = self.create_timer(
            1.0 / watchdog_frequency, self._watchdog)
        if self.capture_diagnostics_enabled:
            self.perf_trace_publisher = self.create_publisher(
                String, '/race/perf_trace', 10)
            self.reverse_safety_publisher = self.create_publisher(
                String, '/race/reverse_safety_state', 10)
            self.perf_trace_timer = self.create_timer(
                1.0,
                self._publish_perf_trace,
                callback_group=self.command_callback_group,
            )
            self.reverse_safety_timer = self.create_timer(
                0.2,
                self._publish_reverse_safety_state,
                callback_group=self.context_callback_group,
            )

    def _validate_parameters(
        self,
        front: float,
        rear: float,
        half_width: float,
        padding: float,
        sample_step: float,
        watchdog_frequency: float,
    ) -> None:
        positive = {
            'profile_max_speed': self.max_speed,
            'lateral_accel_limit': self.lateral_accel_limit,
            'goal_decel': self.goal_decel,
            'inflation_cost_scaling_factor': self.inflation_scale,
            'inflation_radius': self.inflation_radius,
            'inscribed_radius': self.inscribed_radius,
            'min_turning_radius': self.min_turning_radius,
            'command_timeout': self.command_timeout,
            'costmap_timeout': self.costmap_timeout,
            'scan_receipt_timeout': self.scan_receipt_timeout,
            'scan_header_timeout': self.scan_header_timeout,
            'transform_timeout': self.transform_timeout,
            'odom_timeout': self.odom_timeout,
            'footprint_front': front,
            'footprint_half_width': half_width,
            'footprint_sample_step': sample_step,
            'watchdog_frequency': watchdog_frequency,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        non_negative = {
            'profile_max_reverse_speed': self.max_reverse_speed,
            'minimum_drive_speed': self.minimum_drive_speed,
            'curve_preview_distance': self.curve_preview,
            'clearance_preview_distance': self.clearance_preview,
            'closest_search_forward_distance':
                self.closest_search_forward_distance,
            'reaction_time': self.reaction_time,
            'goal_buffer': self.goal_buffer,
            'goal_checker_tolerance': self.goal_tolerance,
            'goal_creep_speed': self.goal_creep_speed,
            'goal_creep_distance': self.goal_creep_distance,
            'clearance_low': self.clearance_low,
            'clearance_min_speed': self.clearance_min_speed,
            'footprint_padding': padding,
        }
        for name, value in non_negative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f'{name} must be finite and non-negative')
        if not math.isfinite(rear) or rear >= front:
            raise ValueError('footprint_rear must be finite and behind footprint_front')
        if (
            not math.isfinite(self.clearance_high)
            or self.clearance_high <= self.clearance_low
        ):
            raise ValueError('clearance_high must be greater than clearance_low')
        if self.clearance_min_speed > self.max_speed:
            raise ValueError(
                'clearance_min_speed must not exceed profile_max_speed')
        if self.max_reverse_speed > self.max_speed:
            raise ValueError(
                'profile_max_reverse_speed must not exceed profile_max_speed')
        if self.minimum_drive_speed > self.max_speed:
            raise ValueError(
                'minimum_drive_speed must not exceed profile_max_speed')
        if not math.isclose(self.min_turning_radius, 0.35, abs_tol=1e-9):
            raise ValueError('min_turning_radius must be exactly 0.35 m')
        if not math.isclose(self.minimum_drive_speed, 0.0, abs_tol=1e-9):
            raise ValueError('minimum_drive_speed uplift must be zero')
        if (
            not math.isfinite(self.clearance_recovery_alpha)
            or not 0.0 < self.clearance_recovery_alpha <= 1.0
        ):
            raise ValueError(
                'clearance_recovery_alpha must be in the range (0, 1]')

    def _on_plan(self, message: Path) -> None:
        if len(message.poses) < 2:
            self._plan = None
            self._closest_index = 0
            return
        self._plan_frame = message.header.frame_id or 'map'
        self._plan = np.asarray(
            [
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    yaw_from_quaternion(pose.pose.orientation),
                )
                for pose in message.poses
            ],
            dtype=np.float64,
        )
        self._closest_index = 0

    def _on_costmap(self, message: Costmap) -> None:
        metadata = message.metadata
        width = int(metadata.size_x)
        height = int(metadata.size_y)
        data = np.asarray(message.data, dtype=np.uint8)
        resolution = float(metadata.resolution)
        if (
            width <= 0
            or height <= 0
            or not math.isfinite(resolution)
            or resolution <= 0.0
            or data.size != width * height
        ):
            self.get_logger().warning('Rejected malformed local costmap')
            return
        self._grid = Grid(
            frame=message.header.frame_id or 'odom',
            receipt_monotonic=time.monotonic(),
            resolution=resolution,
            origin_x=float(metadata.origin.position.x),
            origin_y=float(metadata.origin.position.y),
            width=width,
            height=height,
            data=data.reshape(height, width),
        )

    def _on_command(self, message: Twist) -> None:
        started = time.monotonic()
        if self.capture_diagnostics_enabled:
            if self._perf_last_command_started > 0.0:
                self._perf_loop_period_max_sec = max(
                    self._perf_loop_period_max_sec,
                    started - self._perf_last_command_started,
                )
            self._perf_last_command_started = started
        self._last_command_stamp = started
        try:
            if self._channel_control_active:
                return
            self._publish_limited(message)
        finally:
            if self.capture_diagnostics_enabled:
                elapsed = time.monotonic() - started
                self._perf_callback_count += 1
                self._perf_callback_total_sec += elapsed
                self._perf_callback_max_sec = max(
                    self._perf_callback_max_sec, elapsed)
                self._perf_input_to_output_last_sec = elapsed

    def _on_scan(self, message: LaserScan) -> None:
        stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp_ns <= 0:
            return
        self._last_scan_stamp_ns = stamp_ns
        self._last_scan_receipt = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        if (
            message.header.frame_id != self.odom_frame
            or message.child_frame_id != self.base_frame
        ):
            return
        pose = message.pose.pose
        self._odom_pose = (
            float(pose.position.x),
            float(pose.position.y),
            yaw_from_quaternion(pose.orientation),
            time.monotonic(),
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
        )

    def _on_alignment(self, message: TransformStamped) -> None:
        if (
            message.header.frame_id != self._plan_frame
            or message.child_frame_id != self.odom_frame
        ):
            return
        transform = message.transform
        self._map_to_odom = (
            float(transform.translation.x),
            float(transform.translation.y),
            yaw_from_quaternion(transform.rotation),
        )

    def _on_reverse_only(self, message: Bool) -> None:
        self._reverse_only = bool(message.data)
        if self._reverse_only:
            self.get_logger().warning(
                'Reverse-only interlock is active')
        else:
            self.get_logger().info(
                'Reverse-only interlock is released')

    def _on_channel_control(self, message: Bool) -> None:
        self._channel_control_active = bool(message.data)
        if self._channel_control_active:
            self.get_logger().warning(
                'External channel controller owns /cmd_vel_nav; suppressing Nav2 limiter output')
        else:
            self._last_output_was_zero = True
            self.get_logger().info(
                'External channel controller released /cmd_vel_nav; Nav2 limiter resumed')

    def _publish_zero(self, reason: str) -> None:
        self.command_publisher.publish(Twist())
        self._last_output_was_zero = True
        status = String()
        status.data = reason
        self.active_limit_publisher.publish(status)

    def _watchdog(self) -> None:
        if self._channel_control_active:
            return
        scan_failure = self._scan_failure_reason()
        if scan_failure is not None:
            self._context_failure = scan_failure
            if not self._last_output_was_zero:
                self._publish_zero(f'fail_closed_{scan_failure}')
            return
        if (
            time.monotonic() - self._last_command_stamp
            <= self.command_timeout
        ):
            return
        if not self._last_output_was_zero:
            self._publish_zero('command_timeout')

    def _scan_failure_reason(self) -> Optional[str]:
        if (
            self._last_scan_receipt <= 0.0
            or self._last_scan_stamp_ns is None
        ):
            return 'missing_scan'
        receipt_age = time.monotonic() - self._last_scan_receipt
        if receipt_age > self.scan_receipt_timeout:
            return (
                f'stale_scan_receipt:{receipt_age:.3f}s>'
                f'{self.scan_receipt_timeout:.3f}s'
            )
        header_age = (
            self.get_clock().now().nanoseconds
            - self._last_scan_stamp_ns
        ) / 1e9
        if header_age < -0.05:
            return f'future_scan_stamp:{header_age:.3f}s'
        if header_age > self.scan_header_timeout:
            return (
                f'stale_scan_header:{header_age:.3f}s>'
                f'{self.scan_header_timeout:.3f}s'
            )
        return None

    def _lookup_xy_yaw(
        self, target: str, source: str
    ) -> Optional[Tuple[float, float, float]]:
        started = (
            time.monotonic()
            if self.capture_diagnostics_enabled
            else None
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                target,
                source,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException:
            return None
        finally:
            if started is not None:
                elapsed = time.monotonic() - started
                self._perf_tf_count += 1
                self._perf_tf_total_sec += elapsed
                self._perf_tf_max_sec = max(
                    self._perf_tf_max_sec, elapsed)
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            yaw_from_quaternion(transform.transform.rotation),
        )

    def _publish_perf_trace(self) -> None:
        callback_mean = (
            self._perf_callback_total_sec / self._perf_callback_count
            if self._perf_callback_count
            else None
        )
        tf_mean = (
            self._perf_tf_total_sec / self._perf_tf_count
            if self._perf_tf_count
            else None
        )
        now = time.monotonic()
        longest_no_response = self._perf_loop_period_max_sec
        if self._perf_last_command_started > 0.0:
            longest_no_response = max(
                longest_no_response,
                now - self._perf_last_command_started,
            )
        payload = {
            'node': 'adaptive_speed_limiter',
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'steady_time_ns': time.monotonic_ns(),
            'callback_count': self._perf_callback_count,
            'callback_mean_sec': callback_mean,
            'callback_max_sec': (
                self._perf_callback_max_sec
                if self._perf_callback_count
                else None
            ),
            'input_message_age_sec': None,
            'input_age_reason': 'geometry_msgs/Twist has no header stamp',
            'input_to_output_delay_sec':
                self._perf_input_to_output_last_sec,
            'tf_lookup_count': self._perf_tf_count,
            'tf_lookup_mean_sec': tf_mean,
            'tf_lookup_max_sec': (
                self._perf_tf_max_sec if self._perf_tf_count else None),
            'action_wait_mean_sec': None,
            'action_wait_max_sec': None,
            'main_loop_period_max_sec': (
                self._perf_loop_period_max_sec
                if self._perf_last_command_started > 0.0
                else None
            ),
            'longest_no_response_sec': (
                longest_no_response
                if self._perf_last_command_started > 0.0
                else None
            ),
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))
        self.perf_trace_publisher.publish(message)

    def _reverse_safety_snapshot(self):
        grid = self._grid
        odom_pose = self._odom_pose
        if grid is None:
            return None, None, 'missing_costmap'
        if time.monotonic() - grid.receipt_monotonic > self.costmap_timeout:
            return None, None, 'stale_costmap'
        if odom_pose is None:
            return None, None, 'missing_odom'
        if time.monotonic() - odom_pose[3] > self.odom_timeout:
            return None, None, 'stale_odom'

        map_pose = self._aligned_robot_transform()
        if map_pose is None:
            map_pose = self._lookup_xy_yaw(
                self._plan_frame, self.base_frame)

        if grid.frame == self.odom_frame:
            grid_pose = odom_pose[:3]
        elif grid.frame == self._plan_frame and map_pose is not None:
            grid_pose = map_pose
        else:
            grid_pose = self._lookup_xy_yaw(
                grid.frame, self.base_frame)
        if grid_pose is None:
            return map_pose, None, f'missing_tf:{grid.frame}<-{self.base_frame}'
        clearance = self._footprint_clearance(
            float(grid_pose[0]),
            float(grid_pose[1]),
            float(grid_pose[2]),
            grid,
        )
        return map_pose, clearance, ''

    def _publish_reverse_safety_state(self) -> None:
        if not self._reverse_only:
            return
        map_pose, clearance, reason = self._reverse_safety_snapshot()
        odom_pose = self._odom_pose
        payload = {
            'source': 'adaptive_speed_limiter',
            'ros_time_ns': self.get_clock().now().nanoseconds,
            'steady_time_ns': time.monotonic_ns(),
            'map_x': map_pose[0] if map_pose is not None else None,
            'map_y': map_pose[1] if map_pose is not None else None,
            'map_yaw': map_pose[2] if map_pose is not None else None,
            'odom_vx': odom_pose[4] if odom_pose is not None else None,
            'odom_wz': odom_pose[5] if odom_pose is not None else None,
            'footprint_safe': (
                bool(clearance > 0.0) if clearance is not None else None),
            'footprint_clearance_m': clearance,
            'unavailable_reason': reason or None,
        }
        message = String()
        message.data = json.dumps(
            payload, ensure_ascii=False, separators=(',', ':'))
        self.reverse_safety_publisher.publish(message)

    @staticmethod
    def _transform_points(
        points: np.ndarray,
        transform: Tuple[float, float, float],
    ) -> np.ndarray:
        tx, ty, yaw = transform
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        output = points.copy()
        output[:, 0] = (
            tx + cosine * points[:, 0] - sine * points[:, 1])
        output[:, 1] = (
            ty + sine * points[:, 0] + cosine * points[:, 1])
        output[:, 2] = points[:, 2] + yaw
        return output

    @staticmethod
    def _compose_xy_yaw(
        parent_to_middle: Tuple[float, float, float],
        middle_to_child: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        px, py, parent_yaw = parent_to_middle
        mx, my, middle_yaw = middle_to_child
        cosine = math.cos(parent_yaw)
        sine = math.sin(parent_yaw)
        return (
            px + cosine * mx - sine * my,
            py + sine * mx + cosine * my,
            parent_yaw + middle_yaw,
        )

    @staticmethod
    def _inverse_xy_yaw(
        transform: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        tx, ty, yaw = transform
        inverse_yaw = -yaw
        cosine = math.cos(inverse_yaw)
        sine = math.sin(inverse_yaw)
        return (
            -(cosine * tx - sine * ty),
            -(sine * tx + cosine * ty),
            inverse_yaw,
        )

    def _aligned_robot_transform(
        self,
    ) -> Optional[Tuple[float, float, float]]:
        odom_pose = self._odom_pose
        map_to_odom = self._map_to_odom
        if odom_pose is None or map_to_odom is None:
            return None
        if time.monotonic() - odom_pose[3] > self.odom_timeout:
            return None
        return self._compose_xy_yaw(
            map_to_odom, odom_pose[:3])

    def _footprint_clearance(
        self, x: float, y: float, yaw: float, grid: Grid
    ) -> float:
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        world_x = (
            x
            + cosine * self.footprint[:, 0]
            - sine * self.footprint[:, 1]
        )
        world_y = (
            y
            + sine * self.footprint[:, 0]
            + cosine * self.footprint[:, 1]
        )
        grid_x = np.floor(
            (world_x - grid.origin_x) / grid.resolution
        ).astype(np.int64)
        grid_y = np.floor(
            (world_y - grid.origin_y) / grid.resolution
        ).astype(np.int64)
        valid = (
            (grid_x >= 0)
            & (grid_y >= 0)
            & (grid_x < grid.width)
            & (grid_y < grid.height)
        )
        if not np.all(valid):
            return 0.0
        maximum_cost = int(np.max(grid.data[grid_y, grid_x]))
        return cost_to_clearance(
            maximum_cost,
            self.inflation_scale,
            self.inscribed_radius,
            self.inflation_radius,
        )

    def _limits(
        self,
    ) -> Optional[
        Tuple[float, float, float, float, float, float, float]
    ]:
        scan_failure = self._scan_failure_reason()
        if scan_failure is not None:
            self._context_failure = scan_failure
            return None
        plan = self._plan
        grid = self._grid
        if plan is None or len(plan) < 3:
            self._context_failure = 'missing_plan'
            return None
        if grid is None:
            self._context_failure = 'missing_costmap'
            return None
        costmap_age = time.monotonic() - grid.receipt_monotonic
        if costmap_age > self.costmap_timeout:
            self._context_failure = (
                f'stale_costmap:{costmap_age:.3f}s>'
                f'{self.costmap_timeout:.3f}s')
            return None

        robot_transform = self._aligned_robot_transform()
        if robot_transform is None:
            robot_transform = self._lookup_xy_yaw(
                self._plan_frame, self.base_frame)
        if robot_transform is None:
            plan_to_odom = self._lookup_xy_yaw(
                self._plan_frame, self.odom_frame)
            odom_to_base = self._lookup_xy_yaw(
                self.odom_frame, self.base_frame)
            if plan_to_odom is None or odom_to_base is None:
                missing = []
                if plan_to_odom is None:
                    missing.append(
                        f'{self._plan_frame}<-{self.odom_frame}')
                if odom_to_base is None:
                    missing.append(
                        f'{self.odom_frame}<-{self.base_frame}')
                self._context_failure = (
                    'missing_tf_segments:' + ','.join(missing))
                return None
            robot_transform = self._compose_xy_yaw(
                plan_to_odom, odom_to_base)
        robot_xy = np.asarray(robot_transform[:2])
        closest_index = bounded_monotonic_closest_index(
            plan[:, :2],
            robot_xy,
            self._closest_index,
            self.closest_search_forward_distance,
        )
        self._closest_index = closest_index
        remaining_plan = plan[closest_index:]
        if len(remaining_plan) < 2:
            return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        segment_lengths = np.linalg.norm(
            np.diff(remaining_plan[:, :2], axis=0), axis=1)
        cumulative_distance = np.r_[0.0, np.cumsum(segment_lengths)]

        curve_count = min(
            len(remaining_plan),
            max(
                3,
                int(np.searchsorted(
                    cumulative_distance, self.curve_preview)) + 1,
            ),
        )
        curve_segments = np.diff(
            remaining_plan[:curve_count, :2], axis=0)
        geometry_yaw = np.arctan2(
            curve_segments[:, 1], curve_segments[:, 0])
        if len(geometry_yaw) > 1:
            yaw_changes = (
                np.diff(geometry_yaw) + math.pi
            ) % (2.0 * math.pi) - math.pi
            curvature = np.abs(
                yaw_changes
                / np.maximum(
                    segment_lengths[1:curve_count - 1], 1e-4)
            )
            curvature_p90 = (
                float(np.percentile(curvature, 90))
                if len(curvature)
                else 0.0
            )
        else:
            curvature_p90 = 0.0
        curve_limit = min(
            self.max_speed,
            math.sqrt(
                self.lateral_accel_limit
                / max(curvature_p90, 1e-3)
            ),
        )

        remaining_distance = max(
            0.0, float(cumulative_distance[-1]))
        goal_limit = min(
            self.max_speed,
            goal_speed_limit(
                remaining_distance,
                self.goal_decel,
                self.reaction_time,
                self.goal_buffer,
                self.goal_tolerance,
                self.goal_creep_speed,
                self.goal_creep_distance,
            ),
        )

        clearance_count = min(
            len(remaining_plan),
            max(
                2,
                int(np.searchsorted(
                    cumulative_distance,
                    self.clearance_preview,
                )) + 1,
            ),
        )
        plan_to_grid = None
        if (
            grid.frame == self.odom_frame
            and self._plan_frame == 'map'
            and self._map_to_odom is not None
        ):
            plan_to_grid = self._inverse_xy_yaw(
                self._map_to_odom)
        if plan_to_grid is None:
            plan_to_grid = self._lookup_xy_yaw(
                grid.frame, self._plan_frame)
        if plan_to_grid is None:
            self._context_failure = (
                f'missing_tf:{grid.frame}<-{self._plan_frame}')
            return None
        preview = self._transform_points(
            remaining_plan[:clearance_count], plan_to_grid)
        clearances = [
            self._footprint_clearance(
                float(pose[0]),
                float(pose[1]),
                float(pose[2]),
                grid,
            )
            for pose in preview
        ]
        observed_clearance = (
            float(min(clearances)) if clearances else 0.0
        )
        filtered_clearance = conservative_clearance(
            self._filtered_clearance,
            observed_clearance,
            self.clearance_recovery_alpha,
        )
        self._filtered_clearance = filtered_clearance
        self._context_failure = ''
        clearance_fraction = max(
            0.0,
            min(
                1.0,
                (filtered_clearance - self.clearance_low)
                / (self.clearance_high - self.clearance_low),
            ),
        )
        clearance_limit = (
            self.clearance_min_speed
            + clearance_fraction
            * (self.max_speed - self.clearance_min_speed)
        )
        return (
            curve_limit,
            goal_limit,
            min(self.max_speed, clearance_limit),
            remaining_distance,
            curvature_p90,
            filtered_clearance,
            observed_clearance,
        )

    def _publish_limited(self, command: Twist) -> None:
        if self._channel_control_active:
            return
        raw_input_speed = float(command.linear.x)
        if self._reverse_only and raw_input_speed > 1e-4:
            self._publish_zero('reverse_only_forward_command_blocked')
            return
        if (
            self._reverse_only
            and raw_input_speed < -1e-4
            and abs(float(command.angular.z)) <= 1e-4
        ):
            failure = self._backup_context_failure()
            if failure is not None:
                self._context_failure = failure
                self._publish_zero(f'fail_closed_{failure}')
                return
            output = Twist()
            output.linear.x = -min(
                abs(raw_input_speed), self.max_reverse_speed)
            self.command_publisher.publish(output)
            self._last_output_was_zero = False
            active = String()
            active.data = 'reverse_only_collision_checked_backup'
            self.active_limit_publisher.publish(active)
            return

        limits = self._limits()
        if limits is None:
            if self.fail_closed:
                self._publish_zero(
                    'fail_closed_missing_context:'
                    f'{self._context_failure or "unknown"}')
            else:
                self.command_publisher.publish(command)
                self._last_output_was_zero = (
                    abs(float(command.linear.x)) < 1e-6
                    and abs(float(command.angular.z)) < 1e-6
                )
            return

        (
            curve_limit,
            goal_limit,
            clearance_limit,
            remaining_distance,
            curvature_p90,
            filtered_clearance,
            observed_clearance,
        ) = limits
        profile_limit = (
            self.max_reverse_speed
            if raw_input_speed < 0.0
            else self.max_speed
        )
        named_limits = {
            'profile': profile_limit,
            'curve': curve_limit,
            'goal': goal_limit,
            'clearance': clearance_limit,
        }
        active_limit = min(named_limits, key=named_limits.get)
        speed_limit = named_limits[active_limit]

        input_speed = raw_input_speed
        output_speed = limited_signed_speed(
            input_speed,
            forward_limit=speed_limit,
            reverse_limit=speed_limit,
        )
        output_speed = apply_minimum_drive_speed(
            input_speed,
            output_speed,
            speed_limit,
            self.minimum_drive_speed,
        )
        output = Twist()
        output.linear.x = output_speed
        if abs(input_speed) <= 1e-4 or abs(output_speed) <= 1e-4:
            output.angular.z = 0.0
        else:
            angular_speed = float(command.angular.z)
            if self.preserve_curvature:
                angular_speed *= abs(output_speed / input_speed)
            ackermann_limit = (
                abs(output_speed) / self.min_turning_radius)
            output.angular.z = max(
                -ackermann_limit,
                min(ackermann_limit, angular_speed),
            )

        self.command_publisher.publish(output)
        self._last_output_was_zero = (
            abs(output_speed) <= 1e-4
            and abs(output.angular.z) <= 1e-4
        )

        diagnostics = Float32MultiArray()
        diagnostics.data = [
            input_speed,
            output_speed,
            curve_limit,
            goal_limit,
            clearance_limit,
            remaining_distance,
            curvature_p90,
            filtered_clearance,
            observed_clearance,
        ]
        self.diagnostics_publisher.publish(diagnostics)
        active = String()
        active.data = active_limit
        self.active_limit_publisher.publish(active)

    def _backup_context_failure(self) -> Optional[str]:
        scan_failure = self._scan_failure_reason()
        if scan_failure is not None:
            return scan_failure
        grid = self._grid
        if grid is None:
            return 'missing_costmap'
        costmap_age = time.monotonic() - grid.receipt_monotonic
        if costmap_age > self.costmap_timeout:
            return (
                f'stale_costmap:{costmap_age:.3f}s>'
                f'{self.costmap_timeout:.3f}s'
            )
        odom_pose = self._odom_pose
        if odom_pose is None:
            return 'missing_odom'
        odom_age = time.monotonic() - odom_pose[3]
        if odom_age > self.odom_timeout:
            return (
                f'stale_odom:{odom_age:.3f}s>'
                f'{self.odom_timeout:.3f}s'
            )
        return None

    def destroy_node(self):
        if hasattr(self, 'command_publisher') and rclpy.ok():
            self._publish_zero('node_shutdown')
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AdaptiveSpeedLimiter()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
