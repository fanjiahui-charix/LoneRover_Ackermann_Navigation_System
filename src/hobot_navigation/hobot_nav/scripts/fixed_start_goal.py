#!/usr/bin/env python3

from threading import Thread
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

from fixed_start_utils import load_fixed_start_points, yaw_to_quaternion


class FixedStartGoal(Node):
    # Readiness inputs arrive at roughly 150 Hz in aggregate. Graph discovery,
    # three TF lookups and lifecycle polling do not need to run once per input
    # callback; doing so can consume most of one core exactly while all ROS
    # participants are still joining the graph.
    READINESS_CHECK_PERIOD_SEC = 0.10
    # Action completion and semantic-arrival checks are control observations,
    # not controller updates. Keep them at 20 Hz so a ready subscription cannot
    # turn synchronous spin_once() loops into a full-core busy poll.
    ACTION_STATUS_CHECK_PERIOD_SEC = 0.05

    def __init__(self, node_name='fixed_start_goal'):
        super().__init__(node_name)
        self.declare_parameter('points_file', '')
        self.declare_parameter('readiness_timeout_sec', 60.0)
        self.declare_parameter('navigation_timeout_sec', 300.0)
        self.declare_parameter('topic_stale_timeout_sec', 1.0)
        self.declare_parameter('lifecycle_request_timeout_sec', 2.0)
        self.declare_parameter('lifecycle_retry_period_sec', 0.5)
        self.declare_parameter('lifecycle_max_in_flight', 1)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('required_nodes', [
            '/origincarpro_base_node',
            '/ekf_fusion_node',
            '/n10_driver_node',
            '/scan_filter_node',
            '/simple_lidar_odom',
            '/map_server',
            '/competition_command_limiter',
            '/controller_server',
            '/smoother_server',
            '/planner_server',
            '/behavior_server',
            '/bt_navigator',
            '/velocity_smoother',
            '/waypoint_follower',
            '/collision_monitor',
        ])
        self.declare_parameter('lifecycle_nodes', [
            'map_server',
            'controller_server',
            'smoother_server',
            'planner_server',
            'behavior_server',
            'bt_navigator',
            'velocity_smoother',
            'waypoint_follower',
            'collision_monitor',
        ])

        self.config = load_fixed_start_points(
            self.get_parameter('points_file').value, require_calibrated=True)
        self.readiness_timeout = float(
            self.get_parameter('readiness_timeout_sec').value)
        self.navigation_timeout = float(
            self.get_parameter('navigation_timeout_sec').value)
        self.topic_stale_timeout = float(
            self.get_parameter('topic_stale_timeout_sec').value)
        self.lifecycle_request_timeout = float(
            self.get_parameter('lifecycle_request_timeout_sec').value)
        self.lifecycle_retry_period = float(
            self.get_parameter('lifecycle_retry_period_sec').value)
        self.lifecycle_max_in_flight = int(
            self.get_parameter('lifecycle_max_in_flight').value)
        if self.lifecycle_request_timeout <= 0.0:
            raise ValueError('lifecycle_request_timeout_sec must be positive')
        if self.lifecycle_retry_period < 0.0:
            raise ValueError('lifecycle_retry_period_sec must be non-negative')
        if self.lifecycle_max_in_flight < 1:
            raise ValueError('lifecycle_max_in_flight must be at least 1')
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.behavior_tree = self.get_parameter('behavior_tree').value
        self.required_nodes = set(self.get_parameter('required_nodes').value)
        self.lifecycle_names = list(self.get_parameter('lifecycle_nodes').value)

        self.last_seen = {}
        self._topic_subscriptions = [
            self.create_subscription(Odometry, '/odom', self._seen('odom'), 20),
            self.create_subscription(
                Odometry, '/odom/data_raw', self._seen('odom_raw'), 20),
            self.create_subscription(Imu, '/imu/data_raw', self._seen('imu_raw'), 20),
            self.create_subscription(
                LaserScan, '/scan', self._seen('scan'), qos_profile_sensor_data),
        ]
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._topic_subscriptions.append(self.create_subscription(
            OccupancyGrid, '/map', self._seen('map'), map_qos))

        # Mission control is intentionally synchronous and calls spin_once()
        # itself. Do not put /tf in that same callback queue: odom, IMU, scan,
        # QR and action callbacks can otherwise leave this node's TF buffer
        # permanently behind even while the system TF is current.
        self.tf_buffer = Buffer()
        self._tf_listener_node = Node(
            f'{self.get_name()}_tf_listener',
            context=self.context,
            namespace=self.get_namespace(),
            use_global_arguments=False,
            enable_rosout=False,
            start_parameter_services=False,
        )
        self.tf_listener = TransformListener(
            self.tf_buffer, self._tf_listener_node, spin_thread=False)
        self._tf_executor = SingleThreadedExecutor(context=self.context)
        self._tf_executor.add_node(self._tf_listener_node)
        self._tf_thread = Thread(
            target=self._tf_executor.spin,
            name=f'{self.get_name()}_tf_listener',
            daemon=True,
        )
        self._tf_thread.start()
        self._tf_resources_closed = False
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        audit_qos = QoSProfile(depth=20)
        audit_qos.reliability = ReliabilityPolicy.RELIABLE
        audit_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/fixed_start/goal_pose', audit_qos)
        self.action_result_status_pub = self.create_publisher(
            Int8, '/fixed_start/action_result_status', audit_qos)
        self.mission_event_pub = self.create_publisher(
            String, '/fixed_start/mission_event', audit_qos)
        self.lifecycle_clients = {
            name: self.create_client(GetState, f'/{name}/get_state')
            for name in self.lifecycle_names
        }
        self.lifecycle_futures = {name: None for name in self.lifecycle_names}
        self.lifecycle_request_started = {name: 0.0 for name in self.lifecycle_names}
        self.lifecycle_retry_after = {name: 0.0 for name in self.lifecycle_names}
        self.lifecycle_states = {name: '' for name in self.lifecycle_names}
        self.last_feedback_log = 0.0

    def _seen(self, key):
        def callback(_msg):
            self.last_seen[key] = time.monotonic()
        return callback

    def _spin_once_at_action_check_rate(self):
        started = time.monotonic()
        rclpy.spin_once(
            self, timeout_sec=self.ACTION_STATUS_CHECK_PERIOD_SEC)
        remaining = (
            self.ACTION_STATUS_CHECK_PERIOD_SEC
            - (time.monotonic() - started)
        )
        if remaining > 0.0:
            time.sleep(remaining)

    def _stop_readiness_topic_monitoring(self):
        # These subscriptions only prove that startup inputs are live. Keeping
        # their high-rate callbacks after readiness wastes CPU and can delay QR
        # and action callbacks in the mission's synchronous executor.
        for subscription in self._topic_subscriptions:
            self.destroy_subscription(subscription)
        self._topic_subscriptions.clear()

    def destroy_node(self):
        if not self._tf_resources_closed:
            self._tf_resources_closed = True
            self._tf_executor.shutdown(timeout_sec=1.0)
            self._tf_thread.join(timeout=1.0)
            self._tf_executor.remove_node(self._tf_listener_node)
            self._tf_listener_node.destroy_node()
        return super().destroy_node()

    def _reset_lifecycle_client(self, name):
        client = self.lifecycle_clients[name]
        self.destroy_client(client)
        self.lifecycle_clients[name] = self.create_client(
            GetState, f'/{name}/get_state')

    def _finish_lifecycle_request(self, name, now):
        future = self.lifecycle_futures[name]
        try:
            result = future.result()
            if result is None:
                raise RuntimeError('empty lifecycle GetState response')
            self.lifecycle_states[name] = result.current_state.label
            if self.lifecycle_states[name] == 'active':
                self.lifecycle_retry_after[name] = 0.0
            else:
                self.lifecycle_retry_after[name] = now + self.lifecycle_retry_period
        except Exception as exc:  # pragma: no cover - runtime diagnostics
            self.get_logger().warn(f'get_state failed for {name}: {exc}; retrying')
            self.lifecycle_states[name] = ''
            self.lifecycle_retry_after[name] = now + self.lifecycle_retry_period
            self._reset_lifecycle_client(name)
        finally:
            self.lifecycle_futures[name] = None
            self.lifecycle_request_started[name] = 0.0

    def _timeout_lifecycle_request(self, name, now):
        future = self.lifecycle_futures[name]
        future.cancel()
        self.lifecycle_futures[name] = None
        self.lifecycle_request_started[name] = 0.0
        self.lifecycle_states[name] = ''
        self.lifecycle_retry_after[name] = now + self.lifecycle_retry_period
        self._reset_lifecycle_client(name)
        self.get_logger().warn(
            f'get_state timed out for {name} after '
            f'{self.lifecycle_request_timeout:.1f}s; client reset, retrying')

    def _refresh_lifecycle_states(self):
        now = time.monotonic()
        for name in self.lifecycle_names:
            future = self.lifecycle_futures[name]
            if future is not None and future.done():
                self._finish_lifecycle_request(name, now)
            elif (future is not None and
                  now - self.lifecycle_request_started[name] >=
                  self.lifecycle_request_timeout):
                self._timeout_lifecycle_request(name, now)

        in_flight = sum(
            future is not None for future in self.lifecycle_futures.values())
        if in_flight >= self.lifecycle_max_in_flight:
            return

        for name in self.lifecycle_names:
            # Readiness only needs one confirmed active response. Do not keep
            # polling active lifecycle nodes at the main loop rate.
            if self.lifecycle_states[name] == 'active':
                continue
            if self.lifecycle_futures[name] is not None:
                continue
            if now < self.lifecycle_retry_after[name]:
                continue
            client = self.lifecycle_clients[name]
            if not client.service_is_ready():
                continue
            self.lifecycle_futures[name] = client.call_async(GetState.Request())
            self.lifecycle_request_started[name] = now
            in_flight += 1
            if in_flight >= self.lifecycle_max_in_flight:
                break

    def _current_node_names(self):
        result = set()
        for name, namespace in self.get_node_names_and_namespaces():
            prefix = namespace.rstrip('/')
            result.add(f'{prefix}/{name}' if prefix else f'/{name}')
        return result

    def _missing_readiness(self):
        now = time.monotonic()
        missing = []
        for key in ('odom', 'odom_raw', 'imu_raw', 'scan'):
            stamp = self.last_seen.get(key)
            if stamp is None or now - stamp > self.topic_stale_timeout:
                missing.append(f'topic:{key}')
        if 'map' not in self.last_seen:
            missing.append('topic:map')

        try:
            self.tf_buffer.lookup_transform(
                self.map_frame, self.odom_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            missing.append(f'tf:{self.map_frame}->{self.odom_frame}')
        try:
            self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            missing.append(f'tf:{self.odom_frame}->{self.base_frame}')
        try:
            self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, Time(),
                timeout=Duration(seconds=0.05))
        except TransformException:
            missing.append(f'tf:{self.map_frame}->{self.base_frame}')

        present_nodes = self._current_node_names()
        for name in sorted(self.required_nodes - present_nodes):
            missing.append(f'node:{name}')

        self._refresh_lifecycle_states()
        for name in self.lifecycle_names:
            if self.lifecycle_states[name] != 'active':
                missing.append(
                    f'lifecycle:{name}={self.lifecycle_states[name] or "unavailable"}')
        if not self.action_client.server_is_ready():
            missing.append('action:navigate_to_pose')
        return missing

    def wait_until_ready(self):
        deadline = time.monotonic() + self.readiness_timeout
        next_log = 0.0
        next_readiness_check = 0.0
        last_missing = []
        while rclpy.ok() and time.monotonic() < deadline:
            # Keep consuming startup topic callbacks so freshness remains
            # accurate, but bound the expensive graph/TF/lifecycle audit to
            # 10 Hz. High-rate callbacks can otherwise make spin_once return
            # immediately and turn this loop into an unbounded busy poll.
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now < next_readiness_check:
                continue
            next_readiness_check = now + self.READINESS_CHECK_PERIOD_SEC
            last_missing = self._missing_readiness()
            if not last_missing:
                self.get_logger().info(
                    'Readiness PASS: wheel, IMU, EKF odom, filtered scan, map, TF, '
                    'Nav2 and Collision Monitor are ready.')
                self._stop_readiness_topic_monitoring()
                return True
            if now >= next_log:
                self.get_logger().info(
                    'Waiting for safe navigation readiness: ' + ', '.join(last_missing))
                next_log = now + 5.0
        self.get_logger().error(
            'Readiness timed out; QR_NAV was not sent. Missing: ' +
            ', '.join(last_missing))
        return False

    def _feedback(self, feedback_msg):
        now = time.monotonic()
        if now - self.last_feedback_log < 5.0:
            return
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Navigating to QR_NAV; distance_remaining='
            f'{feedback.distance_remaining:.3f} m')
        self.last_feedback_log = now

    def _publish_mission_event(self, event, status=None):
        event_msg = String()
        event_msg.data = event
        self.mission_event_pub.publish(event_msg)
        if status is not None:
            status_msg = Int8()
            status_msg.data = int(status)
            self.action_result_status_pub.publish(status_msg)

    def navigate(self):
        if not self.wait_until_ready():
            self._publish_mission_event('readiness_failed; goal_not_sent')
            return 1

        x, y, yaw = self.config['QR_NAV']
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        goal.behavior_tree = self.behavior_tree
        self.goal_pose_pub.publish(goal.pose)
        self._publish_mission_event(
            f'goal_request; frame={self.map_frame}; x={x:.9f}; y={y:.9f}; '
            f'yaw={yaw:.9f}')

        self.get_logger().info(
            f'Sending the only task goal QR_NAV=({x:.4f}, {y:.4f}, {yaw:.4f})')
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self._feedback)
        send_deadline = time.monotonic() + 5.0
        while rclpy.ok() and not send_future.done() and time.monotonic() < send_deadline:
            self._spin_once_at_action_check_rate()
        if not send_future.done():
            self.get_logger().error('NavigateToPose goal request timed out.')
            self._publish_mission_event('goal_request_timeout; goal_result_unavailable')
            return 1
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Nav2 rejected QR_NAV.')
            self._publish_mission_event('goal_rejected; goal_result_unavailable')
            return 1
        self._publish_mission_event('goal_accepted')

        result_future = goal_handle.get_result_async()
        nav_deadline = time.monotonic() + self.navigation_timeout
        while rclpy.ok() and not result_future.done() and time.monotonic() < nav_deadline:
            self._spin_once_at_action_check_rate()
        if not result_future.done():
            self.get_logger().error('Navigation timed out; canceling QR_NAV.')
            self._publish_mission_event('navigation_timeout; cancel_requested')
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 2.0
            while (rclpy.ok() and not cancel_future.done() and
                   time.monotonic() < cancel_deadline):
                self._spin_once_at_action_check_rate()
            return 1

        wrapped_result = result_future.result()
        self._publish_mission_event(
            f'action_result; status={wrapped_result.status}', wrapped_result.status)
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(
                'QR_NAV reached successfully. No QR vision or follow-on task will run.')
            return 0
        self.get_logger().error(
            f'QR_NAV failed with action status {wrapped_result.status}.')
        return 1


def main():
    rclpy.init()
    node = FixedStartGoal()
    exit_code = 1
    try:
        exit_code = node.navigate()
    except KeyboardInterrupt:
        node.get_logger().warn('Fixed-start mission interrupted.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
