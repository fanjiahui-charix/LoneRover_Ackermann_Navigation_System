#!/usr/bin/env python3

from collections import deque
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster

from fixed_start_utils import (
    compute_map_to_odom,
    load_fixed_start_points,
    quaternion_to_yaw,
    stable_pose_from_samples,
    yaw_to_quaternion,
)


class FixedStartTf(Node):
    def __init__(self):
        super().__init__('fixed_start_tf')
        self.declare_parameter('points_file', '')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('init_timeout_sec', 45.0)
        self.declare_parameter('stable_sample_count', 30)
        self.declare_parameter('stable_min_duration_sec', 0.5)
        self.declare_parameter('max_pose_drift_m', 0.01)
        self.declare_parameter('max_yaw_drift_rad', 0.02)
        self.declare_parameter('max_linear_speed_mps', 0.02)
        self.declare_parameter('max_angular_speed_rps', 0.03)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.config = load_fixed_start_points(
            self.get_parameter('points_file').value, require_calibrated=True)
        if self.config['frame_id'] != self.map_frame:
            raise ValueError(
                f'points frame {self.config["frame_id"]} != map_frame {self.map_frame}')

        self.init_timeout_sec = float(self.get_parameter('init_timeout_sec').value)
        self.stable_sample_count = max(
            2, int(self.get_parameter('stable_sample_count').value))
        self.stability_args = (
            float(self.get_parameter('stable_min_duration_sec').value),
            float(self.get_parameter('max_pose_drift_m').value),
            float(self.get_parameter('max_yaw_drift_rad').value),
            float(self.get_parameter('max_linear_speed_mps').value),
            float(self.get_parameter('max_angular_speed_rps').value),
        )
        self.samples = deque(maxlen=self.stable_sample_count)
        self.started_at = time.monotonic()
        self.initialized = False
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        state_qos = QoSProfile(depth=1)
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.state_pub = self.create_publisher(
            Bool, '/fixed_start/initialized', state_qos)
        self.alignment_pub = self.create_publisher(
            TransformStamped, '/fixed_start/map_to_odom', state_qos)
        self.state_pub.publish(Bool(data=False))

        odom_topic = self.get_parameter('odom_topic').value
        self.odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, 20)
        self.timeout_timer = self.create_timer(0.25, self._check_timeout)
        self.get_logger().info(
            f'Waiting for {self.stable_sample_count} stable EKF odom samples on {odom_topic}; '
            f'P(map)={self.config["P"]}')

    def _on_odom(self, msg):
        if self.initialized:
            return
        if msg.header.frame_id != self.odom_frame or msg.child_frame_id != self.base_frame:
            self.get_logger().error(
                f'Refusing odom with frames {msg.header.frame_id}->{msg.child_frame_id}; '
                f'expected {self.odom_frame}->{self.base_frame}')
            return

        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(
            orientation.x, orientation.y, orientation.z, orientation.w)
        twist = msg.twist.twist
        self.samples.append((
            time.monotonic(),
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            twist.linear.x,
            twist.linear.y,
            twist.angular.z,
        ))
        if len(self.samples) < self.stable_sample_count:
            return

        start_pose = stable_pose_from_samples(
            list(self.samples), *self.stability_args)
        if start_pose is None:
            return
        map_to_odom = compute_map_to_odom(self.config['P'], start_pose)

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame
        transform.child_frame_id = self.odom_frame
        transform.transform.translation.x = map_to_odom[0]
        transform.transform.translation.y = map_to_odom[1]
        transform.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(map_to_odom[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        # StaticTransformBroadcaster is intentionally called exactly once.
        self.tf_broadcaster.sendTransform(transform)
        self.alignment_pub.publish(transform)
        self.initialized = True
        self.state_pub.publish(Bool(data=True))
        self.get_logger().info(
            'Fixed-start alignment initialized once: '
            f'odom_start={tuple(round(v, 6) for v in start_pose)}, '
            f'map_to_odom={tuple(round(v, 6) for v in map_to_odom)}')

    def _check_timeout(self):
        if self.initialized:
            return
        if time.monotonic() - self.started_at <= self.init_timeout_sec:
            return
        self.get_logger().fatal(
            'Fixed-start initialization timed out. Keep the vehicle stationary and check /odom; '
            'no navigation goal was sent.')
        rclpy.shutdown()


def main():
    rclpy.init()
    node = FixedStartTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
