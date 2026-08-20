#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class StaticFramesPublisher(Node):
    def __init__(self):
        super().__init__('static_frames_publisher')
        self.broadcaster = StaticTransformBroadcaster(self)
        self._declare_parameters()
        transforms = self._build_transforms()
        if transforms:
            self.broadcaster.sendTransform(transforms)
            children = ', '.join(t.child_frame_id for t in transforms)
            self.get_logger().info(f'Published static transforms: {children}')
        else:
            self.get_logger().warn('No static transforms enabled in frames.yaml')

    def _declare_parameters(self):
        self._declare_transform_parameters('imu', 'base_link', 'imu_link', enabled_by_default=True)
        self._declare_transform_parameters('laser', 'base_link', 'laser_link', enabled_by_default=True)

    def _declare_transform_parameters(
        self,
        prefix: str,
        parent_frame: str,
        child_frame: str,
        *,
        enabled_by_default: bool,
        default_xyz=(0.0, 0.0, 0.0),
        default_rpy=(0.0, 0.0, 0.0),
    ):
        self.declare_parameter(f'publish_{prefix}_tf', enabled_by_default)
        self.declare_parameter(f'{prefix}.parent_frame', parent_frame)
        self.declare_parameter(f'{prefix}.child_frame', child_frame)
        self.declare_parameter(f'{prefix}.x', default_xyz[0])
        self.declare_parameter(f'{prefix}.y', default_xyz[1])
        self.declare_parameter(f'{prefix}.z', default_xyz[2])
        self.declare_parameter(f'{prefix}.roll', default_rpy[0])
        self.declare_parameter(f'{prefix}.pitch', default_rpy[1])
        self.declare_parameter(f'{prefix}.yaw', default_rpy[2])

    def _build_transforms(self):
        transforms = []
        for prefix in ('imu', 'laser'):
            if self.get_parameter(f'publish_{prefix}_tf').get_parameter_value().bool_value:
                transforms.append(self._make_transform(prefix))
        return transforms

    def _make_transform(self, prefix: str):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.get_parameter(f'{prefix}.parent_frame').value
        transform.child_frame_id = self.get_parameter(f'{prefix}.child_frame').value
        transform.transform.translation.x = float(self.get_parameter(f'{prefix}.x').value)
        transform.transform.translation.y = float(self.get_parameter(f'{prefix}.y').value)
        transform.transform.translation.z = float(self.get_parameter(f'{prefix}.z').value)
        qx, qy, qz, qw = quaternion_from_euler(
            float(self.get_parameter(f'{prefix}.roll').value),
            float(self.get_parameter(f'{prefix}.pitch').value),
            float(self.get_parameter(f'{prefix}.yaw').value),
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        return transform


def main():
    rclpy.init()
    node = StaticFramesPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
