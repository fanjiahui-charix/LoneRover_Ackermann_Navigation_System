#!/usr/bin/env python3

import math
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class LocalScanPlanner(Node):
    def __init__(self) -> None:
        super().__init__("local_scan_planner")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "cmd_vel")
        self.declare_parameter("control_rate", 10.0)
        self.declare_parameter("scan_timeout", 0.5)
        self.declare_parameter("max_planning_angle_deg", 100.0)
        self.declare_parameter("front_sector_deg", 20.0)
        self.declare_parameter("side_sector_deg", 60.0)
        self.declare_parameter("stop_distance", 0.45)
        self.declare_parameter("slow_distance", 1.20)
        self.declare_parameter("turn_clearance_distance", 0.80)
        self.declare_parameter("forward_speed", 0.35)
        self.declare_parameter("min_forward_speed", 0.08)
        self.declare_parameter("max_angular_speed", 1.20)
        self.declare_parameter("turn_in_place_speed", 0.90)
        self.declare_parameter("heading_gain", 1.80)
        self.declare_parameter("heading_weight", 0.35)
        self.declare_parameter("min_valid_range", 0.05)
        self.declare_parameter("safety_bubble_radius", 0.10)
        self.declare_parameter("scan_filter_enabled", True)
        self.declare_parameter("scan_filter_window", 5)

        self.scan_topic = self.get_parameter("scan_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.control_rate = float(self.get_parameter("control_rate").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        self.max_planning_angle = math.radians(float(self.get_parameter("max_planning_angle_deg").value))
        self.front_sector = math.radians(float(self.get_parameter("front_sector_deg").value))
        self.side_sector = math.radians(float(self.get_parameter("side_sector_deg").value))
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.slow_distance = float(self.get_parameter("slow_distance").value)
        self.turn_clearance_distance = float(self.get_parameter("turn_clearance_distance").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.min_forward_speed = float(self.get_parameter("min_forward_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.turn_in_place_speed = float(self.get_parameter("turn_in_place_speed").value)
        self.heading_gain = float(self.get_parameter("heading_gain").value)
        self.heading_weight = float(self.get_parameter("heading_weight").value)
        self.min_valid_range = float(self.get_parameter("min_valid_range").value)
        self.safety_bubble_radius = float(self.get_parameter("safety_bubble_radius").value)
        self.scan_filter_enabled = bool(self.get_parameter("scan_filter_enabled").value)
        self.scan_filter_window = int(self.get_parameter("scan_filter_window").value)
        if self.scan_filter_window % 2 == 0:
            self.scan_filter_window += 1

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, cmd_qos)
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / max(self.control_rate, 1.0), self.on_timer)

        self.last_scan: Optional[LaserScan] = None
        self.last_scan_stamp = None
        self.last_turn_sign = 1.0

        self.get_logger().info(
            f"local_scan_planner started, scan_topic={self.scan_topic}, cmd_vel_topic={self.cmd_vel_topic}"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = self.filter_scan(msg) if self.scan_filter_enabled else msg
        self.last_scan_stamp = self.get_clock().now()

    def on_timer(self) -> None:
        if self.last_scan is None or self.last_scan_stamp is None:
            self.publish_stop()
            return

        if self.get_clock().now() - self.last_scan_stamp > Duration(seconds=self.scan_timeout):
            self.publish_stop()
            return

        cmd = self.compute_command(self.last_scan)
        self.cmd_pub.publish(cmd)

    def compute_command(self, scan: LaserScan) -> Twist:
        best_angle, best_range = self.find_best_heading(scan)
        front_clearance = self.sector_clearance(scan, -self.front_sector, self.front_sector)
        left_clearance = self.sector_clearance(scan, 0.0, self.side_sector)
        right_clearance = self.sector_clearance(scan, -self.side_sector, 0.0)

        cmd = Twist()
        if best_angle is None or best_range is None:
            cmd.angular.z = self.turn_in_place_speed * self.preferred_turn_sign(left_clearance, right_clearance)
            self.last_turn_sign = math.copysign(1.0, cmd.angular.z)
            return cmd

        angular = clamp(self.heading_gain * best_angle, -self.max_angular_speed, self.max_angular_speed)
        preferred_turn_sign = math.copysign(1.0, angular) if abs(angular) > 1e-3 else self.preferred_turn_sign(
            left_clearance, right_clearance
        )

        if front_clearance <= self.stop_distance:
            cmd.angular.z = self.turn_in_place_speed * preferred_turn_sign
            self.last_turn_sign = preferred_turn_sign
            return cmd

        if abs(best_angle) > math.radians(35.0) and front_clearance < self.turn_clearance_distance:
            cmd.angular.z = clamp(angular, -self.turn_in_place_speed, self.turn_in_place_speed)
            self.last_turn_sign = preferred_turn_sign
            return cmd

        speed_scale = clamp(
            (front_clearance - self.stop_distance) / max(self.slow_distance - self.stop_distance, 1e-3),
            0.0,
            1.0,
        )
        alignment_scale = clamp(1.0 - abs(best_angle) / max(self.max_planning_angle, 1e-3), 0.2, 1.0)

        linear = self.forward_speed * min(speed_scale, alignment_scale)
        if linear > 0.0:
            linear = max(linear, self.min_forward_speed)

        cmd.linear.x = linear
        cmd.angular.z = angular
        self.last_turn_sign = preferred_turn_sign
        return cmd

    def sector_clearance(self, scan: LaserScan, min_angle: float, max_angle: float) -> float:
        values: List[float] = []
        for angle, distance in self.iter_valid_points(scan):
            if min_angle <= angle <= max_angle:
                values.append(distance)
        if not values:
            return 0.0
        return min(values)

    def find_best_heading(self, scan: LaserScan) -> Tuple[Optional[float], Optional[float]]:
        candidates: List[Tuple[float, float]] = []
        for angle, distance in self.iter_valid_points(scan):
            if abs(angle) > self.max_planning_angle:
                continue
            effective_distance = max(0.0, distance - self.safety_bubble_radius)
            score = effective_distance - self.heading_weight * abs(angle)
            candidates.append((score, angle, effective_distance))

        if not candidates:
            return None, None

        _, best_angle, best_range = max(candidates, key=lambda item: item[0])
        return best_angle, best_range

    def iter_valid_points(self, scan: LaserScan):
        for index, raw_range in enumerate(scan.ranges):
            if math.isnan(raw_range):
                continue
            if math.isinf(raw_range):
                if scan.range_max <= 0.0:
                    continue
                distance = scan.range_max
            else:
                distance = raw_range
            if distance < max(scan.range_min, self.min_valid_range):
                continue
            if scan.range_max > 0.0 and distance > scan.range_max:
                continue
            angle = scan.angle_min + index * scan.angle_increment
            yield angle, distance

    def filter_scan(self, scan: LaserScan) -> LaserScan:
        if self.scan_filter_window <= 1 or not scan.ranges:
            return scan

        filtered = LaserScan()
        filtered.header = scan.header
        filtered.angle_min = scan.angle_min
        filtered.angle_max = scan.angle_max
        filtered.angle_increment = scan.angle_increment
        filtered.time_increment = scan.time_increment
        filtered.scan_time = scan.scan_time
        filtered.range_min = scan.range_min
        filtered.range_max = scan.range_max
        filtered.intensities = list(scan.intensities)
        filtered.ranges = list(scan.ranges)

        radius = self.scan_filter_window // 2
        total = len(scan.ranges)
        for index in range(total):
            window = []
            start = max(0, index - radius)
            end = min(total - 1, index + radius)
            for sample in range(start, end + 1):
                value = scan.ranges[sample]
                if math.isfinite(value) and value >= scan.range_min and (scan.range_max <= 0.0 or value <= scan.range_max):
                    window.append(value)
            if not window:
                continue
            window.sort()
            filtered.ranges[index] = window[len(window) // 2]

        return filtered

    def preferred_turn_sign(self, left_clearance: float, right_clearance: float) -> float:
        if left_clearance == right_clearance:
            return self.last_turn_sign
        return 1.0 if left_clearance >= right_clearance else -1.0

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalScanPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
