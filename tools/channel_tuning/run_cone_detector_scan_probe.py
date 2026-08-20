#!/usr/bin/env python3
"""Motor-free X5 probe for the installed /scan -> cone detector stage."""

from __future__ import annotations

import argparse
import json
import math
import struct
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanProbe(Node):
    def __init__(self, targets: list[tuple[float, float]], frames: int, out: str):
        super().__init__("channel_scan_probe")
        self.publisher = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, "/cones/points", self.on_cones, qos_profile_sensor_data)
        self.targets = targets
        self.frames = frames
        self.out = out
        self.received: list[list[dict[str, float]]] = []
        self.sent = 0
        self.timer = self.create_timer(0.10, self.tick)

    def on_cones(self, msg: PointCloud2) -> None:
        points = []
        # cone_detector_node publishes x/y/z/(range) float32 fields.  Read the
        # first two named fields without depending on point-field ordering.
        offsets = {field.name: field.offset for field in msg.fields}
        if "x" not in offsets or "y" not in offsets:
            return
        for index in range(msg.width * max(1, msg.height)):
            base = index * msg.point_step
            try:
                x = struct.unpack_from("<f", msg.data, base + offsets["x"])[0]
                y = struct.unpack_from("<f", msg.data, base + offsets["y"])[0]
            except struct.error:
                continue
            if math.isfinite(x) and math.isfinite(y):
                points.append({"x_m": round(float(x), 5), "y_m": round(float(y), 5)})
        if points:
            self.received.append(points)

    def tick(self) -> None:
        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = "laser_link"
        scan.angle_min = -math.pi
        scan.angle_increment = math.radians(0.5)
        scan.angle_max = scan.angle_min + scan.angle_increment * 719
        scan.range_min = 0.15
        scan.range_max = 6.0
        scan.ranges = [float("inf")] * 720
        # Render the near-side circular arc of a 45 mm scan-height cone rather
        # than a constant-range line.  This keeps the production PCA filter
        # meaningful while producing the same cluster shape as a real cone.
        for radius, angle in self.targets:
            center = round((angle - scan.angle_min) / scan.angle_increment)
            for delta in range(-5, 6):
                index = center + delta
                if 0 <= index < len(scan.ranges):
                    relative_angle = delta * scan.angle_increment
                    cross_track = radius * math.sin(relative_angle)
                    disc = max(0.0, 0.045 * 0.045 - cross_track * cross_track)
                    near_range = radius * math.cos(relative_angle) - math.sqrt(disc)
                    scan.ranges[index] = near_range
        self.publisher.publish(scan)
        self.sent += 1
        if self.sent >= self.frames:
            self.timer.cancel()
            self.create_timer(0.50, self.finish)

    def finish(self) -> None:
        payload = {
            "schema_version": 1,
            "scan_topic": "/scan",
            "detector_output": "/cones/points",
            "sent_frames": self.sent,
            "received_messages": len(self.received),
            "last_detected": self.received[-1] if self.received else [],
            "detected_count": len(self.received[-1]) if self.received else 0,
            "pass": bool(self.received and len(self.received[-1]) >= 1),
            "physical_base_started": False,
            "stm32_output_enabled": False,
        }
        with open(self.out, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(payload, sort_keys=True))
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True)
    parser.add_argument("--frames", type=int, default=10)
    parser.add_argument(
        "--target", action="append", default=["1.00,-0.30", "1.20,0.50"],
        help="range_m,angle_rad; repeat for additional fixed cones",
    )
    args = parser.parse_args()
    targets = []
    for value in args.target:
        radius, angle = value.split(",", 1)
        targets.append((float(radius), float(angle)))
    rclpy.init()
    node = ScanProbe(targets, max(4, args.frames), args.out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
