#!/usr/bin/env python3

from pathlib import Path

import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def next_token(handle):
    token = bytearray()
    while True:
        char = handle.read(1)
        if not char:
            return bytes(token) if token else b""
        if char == b"#":
            handle.readline()
            continue
        if char.isspace():
            if token:
                return bytes(token)
            continue
        token.extend(char)


def load_pgm(path):
    with Path(path).open("rb") as handle:
        magic = next_token(handle)
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"Unsupported PGM format: {magic!r}")
        width = int(next_token(handle))
        height = int(next_token(handle))
        max_value = int(next_token(handle))
        if max_value <= 0 or max_value > 255:
            raise ValueError(f"Unsupported PGM max value: {max_value}")
        if magic == b"P5":
            pixels = list(handle.read(width * height))
        else:
            pixels = [int(next_token(handle)) for _ in range(width * height)]
    if len(pixels) != width * height:
        raise ValueError(f"PGM size mismatch: expected {width * height}, got {len(pixels)}")
    return width, height, pixels


def pixel_to_occupancy(pixel, occupied_thresh, free_thresh, negate):
    value = pixel / 255.0
    if negate:
        value = 1.0 - value
    occupancy = 1.0 - value
    if occupancy > occupied_thresh:
        return 100
    if occupancy < free_thresh:
        return 0
    return -1


class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__("ekf_fusion_static_map_publisher")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("map_yaml_file", "")
        self.declare_parameter("publish_period", 1.0)

        map_topic = self.get_parameter("map_topic").value
        frame_id = self.get_parameter("frame_id").value
        map_yaml_file = Path(self.get_parameter("map_yaml_file").value).expanduser()
        publish_period = float(self.get_parameter("publish_period").value)

        self.map_msg = self.load_map(map_yaml_file, frame_id)

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher = self.create_publisher(OccupancyGrid, map_topic, qos)
        self.timer = self.create_timer(max(0.2, publish_period), self.publish_map)
        self.publish_map()

        self.get_logger().info(
            f"Publishing static map {map_yaml_file} on {map_topic}, frame_id={frame_id}"
        )

    def load_map(self, yaml_path, frame_id):
        if not yaml_path:
            raise ValueError("map_yaml_file parameter is empty")
        with yaml_path.open("r", encoding="utf-8") as handle:
            config = yaml.safe_load(handle)

        image_path = Path(config["image"])
        if not image_path.is_absolute():
            image_path = yaml_path.parent / image_path

        width, height, pixels = load_pgm(image_path)
        resolution = float(config["resolution"])
        origin = config.get("origin", [0.0, 0.0, 0.0])
        occupied_thresh = float(config.get("occupied_thresh", 0.65))
        free_thresh = float(config.get("free_thresh", 0.196))
        negate = int(config.get("negate", 0)) != 0

        msg = OccupancyGrid()
        msg.header.frame_id = frame_id
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.position.x = float(origin[0])
        msg.info.origin.position.y = float(origin[1])
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        data = []
        for y in reversed(range(height)):
            row_start = y * width
            for x in range(width):
                data.append(pixel_to_occupancy(
                    pixels[row_start + x],
                    occupied_thresh,
                    free_thresh,
                    negate,
                ))
        msg.data = data
        return msg

    def publish_map(self):
        self.map_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.map_msg)


def main():
    rclpy.init()
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
