#!/usr/bin/env python3

import json
import math
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import List, Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class SharedScanState:
    def __init__(
        self,
        cluster_distance_threshold: float,
        cluster_min_points: int,
        cluster_max_range: float,
        view_front_range: float,
        view_rear_range: float,
        view_side_range: float,
    ) -> None:
        self.lock = threading.Lock()
        self.scan = None
        self.updated_at = None
        self.cluster_distance_threshold = cluster_distance_threshold
        self.cluster_min_points = cluster_min_points
        self.cluster_max_range = cluster_max_range
        self.view_front_range = view_front_range
        self.view_rear_range = view_rear_range
        self.view_side_range = view_side_range

    def update(self, msg: LaserScan, stamp) -> None:
        ranges = []
        points = []
        for value in msg.ranges:
            if math.isnan(value):
                ranges.append(None)
            elif math.isinf(value):
                ranges.append(None)
            else:
                ranges.append(value)

        for index, distance in enumerate(ranges):
            if distance is None or distance <= 0.0:
                continue
            if distance < max(msg.range_min, 0.02):
                continue
            if self.cluster_max_range > 0.0 and distance > self.cluster_max_range:
                continue
            angle = msg.angle_min + index * msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            points.append({
                "index": index,
                "angle": angle,
                "range": distance,
                "x": x,
                "y": y,
            })

        clusters = self._cluster_points(points)

        scan_data = {
            "frame_id": msg.header.frame_id,
            "angle_min": msg.angle_min,
            "angle_max": msg.angle_max,
            "angle_increment": msg.angle_increment,
            "range_min": msg.range_min,
            "range_max": msg.range_max,
            "scan_time": msg.scan_time,
            "ranges": ranges,
            "clusters": clusters,
            "view_front_range": self.view_front_range,
            "view_rear_range": self.view_rear_range,
            "view_side_range": self.view_side_range,
        }
        with self.lock:
            self.scan = scan_data
            self.updated_at = stamp

    def snapshot(self):
        with self.lock:
            if self.scan is None:
                return None, None
            return dict(self.scan), self.updated_at

    def _cluster_points(self, points: List[dict]) -> List[dict]:
        if not points:
            return []

        grouped: List[List[dict]] = []
        current = [points[0]]
        for prev, point in zip(points, points[1:]):
            distance = math.hypot(point["x"] - prev["x"], point["y"] - prev["y"])
            if distance <= self.cluster_distance_threshold:
                current.append(point)
            else:
                grouped.append(current)
                current = [point]
        grouped.append(current)

        clusters = []
        cluster_id = 0
        for group in grouped:
            if len(group) < self.cluster_min_points:
                continue

            xs = [point["x"] for point in group]
            ys = [point["y"] for point in group]
            ranges = [point["range"] for point in group]
            centroid_x = sum(xs) / len(xs)
            centroid_y = sum(ys) / len(ys)
            clusters.append({
                "id": cluster_id,
                "count": len(group),
                "min_range": min(ranges),
                "max_range": max(ranges),
                "centroid_x": centroid_x,
                "centroid_y": centroid_y,
                "width": max(xs) - min(xs),
                "height": max(ys) - min(ys),
                "bbox_min_x": min(xs),
                "bbox_max_x": max(xs),
                "bbox_min_y": min(ys),
                "bbox_max_y": max(ys),
                "points": [{"x": point["x"], "y": point["y"], "range": point["range"]} for point in group],
            })
            cluster_id += 1
        return clusters


class LidarHttpServer:
    def __init__(self, host: str, port: int, web_root: Path, state: SharedScanState) -> None:
        self.host = host
        self.port = port
        self.web_root = web_root
        self.state = state
        self.httpd: Optional[ThreadingHTTPServer] = None
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        handler = self._make_handler()
        self.httpd = ThreadingHTTPServer((self.host, self.port), handler)
        self.thread = threading.Thread(target=self.httpd.serve_forever, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        if self.httpd is not None:
            self.httpd.shutdown()
            self.httpd.server_close()
            self.httpd = None
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

    def _make_handler(self):
        web_root = self.web_root
        state = self.state

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self._serve_file(web_root / "index.html", "text/html; charset=utf-8")
                    return
                if self.path == "/scan":
                    self._serve_scan()
                    return
                if self.path == "/health":
                    self._write_json({"status": "ok"})
                    return
                self.send_error(HTTPStatus.NOT_FOUND, "Not found")

            def log_message(self, format, *args):  # noqa: A003
                return

            def _serve_file(self, path: Path, content_type: str) -> None:
                if not path.exists():
                    self.send_error(HTTPStatus.NOT_FOUND, "File not found")
                    return
                body = path.read_bytes()
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def _serve_scan(self) -> None:
                scan, updated_at = state.snapshot()
                if scan is None:
                    self._write_json({"ready": False, "message": "No scan received yet"}, status=HTTPStatus.SERVICE_UNAVAILABLE)
                    return
                if updated_at is not None:
                    scan["updated_at_ns"] = updated_at.nanoseconds
                scan["ready"] = True
                self._write_json(scan)

            def _write_json(self, payload, status: HTTPStatus = HTTPStatus.OK) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        return Handler


class LidarWebViewer(Node):
    def __init__(self) -> None:
        super().__init__("lidar_web_viewer")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8765)
        self.declare_parameter("scan_timeout", 1.0)
        self.declare_parameter("cluster_distance_threshold", 0.18)
        self.declare_parameter("cluster_min_points", 4)
        self.declare_parameter("cluster_max_range", 8.0)
        self.declare_parameter("view_front_range", 5.0)
        self.declare_parameter("view_rear_range", 1.0)
        self.declare_parameter("view_side_range", 3.0)

        scan_topic = self.get_parameter("scan_topic").value
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)
        self.scan_timeout = float(self.get_parameter("scan_timeout").value)
        cluster_distance_threshold = float(self.get_parameter("cluster_distance_threshold").value)
        cluster_min_points = int(self.get_parameter("cluster_min_points").value)
        cluster_max_range = float(self.get_parameter("cluster_max_range").value)
        view_front_range = float(self.get_parameter("view_front_range").value)
        view_rear_range = float(self.get_parameter("view_rear_range").value)
        view_side_range = float(self.get_parameter("view_side_range").value)

        web_root = Path(get_package_share_directory("lidar_web_viewer")) / "web"
        self.state = SharedScanState(
            cluster_distance_threshold,
            cluster_min_points,
            cluster_max_range,
            view_front_range,
            view_rear_range,
            view_side_range,
        )
        self.server = LidarHttpServer(host, port, web_root, self.state)
        self.server.start()

        self.subscription = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.5, self.watchdog)
        self.get_logger().info(
            f"lidar_web_viewer started, scan_topic={scan_topic}, web=http://{host}:{port}, "
            f"cluster_distance_threshold={cluster_distance_threshold:.2f}, "
            f"cluster_min_points={cluster_min_points}, cluster_max_range={cluster_max_range:.2f}, "
            f"view_front_range={view_front_range:.2f}, view_rear_range={view_rear_range:.2f}, "
            f"view_side_range={view_side_range:.2f}"
        )

    def scan_callback(self, msg: LaserScan) -> None:
        self.state.update(msg, self.get_clock().now())

    def watchdog(self) -> None:
        _, updated_at = self.state.snapshot()
        if updated_at is None:
            return
        if self.get_clock().now() - updated_at > Duration(seconds=self.scan_timeout):
            self.get_logger().warn("scan timeout, browser view will show stale data")

    def destroy_node(self):
        self.server.stop()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LidarWebViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
