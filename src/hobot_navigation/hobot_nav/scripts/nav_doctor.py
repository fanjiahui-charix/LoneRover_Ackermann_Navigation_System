#!/usr/bin/env python3

import argparse
from pathlib import Path
import sys
import time
from typing import List

import yaml


def build_argument_parser():
    default_waypoints = Path(__file__).resolve().parents[1] / 'waypoints' / 'example_waypoints.yaml'
    parser = argparse.ArgumentParser(description='Check whether the navigation stack topics, TF, and Nav2 actions are ready.')
    parser.add_argument(
        '--mode',
        default='navigation',
        choices=['base', 'pipeline', 'navigation'],
    )
    parser.add_argument('--timeout', default='6.0')
    parser.add_argument('--odom-raw-topic', default='/odom/data_raw')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--imu-topics', default='/imu/data_raw,/imu/fused/data_raw')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--imu-frame', default='imu_link')
    parser.add_argument('--laser-frame', default='laser_link')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--waypoints-file', default=str(default_waypoints))
    parser.add_argument('--route-name', default='via_b')
    return parser


def parse_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def parse_topics(value: str) -> List[str]:
    return [item.strip() for item in value.split(',') if item.strip()]


def format_status(kind: str, label: str, detail: str) -> str:
    return f'[{kind}] {label}: {detail}'


def wait_for_action_server(client, node, timeout_sec: float, rclpy_module) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if client.wait_for_server(timeout_sec=0.2):
            return True
        rclpy_module.spin_once(node, timeout_sec=0.05)
    return False


def wait_for_service(client, node, timeout_sec: float, rclpy_module) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        if client.wait_for_service(timeout_sec=0.2):
            return True
        rclpy_module.spin_once(node, timeout_sec=0.05)
    return False


def load_route_check(path: Path, route_name: str):
    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    params = data.get('navigation_waypoints', {}).get('ros__parameters', {})
    points = params.get('points', {})
    routes = params.get('routes', {})
    if route_name not in routes:
        return False, f'route "{route_name}" not found'
    missing = [name for name in routes[route_name] if name not in points]
    if missing:
        return False, f'route "{route_name}" references missing points: {missing}'
    return True, f'route "{route_name}" = {" -> ".join(routes[route_name])}'


def main(argv=None):
    raw_args = list(argv if argv is not None else sys.argv[1:])
    parser = build_argument_parser()
    args, _unknown = parser.parse_known_args(raw_args)

    try:
        from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
        from rclpy.time import Time
        from sensor_msgs.msg import Imu, LaserScan
        from tf2_ros import Buffer, TransformException, TransformListener
    except ModuleNotFoundError as exc:
        print(f'ROS import failed: {exc}', file=sys.stderr)
        print('Tip: source the target ROS environment before running nav_doctor.py.', file=sys.stderr)
        return 2

    class NavDoctor(Node):
        def __init__(self):
            super().__init__('nav_doctor')
            self.seen = {}
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        def watch_topic(self, topic_name: str, msg_type, key: str, qos=None):
            qos_profile = qos if qos is not None else 10

            def _callback(_msg):
                self.seen[key] = self.seen.get(key, 0) + 1

            self.create_subscription(msg_type, topic_name, _callback, qos_profile)

        def wait(self, timeout_sec: float):
            deadline = time.monotonic() + timeout_sec
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        def count(self, key: str) -> int:
            return self.seen.get(key, 0)

        def check_transform(self, target_frame: str, source_frame: str):
            try:
                self.tf_buffer.lookup_transform(target_frame, source_frame, Time(), timeout=Duration(seconds=0.5))
                return True, ''
            except TransformException as exc:
                return False, str(exc)

    rclpy.init(args=argv)
    node = NavDoctor()

    timeout_sec = max(1.0, float(args.timeout))
    imu_topics = parse_topics(args.imu_topics)
    map_qos = QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )

    node.watch_topic(args.odom_raw_topic, Odometry, 'odom_raw')
    node.watch_topic(args.odom_topic, Odometry, 'odom')
    node.watch_topic(args.scan_topic, LaserScan, 'scan')
    node.watch_topic(args.map_topic, OccupancyGrid, 'map', qos=map_qos)
    for index, topic in enumerate(imu_topics):
        node.watch_topic(topic, Imu, f'imu_{index}')

    node.wait(timeout_sec)

    failures = 0
    lines = []

    mode_requirements = {
        'base': {'map': False, 'nav2': False, 'route': False},
        'pipeline': {'map': False, 'nav2': False, 'route': False},
        'navigation': {'map': True, 'nav2': True, 'route': False},
    }
    expected = mode_requirements[args.mode]

    if node.count('odom_raw') > 0:
        lines.append(format_status('PASS', args.odom_raw_topic, 'raw wheel odom messages received'))
    else:
        failures += 1
        lines.append(format_status('FAIL', args.odom_raw_topic, 'no raw wheel odom messages received'))

    if node.count('odom') > 0:
        lines.append(format_status('PASS', args.odom_topic, 'filtered odom messages received'))
    elif args.mode == 'base':
        lines.append(format_status('WARN', args.odom_topic, 'no filtered odom yet; start EKF before checking'))
    else:
        failures += 1
        lines.append(format_status('FAIL', args.odom_topic, 'no filtered odom messages received'))

    imu_seen = [(topic, node.count(f'imu_{index}')) for index, topic in enumerate(imu_topics)]
    active_imus = [topic for topic, count in imu_seen if count > 0]
    if active_imus:
        lines.append(format_status('PASS', 'IMU', f'detected topic(s): {", ".join(active_imus)}'))
    else:
        failures += 1
        lines.append(format_status('FAIL', 'IMU', f'no IMU messages on candidates: {", ".join(imu_topics)}'))

    if args.mode != 'base':
        if node.count('scan') > 0:
            lines.append(format_status('PASS', args.scan_topic, 'LaserScan messages received'))
        else:
            failures += 1
            lines.append(format_status('FAIL', args.scan_topic, 'no LaserScan messages received'))

    if expected['map']:
        if node.count('map') > 0:
            lines.append(format_status('PASS', args.map_topic, 'map messages received'))
        else:
            failures += 1
            lines.append(format_status('FAIL', args.map_topic, 'no map messages received'))

    tf_checks = [
        (args.odom_frame, args.base_frame, True),
        (args.base_frame, args.imu_frame, True),
    ]
    if args.mode != 'base':
        tf_checks.append((args.base_frame, args.laser_frame, True))
    if expected['map']:
        tf_checks.append((args.map_frame, args.odom_frame, True))

    for target, source, required in tf_checks:
        ok, detail = node.check_transform(target, source)
        label = f'{target} <= {source}'
        if ok:
            lines.append(format_status('PASS', label, 'transform available'))
        elif required:
            failures += 1
            lines.append(format_status('FAIL', label, detail or 'transform unavailable'))
        else:
            lines.append(format_status('WARN', label, detail or 'transform unavailable'))

    if expected['nav2']:
        nav_to_pose_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        nav_through_poses_client = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
        if wait_for_action_server(nav_to_pose_client, node, 2.0, rclpy):
            lines.append(format_status('PASS', 'navigate_to_pose', 'action server available'))
        else:
            failures += 1
            lines.append(format_status('FAIL', 'navigate_to_pose', 'action server unavailable'))
        if wait_for_action_server(nav_through_poses_client, node, 2.0, rclpy):
            lines.append(format_status('PASS', 'navigate_through_poses', 'action server available'))
        else:
            failures += 1
            lines.append(format_status('FAIL', 'navigate_through_poses', 'action server unavailable'))
        nav_to_pose_client.destroy()
        nav_through_poses_client.destroy()

    if expected['route']:
        route_path = Path(args.waypoints_file)
        if not route_path.exists():
            failures += 1
            lines.append(format_status('FAIL', str(route_path), 'waypoint file does not exist'))
        else:
            ok, detail = load_route_check(route_path, args.route_name)
            if ok:
                lines.append(format_status('PASS', 'route', detail))
            else:
                failures += 1
                lines.append(format_status('FAIL', 'route', detail))

    print(f'Navigation doctor mode: {args.mode}')
    for line in lines:
        print(line)

    if failures == 0:
        print('Summary: PASS')
        exit_code = 0
    else:
        print(f'Summary: FAIL ({failures} issue(s))')
        exit_code = 1

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    raise SystemExit(main())
