#!/usr/bin/env python3

from copy import deepcopy
import importlib.util
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
import yaml

try:
    from waypoint_audit import FREE, MapAuditGrid, OCCUPIED, UNKNOWN
except ModuleNotFoundError:
    _AUDIT_PATH = Path(__file__).resolve().with_name('waypoint_audit.py')
    _AUDIT_SPEC = importlib.util.spec_from_file_location('waypoint_audit', _AUDIT_PATH)
    _AUDIT_MODULE = importlib.util.module_from_spec(_AUDIT_SPEC)
    _AUDIT_SPEC.loader.exec_module(_AUDIT_MODULE)
    FREE = _AUDIT_MODULE.FREE
    OCCUPIED = _AUDIT_MODULE.OCCUPIED
    UNKNOWN = _AUDIT_MODULE.UNKNOWN
    MapAuditGrid = _AUDIT_MODULE.MapAuditGrid


def quaternion_to_yaw(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def round_list(values, decimals: int):
    return [round(float(value), decimals) for value in values]


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')
        self._declare_parameters()

        self.waypoints_path = Path(self.get_parameter('waypoints_file').value).expanduser()
        self.labels = self._parse_labels(self.get_parameter('labels_csv').value)
        self.input_mode = str(self.get_parameter('input_mode').value).strip().lower()
        self.default_yaw = float(self.get_parameter('default_yaw').value)
        self.round_decimals = max(0, int(self.get_parameter('round_decimals').value))
        self.shutdown_on_complete = bool(self.get_parameter('shutdown_on_complete').value)
        self.write_backup = bool(self.get_parameter('write_backup').value)
        self.update_frame_id = bool(self.get_parameter('update_frame_id').value)
        self.audit_map_yaml = str(self.get_parameter('audit_map_yaml').value).strip()
        self.audit_route_name = str(self.get_parameter('audit_route_name').value).strip()
        self.audit_allow_unknown = bool(self.get_parameter('audit_allow_unknown').value)
        self.audit_connectivity = max(4, int(self.get_parameter('audit_connectivity').value))
        self.audit_node_budget = max(1000, int(self.get_parameter('audit_node_budget').value))

        self._load_waypoint_file()
        self._load_audit_map()
        self._backup_written = False
        self._pending_index = 0

        self.pose_topic = str(self.get_parameter('goal_pose_topic').value)
        self.point_topic = str(self.get_parameter('clicked_point_topic').value)

        self.pose_sub = None
        self.point_sub = None
        if self.input_mode in ('goal_pose', 'auto'):
            self.pose_sub = self.create_subscription(
                PoseStamped, self.pose_topic, self._on_pose, 10
            )
        if self.input_mode in ('clicked_point', 'auto'):
            self.point_sub = self.create_subscription(
                PointStamped, self.point_topic, self._on_point, 10
            )

        self._log_startup()

    def _declare_parameters(self):
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('waypoints_file', '')
        self.declare_parameter('labels_csv', 'P,A,B,C')
        self.declare_parameter('input_mode', 'goal_pose')
        self.declare_parameter('goal_pose_topic', '/waypoint_goal_pose')
        self.declare_parameter('clicked_point_topic', '/waypoint_clicked_point')
        self.declare_parameter('default_yaw', 0.0)
        self.declare_parameter('round_decimals', 4)
        self.declare_parameter('shutdown_on_complete', False)
        self.declare_parameter('write_backup', True)
        self.declare_parameter('update_frame_id', True)
        self.declare_parameter('audit_map_yaml', '')
        self.declare_parameter('audit_route_name', '')
        self.declare_parameter('audit_allow_unknown', False)
        self.declare_parameter('audit_connectivity', 8)
        self.declare_parameter('audit_node_budget', 400000)

    def _parse_labels(self, raw: str):
        labels = [item.strip() for item in raw.split(',') if item.strip()]
        if not labels:
            raise ValueError('labels_csv must contain at least one label')
        return labels

    def _load_waypoint_file(self):
        if self.waypoints_path.exists():
            self.data = yaml.safe_load(self.waypoints_path.read_text(encoding='utf-8')) or {}
        else:
            self.data = {}

        navigation_waypoints = self.data.setdefault('navigation_waypoints', {})
        params = navigation_waypoints.setdefault('ros__parameters', {})
        params.setdefault('frame_id', 'map')
        params.setdefault('points', {})
        params.setdefault('routes', {
            'default': deepcopy(self.labels),
        })
        self.params = params
        self.points = self.params['points']

    def _load_audit_map(self):
        self.audit_grid = None
        if not self.audit_map_yaml:
            return

        audit_map_path = Path(self.audit_map_yaml).expanduser()
        if not audit_map_path.is_file():
            self.get_logger().warn(f'Audit map not found, disabled map feedback: {audit_map_path}')
            return

        try:
            self.audit_grid = MapAuditGrid(audit_map_path)
        except Exception as exc:  # pragma: no cover - defensive logging path
            self.get_logger().warn(f'Failed to load audit map {audit_map_path}: {exc}')
            self.audit_grid = None

    def _log_startup(self):
        self.get_logger().info(f'Waypoint file: {self.waypoints_path}')
        self.get_logger().info(f'Labels to capture: {", ".join(self.labels)}')
        self.get_logger().info(f'Input mode: {self.input_mode}')
        if self.input_mode in ('goal_pose', 'auto'):
            self.get_logger().info(f'Listening pose topic: {self.pose_topic}')
        if self.input_mode in ('clicked_point', 'auto'):
            self.get_logger().info(f'Listening point topic: {self.point_topic}')
        self.get_logger().info(
            'Use RViz GoalTool for pose capture or PublishPoint for point capture, '
            'depending on input_mode.'
        )
        if self.audit_grid is not None:
            self.get_logger().info(
                f'Audit map loaded: {self.audit_grid.map_yaml_path} '
                f'({self.audit_grid.width}x{self.audit_grid.height}, '
                f'{self.audit_grid.resolution:.4f} m/cell)'
            )
            if self.audit_route_name:
                self.get_logger().info(f'Audit route target: {self.audit_route_name}')
        self.get_logger().info(f'Next label: {self._next_label()}')

    def _next_label(self):
        if self._pending_index >= len(self.labels):
            return '(complete)'
        return self.labels[self._pending_index]

    def _ensure_backup(self):
        if not self.write_backup or self._backup_written:
            return
        if not self.waypoints_path.exists():
            self._backup_written = True
            return
        backup_path = self.waypoints_path.with_suffix(self.waypoints_path.suffix + '.bak')
        backup_path.write_text(self.waypoints_path.read_text(encoding='utf-8'), encoding='utf-8')
        self._backup_written = True
        self.get_logger().info(f'Backup written to {backup_path}')

    def _save(self):
        self._ensure_backup()
        self.waypoints_path.parent.mkdir(parents=True, exist_ok=True)
        yaml_text = yaml.safe_dump(
            self.data,
            sort_keys=False,
            allow_unicode=False,
            default_flow_style=False,
        )
        self.waypoints_path.write_text(yaml_text, encoding='utf-8')

    def _point_is_traversable(self, state: int):
        return state == FREE or (state == UNKNOWN and self.audit_allow_unknown)

    def _audit_point(self, label: str):
        if self.audit_grid is None:
            return
        if self.params.get('frame_id', 'map') != 'map':
            self.get_logger().warn(
                f'Point {label} recorded in frame "{self.params.get("frame_id")}", '
                'static-map audit assumes frame_id=map.'
            )
            return

        values = self.points.get(label)
        if not isinstance(values, (list, tuple)) or len(values) < 3:
            self.get_logger().warn(f'Point {label} has invalid format, cannot audit.')
            return

        x = float(values[0])
        y = float(values[1])
        cell = self.audit_grid.world_to_cell(x, y)
        if cell is None:
            self.get_logger().error(
                f'Point {label} = ({x:.3f}, {y:.3f}) is outside audit map bounds.'
            )
            return

        state = self.audit_grid.state_at_cell(cell[0], cell[1])
        state_name = self.audit_grid.state_name(state)
        if state == FREE:
            self.get_logger().info(
                f'Point {label} is on FREE space: cell={cell}, world=({x:.3f}, {y:.3f})'
            )
        elif state == UNKNOWN and self.audit_allow_unknown:
            self.get_logger().warn(
                f'Point {label} is on UNKNOWN space but allow_unknown=true: cell={cell}, '
                f'world=({x:.3f}, {y:.3f})'
            )
        else:
            self.get_logger().error(
                f'Point {label} is on {state_name.upper()} space: cell={cell}, '
                f'world=({x:.3f}, {y:.3f})'
            )

    def _audit_route(self):
        if self.audit_grid is None or not self.audit_route_name:
            return

        routes = self.params.get('routes', {})
        route = routes.get(self.audit_route_name)
        if route is None:
            self.get_logger().warn(f'Route "{self.audit_route_name}" not found; skip route audit.')
            return

        missing = [name for name in route if name not in self.points]
        if missing:
            self.get_logger().info(
                f'Route "{self.audit_route_name}" still missing points: {missing}'
            )
            return

        point_cells = {}
        bad_points = []
        for name in route:
            values = self.points[name]
            cell = self.audit_grid.world_to_cell(float(values[0]), float(values[1]))
            if cell is None:
                bad_points.append(f'{name}(out_of_bounds)')
                continue
            state = self.audit_grid.state_at_cell(cell[0], cell[1])
            point_cells[name] = cell
            if not self._point_is_traversable(state):
                bad_points.append(f'{name}({self.audit_grid.state_name(state)})')

        if bad_points:
            self.get_logger().warn(
                f'Route "{self.audit_route_name}" has non-traversable points: {bad_points}'
            )
            return

        total = 0.0
        for start_name, goal_name in zip(route, route[1:]):
            path_length, expansions = self.audit_grid.shortest_path_length(
                point_cells[start_name],
                point_cells[goal_name],
                connectivity=self.audit_connectivity,
                allow_unknown=self.audit_allow_unknown,
                node_budget=self.audit_node_budget,
            )
            if path_length is None:
                self.get_logger().warn(
                    f'Route "{self.audit_route_name}" segment {start_name}->{goal_name} '
                    f'has no traversable path (expanded {expansions} cells).'
                )
                return
            total += path_length

        self.get_logger().info(
            f'Route "{self.audit_route_name}" is currently traversable on the audit map; '
            f'total path length {total:.3f} m'
        )

    def _capture(self, frame_id: str, x: float, y: float, yaw: float, source: str):
        if self._pending_index >= len(self.labels):
            self.get_logger().warn('All requested labels have already been captured; ignoring extra input.')
            return

        label = self.labels[self._pending_index]
        if self.update_frame_id and frame_id:
            self.params['frame_id'] = frame_id

        values = round_list([x, y, yaw], self.round_decimals)
        self.points[label] = values
        self._save()
        self.get_logger().info(f'Captured {label} = {values} from {source}')
        self._audit_point(label)
        self._audit_route()

        self._pending_index += 1
        if self._pending_index >= len(self.labels):
            self.get_logger().info('All requested labels captured.')
            if self.shutdown_on_complete:
                self.get_logger().info('shutdown_on_complete=true, stopping recorder.')
                rclpy.shutdown()
        else:
            self.get_logger().info(f'Next label: {self._next_label()}')

    def _on_pose(self, msg: PoseStamped):
        yaw = quaternion_to_yaw(msg.pose.orientation.z, msg.pose.orientation.w)
        self._capture(
            msg.header.frame_id,
            msg.pose.position.x,
            msg.pose.position.y,
            yaw,
            source='goal_pose',
        )

    def _on_point(self, msg: PointStamped):
        label = self._next_label()
        existing = self.points.get(label, [msg.point.x, msg.point.y, self.default_yaw])
        yaw = float(existing[2]) if isinstance(existing, (list, tuple)) and len(existing) >= 3 else self.default_yaw
        self._capture(
            msg.header.frame_id,
            msg.point.x,
            msg.point.y,
            yaw,
            source='clicked_point',
        )


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
