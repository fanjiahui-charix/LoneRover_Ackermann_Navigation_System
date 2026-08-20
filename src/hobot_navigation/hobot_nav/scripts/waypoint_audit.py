#!/usr/bin/env python3

import argparse
from heapq import heappop, heappush
import math
from pathlib import Path
import sys
from typing import Iterable

import yaml


SQRT2 = math.sqrt(2.0)
FREE = 0
OCCUPIED = 1
UNKNOWN = 2


def parse_bool(value: str) -> bool:
    return value.lower() in ('1', 'true', 'yes', 'on')


def candidate_share_dir() -> Path | None:
    script_path = Path(__file__).resolve()
    candidates = [
        script_path.parents[1],
        script_path.parents[2] / 'share' / 'hobot_nav',
    ]
    for candidate in candidates:
        if (candidate / 'waypoints' / 'example_waypoints.yaml').exists():
            return candidate
    return None


def default_waypoints_file() -> str:
    share_dir = candidate_share_dir()
    if share_dir is None:
        return 'example_waypoints.yaml'
    return str(share_dir / 'waypoints' / 'example_waypoints.yaml')


def default_map_yaml() -> str:
    share_dir = candidate_share_dir()
    if share_dir is None:
        return 'rdk_2026_hospital_static_1cm.yaml'
    return str(share_dir / 'maps' / 'rdk_2026_hospital_static_1cm.yaml')


def build_argument_parser():
    parser = argparse.ArgumentParser(
        description='Audit mission waypoints against a static map and route connectivity.'
    )
    parser.add_argument('--waypoints-file', default=default_waypoints_file())
    parser.add_argument('--map-yaml', default=default_map_yaml())
    parser.add_argument('--route-name', default='via_b')
    parser.add_argument('--check-all-routes', default='false')
    parser.add_argument('--allow-unknown', default='false')
    parser.add_argument('--connectivity', choices=['4', '8'], default='8')
    parser.add_argument('--path-timeout-nodes', default='400000')
    return parser


def load_waypoint_data(path: Path):
    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    params = data.get('navigation_waypoints', {}).get('ros__parameters', {})
    frame_id = params.get('frame_id', 'map')
    points = params.get('points', {}) or {}
    routes = params.get('routes', {}) or {}
    return frame_id, points, routes


def _read_pgm_tokens(binary: bytes):
    index = 0
    length = len(binary)

    def skip():
        nonlocal index
        while index < length:
            char = binary[index:index + 1]
            if char == b'#':
                while index < length and binary[index:index + 1] not in (b'\n', b'\r'):
                    index += 1
            elif char in b' \t\r\n':
                index += 1
            else:
                break

    def read_token():
        nonlocal index
        skip()
        start = index
        while index < length and binary[index:index + 1] not in b' \t\r\n#':
            index += 1
        return binary[start:index]

    magic = read_token().decode('ascii')
    width = int(read_token())
    height = int(read_token())
    max_value = int(read_token())
    skip()
    payload = binary[index:]
    return magic, width, height, max_value, payload


def load_pgm_image(path: Path):
    raw = path.read_bytes()
    magic, width, height, max_value, payload = _read_pgm_tokens(raw)
    if max_value > 255:
        raise ValueError(f'Unsupported PGM max value {max_value}, only 8-bit maps are supported')

    if magic == 'P5':
        expected = width * height
        if len(payload) < expected:
            raise ValueError(f'PGM payload too short: expected {expected} bytes, got {len(payload)}')
        pixels = payload[:expected]
    elif magic == 'P2':
        ascii_values = payload.decode('ascii').split()
        pixels = bytes(int(value) for value in ascii_values[:width * height])
        if len(pixels) < width * height:
            raise ValueError('PGM payload too short for ASCII image')
    else:
        raise ValueError(f'Unsupported image format {magic}, only P2/P5 PGM are supported')

    return width, height, pixels


class MapAuditGrid:
    def __init__(self, map_yaml_path: Path):
        config = yaml.safe_load(map_yaml_path.read_text(encoding='utf-8')) or {}
        image_path = Path(config['image'])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path

        self.map_yaml_path = map_yaml_path
        self.image_path = image_path
        self.resolution = float(config['resolution'])
        self.origin = config.get('origin', [0.0, 0.0, 0.0])
        self.negate = int(config.get('negate', 0))
        self.free_thresh = float(config.get('free_thresh', 0.25))
        self.occupied_thresh = float(config.get('occupied_thresh', 0.65))
        self.mode = str(config.get('mode', 'trinary'))

        self.width, self.height, image = load_pgm_image(image_path)
        self.states = bytearray(self.width * self.height)
        for idx, value in enumerate(image):
            occ = (float(value) / 255.0) if self.negate else ((255.0 - float(value)) / 255.0)
            if occ >= self.occupied_thresh:
                self.states[idx] = OCCUPIED
            elif occ <= self.free_thresh:
                self.states[idx] = FREE
            else:
                self.states[idx] = UNKNOWN

    def state_name(self, state: int) -> str:
        if state == FREE:
            return 'free'
        if state == OCCUPIED:
            return 'occupied'
        return 'unknown'

    def in_bounds_cell(self, col: int, row: int) -> bool:
        return 0 <= col < self.width and 0 <= row < self.height

    def world_to_cell(self, x: float, y: float):
        rel_x = (x - float(self.origin[0])) / self.resolution
        rel_y = (y - float(self.origin[1])) / self.resolution
        col = int(math.floor(rel_x))
        row_from_bottom = int(math.floor(rel_y))
        row = self.height - 1 - row_from_bottom
        if not self.in_bounds_cell(col, row):
            return None
        return col, row

    def cell_to_index(self, col: int, row: int) -> int:
        return row * self.width + col

    def cell_to_world_center(self, col: int, row: int):
        x = float(self.origin[0]) + (col + 0.5) * self.resolution
        row_from_bottom = self.height - 1 - row
        y = float(self.origin[1]) + (row_from_bottom + 0.5) * self.resolution
        return x, y

    def state_at_cell(self, col: int, row: int) -> int:
        return self.states[self.cell_to_index(col, row)]

    def neighbors(self, col: int, row: int, connectivity: int) -> Iterable[tuple[int, int, float]]:
        offsets = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]
        if connectivity == 8:
            offsets.extend([
                (-1, -1, SQRT2),
                (-1, 1, SQRT2),
                (1, -1, SQRT2),
                (1, 1, SQRT2),
            ])

        for dc, dr, cost in offsets:
            next_col = col + dc
            next_row = row + dr
            if self.in_bounds_cell(next_col, next_row):
                yield next_col, next_row, cost

    def heuristic(self, col: int, row: int, goal_col: int, goal_row: int, connectivity: int) -> float:
        dx = abs(goal_col - col)
        dy = abs(goal_row - row)
        if connectivity == 8:
            diagonal = min(dx, dy)
            straight = max(dx, dy) - diagonal
            return diagonal * SQRT2 + straight
        return float(dx + dy)

    def shortest_path_length(
        self,
        start_cell: tuple[int, int],
        goal_cell: tuple[int, int],
        connectivity: int,
        allow_unknown: bool,
        node_budget: int,
    ):
        if start_cell == goal_cell:
            return 0.0, 1

        start_col, start_row = start_cell
        goal_col, goal_row = goal_cell
        start_idx = self.cell_to_index(start_col, start_row)
        goal_idx = self.cell_to_index(goal_col, goal_row)

        open_heap = [(self.heuristic(start_col, start_row, goal_col, goal_row, connectivity), 0.0, start_idx)]
        best_cost = {start_idx: 0.0}
        expansions = 0

        while open_heap:
            _f_score, cost_so_far, current_idx = heappop(open_heap)
            if cost_so_far > best_cost.get(current_idx, float('inf')):
                continue

            if current_idx == goal_idx:
                return cost_so_far * self.resolution, expansions

            expansions += 1
            if expansions > node_budget:
                return None, expansions

            row = current_idx // self.width
            col = current_idx % self.width
            for next_col, next_row, step_cost in self.neighbors(col, row, connectivity):
                state = self.state_at_cell(next_col, next_row)
                if state == OCCUPIED:
                    continue
                if state == UNKNOWN and not allow_unknown:
                    continue

                next_idx = self.cell_to_index(next_col, next_row)
                next_cost = cost_so_far + step_cost
                if next_cost >= best_cost.get(next_idx, float('inf')):
                    continue

                best_cost[next_idx] = next_cost
                heuristic = self.heuristic(next_col, next_row, goal_col, goal_row, connectivity)
                heappush(open_heap, (next_cost + heuristic, next_cost, next_idx))

        return None, expansions


def format_status(kind: str, label: str, detail: str) -> str:
    return f'[{kind}] {label}: {detail}'


def main(argv=None):
    raw_args = list(argv if argv is not None else sys.argv[1:])
    args, _unknown = build_argument_parser().parse_known_args(raw_args)

    waypoints_path = Path(args.waypoints_file).expanduser()
    map_yaml_path = Path(args.map_yaml).expanduser()
    check_all_routes = parse_bool(args.check_all_routes)
    allow_unknown = parse_bool(args.allow_unknown)
    connectivity = int(args.connectivity)
    node_budget = max(1000, int(args.path_timeout_nodes))

    failures = 0
    warnings = 0
    lines = []

    if not waypoints_path.is_file():
        print(format_status('FAIL', 'waypoints', f'file not found: {waypoints_path}'))
        return 1
    if not map_yaml_path.is_file():
        print(format_status('FAIL', 'map', f'file not found: {map_yaml_path}'))
        return 1

    frame_id, points, routes = load_waypoint_data(waypoints_path)
    grid = MapAuditGrid(map_yaml_path)

    print(f'Waypoint audit')
    print(f'- waypoints: {waypoints_path}')
    print(f'- map: {map_yaml_path}')
    print(f'- image: {grid.image_path}')
    print(f'- frame_id: {frame_id}')
    print(f'- map size: {grid.width} x {grid.height}, resolution={grid.resolution:.4f} m/cell')

    if frame_id != 'map':
        warnings += 1
        lines.append(format_status('WARN', 'frame_id', f'waypoint frame is "{frame_id}", static map checks assume "map"'))
    else:
        lines.append(format_status('PASS', 'frame_id', 'waypoints are expressed in map frame'))

    point_cells = {}
    traversable_points = set()
    for name, values in points.items():
        if not isinstance(values, (list, tuple)) or len(values) < 3:
            failures += 1
            lines.append(format_status('FAIL', f'point {name}', 'expected [x, y, yaw]'))
            continue

        x = float(values[0])
        y = float(values[1])
        yaw = float(values[2])
        cell = grid.world_to_cell(x, y)
        if cell is None:
            failures += 1
            lines.append(format_status('FAIL', f'point {name}', f'({x:.3f}, {y:.3f}, {yaw:.3f}) is outside map bounds'))
            continue

        state = grid.state_at_cell(cell[0], cell[1])
        state_name = grid.state_name(state)
        detail = f'world=({x:.3f}, {y:.3f}, {yaw:.3f}) cell={cell} state={state_name}'
        point_cells[name] = cell
        if state == OCCUPIED:
            failures += 1
            lines.append(format_status('FAIL', f'point {name}', detail))
        elif state == UNKNOWN:
            if allow_unknown:
                traversable_points.add(name)
                warnings += 1
                lines.append(format_status('WARN', f'point {name}', detail))
            else:
                failures += 1
                lines.append(format_status('FAIL', f'point {name}', detail))
        else:
            traversable_points.add(name)
            lines.append(format_status('PASS', f'point {name}', detail))

    routes_to_check = routes.keys() if check_all_routes else [args.route_name]
    for route_name in routes_to_check:
        route = routes.get(route_name)
        if route is None:
            failures += 1
            lines.append(format_status('FAIL', f'route {route_name}', 'route not found'))
            continue

        missing = [name for name in route if name not in points]
        if missing:
            failures += 1
            lines.append(format_status('FAIL', f'route {route_name}', f'references missing points: {missing}'))
            continue

        lines.append(format_status('PASS', f'route {route_name}', ' -> '.join(route)))

        total_path_length = 0.0
        for start_name, goal_name in zip(route, route[1:]):
            if start_name not in point_cells or goal_name not in point_cells:
                failures += 1
                lines.append(
                    format_status(
                        'FAIL',
                        f'segment {start_name}->{goal_name}',
                        'cannot check path because one endpoint is invalid',
                    )
                )
                continue
            if start_name not in traversable_points or goal_name not in traversable_points:
                failures += 1
                lines.append(
                    format_status(
                        'FAIL',
                        f'segment {start_name}->{goal_name}',
                        'cannot check path because one endpoint is not traversable on the static map',
                    )
                )
                continue

            path_length, expansions = grid.shortest_path_length(
                point_cells[start_name],
                point_cells[goal_name],
                connectivity=connectivity,
                allow_unknown=allow_unknown,
                node_budget=node_budget,
            )

            if path_length is None:
                failures += 1
                lines.append(
                    format_status(
                        'FAIL',
                        f'segment {start_name}->{goal_name}',
                        f'no traversable path found within node budget {node_budget} (expanded {expansions} cells)',
                    )
                )
            else:
                total_path_length += path_length
                lines.append(
                    format_status(
                        'PASS',
                        f'segment {start_name}->{goal_name}',
                        f'path length {path_length:.3f} m (expanded {expansions} cells)',
                    )
                )

        lines.append(
            format_status(
                'INFO',
                f'route {route_name}',
                f'total traversable path length {total_path_length:.3f} m',
            )
        )

    for line in lines:
        print(line)

    if failures == 0 and warnings == 0:
        print('Summary: PASS')
        return 0
    if failures == 0:
        print(f'Summary: WARN ({warnings} warning(s))')
        return 0
    print(f'Summary: FAIL ({failures} issue(s), {warnings} warning(s))')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
