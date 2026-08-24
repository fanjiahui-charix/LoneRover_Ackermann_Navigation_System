#!/usr/bin/env python3
"""Generate a static Reverse->Tube-handoff reachability mask from Nav2 Smac.

This script is intended to run on the vehicle's ROS 2 environment with the
planner server active and the cone detector disabled.  It never publishes a
velocity command.  Each sampled start pose is sent to ComputePathToPose with
use_start=true; the first successful plan to any of the tolerant Gate goal
    yaws marks the state feasible and its path is stored for audit/replay.
    The handoff is a small region represented by nine concrete Smac goals;
    the runtime contract can accept any one of them.
"""

import argparse
import base64
import json
import math
import os
import struct
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap
from rclpy.action import ActionClient
from rclpy.node import Node


def quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def pose(x, y, yaw, stamp):
    msg = PoseStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = stamp
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = quat(yaw)[2]
    msg.pose.orientation.w = quat(yaw)[3]
    return msg


def grid_values(minimum, maximum, step):
    values = []
    value = minimum
    while value <= maximum + 1.0e-9:
        values.append(round(value, 6))
        value += step
    if not values or abs(values[-1] - maximum) > 1.0e-6:
        values.append(round(maximum, 6))
    return values


class Generator(Node):
    def __init__(self, args):
        super().__init__('reverse_gate_lut_generator')
        self.args = args
        self.client = ActionClient(self, ComputePathToPose,
                                   '/compute_path_to_pose')
        self.total = 0
        self.feasible = 0
        self.timeouts = 0
        self.failures = 0
        self.prefiltered_starts = 0
        self.entries = []
        self.global_costmap = None
        self.audit_costmap = None
        self.path_audit_rejections = {}
        self.costmap_sub = self.create_subscription(
            Costmap, '/global_costmap/costmap_raw', self._costmap_cb, 10)
        raw_goal_specs = [
            (goal_x, goal_y, goal_yaw_deg)
            for goal_x in self.args.goal_xs
            for goal_y in self.args.goal_ys
            for goal_yaw_deg in self.args.goal_yaws
        ]
        # Try the geometric center first so normal states usually need one
        # Smac request; retain all goals as deterministic fallbacks.
        self.goal_specs = sorted(
            raw_goal_specs,
            key=lambda item: (
                abs(item[0] - 2.50) + abs(item[1] - 2.50),
                abs(item[2] - 90),
                item[1], item[2]),
        )

    def _costmap_cb(self, msg):
        if msg.metadata.size_x > 0 and msg.metadata.size_y > 0 and msg.data:
            self.global_costmap = msg

    def _cost_at(self, x, y):
        grid = self.audit_costmap or self.global_costmap
        if grid is None:
            return 255
        resolution = grid.metadata.resolution
        ox = grid.metadata.origin.position.x
        oy = grid.metadata.origin.position.y
        mx = int(math.floor((x - ox) / resolution))
        my = int(math.floor((y - oy) / resolution))
        if mx < 0 or my < 0 or mx >= grid.metadata.size_x or my >= grid.metadata.size_y:
            return 254
        index = my * grid.metadata.size_x + mx
        return grid.data[index]

    @staticmethod
    def _angle_delta(a, b):
        return math.atan2(math.sin(b - a), math.cos(b - a))

    def _footprint_samples(self, front, rear, half, step):
        samples = []
        nx = int(math.ceil((front + rear) / step))
        ny = int(math.ceil((2.0 * half) / step))
        for ix in range(nx + 1):
            local_x = -rear + min(ix * step, front + rear)
            for iy in range(ny + 1):
                local_y = -half + min(iy * step, 2.0 * half)
                samples.append((local_x, local_y))
        return samples

    def _path_hard_safe(self, points):
        """Audit a Smac witness against the frozen global costmap.

        The planner already enforces its configured Hybrid-A* model.  This
        pass is intentionally limited to the navigation hard-safety contract:
        forward motion, map/Keepout occupancy, and a small extra footprint
        margin so states that merely graze a wall are not installed in the
        runtime LUT.  Geometric curvature from sparse Smac poses is reported
        but is not a second hard rejection.
        """
        if len(points) < 2:
            return False, 'path_too_short', 0.0, 0.0, 0
        front = 0.28 + self.args.footprint_padding_m + self.args.clearance_margin_m
        rear = 0.11 + self.args.footprint_padding_m + self.args.clearance_margin_m
        half = 0.13 + self.args.footprint_padding_m + self.args.clearance_margin_m
        footprint = self._footprint_samples(front, rear, half, self.args.footprint_step_m)
        total_length = 0.0
        max_curvature = 0.0
        max_footprint_cost = 0
        start_x, start_y = points[0][0], points[0][1]
        goal_x, goal_y = points[-1][0], points[-1][1]
        direct_length = math.hypot(goal_x - start_x, goal_y - start_y)
        if direct_length < 1.0e-6:
            return False, 'zero_goal_displacement', 0.0, 0.0, 0
        ux = (goal_x - start_x) / direct_length
        uy = (goal_y - start_y) / direct_length
        projected = [
            (x - start_x) * ux + (y - start_y) * uy
            for x, y, _ in points
        ]
        goal_backtrack_m = sum(
            max(0.0, a - b) for a, b in zip(projected, projected[1:]))
        for a, b in zip(points, points[1:]):
            dx = b[0] - a[0]
            dy = b[1] - a[1]
            segment = math.hypot(dx, dy)
            if segment < 1.0e-6:
                continue
            projection = dx * math.cos(a[2]) + dy * math.sin(a[2])
            if projection < -0.01:
                return (False, 'reverse_segment', total_length,
                        max_curvature, max_footprint_cost)
            total_length += segment
            count = max(1, int(math.ceil(segment / self.args.path_step_m)))
            for sample_index in range(count + 1):
                ratio = sample_index / count
                yaw = a[2] + ratio * self._angle_delta(a[2], b[2])
                x = a[0] + ratio * dx
                y = a[1] + ratio * dy
                c, s = math.cos(yaw), math.sin(yaw)
                for local_x, local_y in footprint:
                    px = x + c * local_x - s * local_y
                    py = y + s * local_x + c * local_y
                    value = self._cost_at(px, py)
                    max_footprint_cost = max(max_footprint_cost, value)
                    if value >= self.args.lethal_cost:
                        return (False, 'footprint_or_keepout_collision',
                                total_length, max_curvature,
                                max_footprint_cost)
        if total_length / direct_length > self.args.max_detour_ratio:
            return (False, 'path_detour_too_long', total_length,
                    max_curvature, max_footprint_cost)
        if goal_backtrack_m > self.args.max_goal_backtrack_m:
            return (False, 'path_backtracks_from_goal', total_length,
                    max_curvature, max_footprint_cost)
        for a, b, c in zip(points, points[1:], points[2:]):
            ab = math.hypot(b[0] - a[0], b[1] - a[1])
            bc = math.hypot(c[0] - b[0], c[1] - b[1])
            ac = math.hypot(c[0] - a[0], c[1] - a[1])
            if min(ab, bc, ac) < 1.0e-5:
                continue
            cross = abs((b[0] - a[0]) * (c[1] - a[1]) -
                        (b[1] - a[1]) * (c[0] - a[0]))
            max_curvature = max(max_curvature, 2.0 * cross / (ab * bc * ac))
        return True, '', total_length, max_curvature, max_footprint_cost

    def _pose_footprint_free(self, x, y, yaw):
        # Match the vehicle footprint passed to the car's global costmap.
        # This is an inexpensive prefilter only; Smac remains authoritative.
        half = 0.13
        front = 0.28
        rear = 0.11
        samples = [
            (0.0, 0.0), (front, half), (front, -half),
            (-rear, -half), (-rear, half),
            (front, 0.0), (-rear, 0.0), (0.0, half), (0.0, -half),
        ]
        c, s = math.cos(yaw), math.sin(yaw)
        for local_x, local_y in samples:
            px = x + c * local_x - s * local_y
            py = y + s * local_x + c * local_y
            value = self._cost_at(px, py)
            if 253 <= value < 255:
                return False
        return True

    def _start_footprint_free(self, x, y, yaw):
        return self._pose_footprint_free(x, y, yaw)

    def _save_checkpoint(self, path, next_index, state_count, mask,
                         goal_ids, path_lengths):
        payload = {
            'version': 1,
            'next_index': next_index,
            'state_count': state_count,
            'grid': {
                'xmin': self.args.xmin, 'xmax': self.args.xmax,
                'ymin': self.args.ymin, 'ymax': self.args.ymax,
                'xy_step': self.args.xy_step,
                'yaw_min': self.args.yaw_min, 'yaw_max': self.args.yaw_max,
                'yaw_step': self.args.yaw_step,
            },
            'mask_b64': base64.b64encode(bytes(mask)).decode('ascii'),
            'goal_ids_b64': base64.b64encode(bytes(goal_ids)).decode('ascii'),
            'path_lengths': path_lengths,
            'total': self.total,
            'feasible': self.feasible,
            'timeouts': self.timeouts,
            'failures': self.failures,
            'prefiltered_starts': self.prefiltered_starts,
            'path_audit_rejections': self.path_audit_rejections,
        }
        temporary = Path(str(path) + '.tmp')
        temporary.write_text(json.dumps(payload, separators=(',', ':')),
                             encoding='utf-8')
        os.replace(temporary, path)

    def _load_checkpoint(self, path, state_count, mask, goal_ids,
                         path_lengths):
        payload = json.loads(path.read_text(encoding='utf-8'))
        if payload.get('version') != 1 or payload.get('state_count') != state_count:
            raise RuntimeError('reverse LUT checkpoint grid/state count mismatch')
        expected_grid = {
            'xmin': self.args.xmin, 'xmax': self.args.xmax,
            'ymin': self.args.ymin, 'ymax': self.args.ymax,
            'xy_step': self.args.xy_step,
            'yaw_min': self.args.yaw_min, 'yaw_max': self.args.yaw_max,
            'yaw_step': self.args.yaw_step,
        }
        if payload.get('grid') != expected_grid:
            raise RuntimeError('reverse LUT checkpoint grid parameters mismatch')
        saved_mask = base64.b64decode(payload['mask_b64'])
        saved_goal_ids = base64.b64decode(payload['goal_ids_b64'])
        if len(saved_mask) != len(mask) or len(saved_goal_ids) != len(goal_ids):
            raise RuntimeError('reverse LUT checkpoint array size mismatch')
        mask[:] = saved_mask
        goal_ids[:] = saved_goal_ids
        path_lengths[:] = payload['path_lengths']
        self.total = int(payload.get('total', 0))
        self.feasible = int(payload.get('feasible', 0))
        self.timeouts = int(payload.get('timeouts', 0))
        self.failures = int(payload.get('failures', 0))
        self.prefiltered_starts = int(payload.get('prefiltered_starts', 0))
        self.path_audit_rejections = dict(
            payload.get('path_audit_rejections', {}))
        return int(payload['next_index'])

    def query(self, x, y, yaw):
        stamp = self.get_clock().now().to_msg()
        start = pose(x, y, yaw, stamp)
        safe_results = []
        # A Gate entry is a region, not one brittle terminal pose.  Try a
        # small set of points inside the official CHANNEL_GATE_IN rectangle
        # and the permitted heading range; low-cost witnesses stop early,
        # while near-obstacle witnesses are compared against other goals.
        for goal_id, (goal_x, goal_y, goal_yaw_deg) in enumerate(
                self.goal_specs):
            if not self._pose_footprint_free(
                    goal_x, goal_y, math.radians(goal_yaw_deg)):
                self.path_audit_rejections['goal_footprint_prefilter'] = \
                    self.path_audit_rejections.get(
                        'goal_footprint_prefilter', 0) + 1
                continue
            goal = pose(goal_x, goal_y, math.radians(goal_yaw_deg), stamp)
            goal_msg = ComputePathToPose.Goal()
            goal_msg.start = start
            goal_msg.goal = goal
            goal_msg.use_start = True
            goal_msg.planner_id = 'GridBased'
            send_future = self.client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self, send_future, timeout_sec=self.args.timeout)
            if not send_future.done():
                self.timeouts += 1
                continue
            handle = send_future.result()
            if handle is None or not handle.accepted:
                continue
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, result_future, timeout_sec=self.args.timeout)
            if not result_future.done():
                self.timeouts += 1
                cancel_future = handle.cancel_goal_async()
                rclpy.spin_until_future_complete(
                    self, cancel_future,
                    timeout_sec=min(0.25, self.args.timeout))
                continue
            wrapped = result_future.result()
            if wrapped is None or wrapped.result is None:
                self.timeouts += 1
                continue
            result = wrapped.result
            path = result.path
            if len(path.poses) < 2:
                self.failures += 1
                continue
            # ComputePathToPose is the authoritative feasibility test. Keep
            # every accepted path long enough to choose the best-clearance
            # witness; runtime will still audit live costmaps before use.
            points = []
            for p in path.poses:
                q = p.pose.orientation
                pyaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                points.append((p.pose.position.x, p.pose.position.y, pyaw))
            safe, reason, path_length, max_curvature, max_cost = \
                self._path_hard_safe(points)
            if not safe:
                self.path_audit_rejections[reason] = \
                    self.path_audit_rejections.get(reason, 0) + 1
                continue
            safe_results.append({
                'goal_id': goal_id,
                'goal_x': goal_x, 'goal_y': goal_y,
                'goal_yaw_deg': goal_yaw_deg,
                'path_length_m': path_length, 'max_geometric_curvature_per_m': max_curvature,
                'max_footprint_cost': max_cost,
                'path_hard_safe': True, 'clearance_margin_m': self.args.clearance_margin_m,
                'path': points,
            })
            # A low-cost path already has comfortable static clearance. Do not
            # spend 62 more Smac calls searching for a marginally shorter one.
            if max_cost <= self.args.preferred_max_footprint_cost:
                break
        if not safe_results:
            return None
        safe_results.sort(key=lambda result: (
            result['max_footprint_cost'], result['path_length_m']))
        return safe_results[0]

    def run(self):
        if not self.client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError('compute_path_to_pose action server unavailable')
        costmap_deadline = time.monotonic() + 30.0
        while self.global_costmap is None and time.monotonic() < costmap_deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.global_costmap is None:
            raise RuntimeError('global costmap snapshot unavailable')
        # Freeze one real vehicle costmap snapshot for the complete offline
        # audit.  The action server remains responsible for the Smac query;
        # this pass prevents a later rolling update from changing the meaning
        # of already accepted witnesses.
        self.audit_costmap = self.global_costmap
        xs = grid_values(self.args.xmin, self.args.xmax, self.args.xy_step)
        ys = grid_values(self.args.ymin, self.args.ymax, self.args.xy_step)
        yaws = list(range(self.args.yaw_min, self.args.yaw_max + 1,
                          self.args.yaw_step))
        # The ordering is only a generation detail.  It is deliberately not
        # tied to one measured task pose: scan the geometric center of the
        # requested hall region first, while keeping the serialized index
        # equal to the regular lattice index.
        center_x = 0.5 * (self.args.xmin + self.args.xmax)
        center_y = 0.5 * (self.args.ymin + self.args.ymax)
        center_yaw = math.radians(0.5 * (self.args.yaw_min + self.args.yaw_max))
        states = []
        for iyaw, yaw_deg in enumerate(yaws):
            for iy, y in enumerate(ys):
                for ix, x in enumerate(xs):
                    dyaw = math.atan2(
                        math.sin(math.radians(yaw_deg) - center_yaw),
                        math.cos(math.radians(yaw_deg) - center_yaw))
                    states.append((math.hypot(x - center_x, y - center_y) +
                                   0.20 * abs(dyaw),
                                   iyaw, iy, ix, x, y, yaw_deg))
        states.sort(key=lambda item: item[0])
        self.args.out.parent.mkdir(parents=True, exist_ok=True)
        self.args.paths.parent.mkdir(parents=True, exist_ok=True)
        self.args.goal_ids.parent.mkdir(parents=True, exist_ok=True)
        self.args.path_lengths.parent.mkdir(parents=True, exist_ok=True)
        checkpoint = self.args.checkpoint
        if checkpoint is None:
            checkpoint = Path(str(self.args.out) + '.checkpoint.json')
        if not self.args.resume:
            self.args.paths.unlink(missing_ok=True)
            checkpoint.unlink(missing_ok=True)
        mask = bytearray((len(xs) * len(ys) * len(yaws) + 7) // 8)
        state_count = len(xs) * len(ys) * len(yaws)
        goal_ids = bytearray([255] * state_count)
        path_lengths = [float('inf')] * state_count
        start_index = 0
        if self.args.resume and checkpoint.exists():
            start_index = self._load_checkpoint(
                checkpoint, state_count, mask, goal_ids, path_lengths)
            self.get_logger().info(
                f'resuming checkpoint next_index={start_index}/{state_count}')
        elif self.args.resume:
            self.get_logger().warning(
                'resume requested but checkpoint is absent; starting at index 0')
        started = time.monotonic()
        for state_index, (_, iyaw, iy, ix, x, y, yaw_deg) in enumerate(states):
            if state_index < start_index:
                continue
            self.total += 1
            if not self._start_footprint_free(x, y, math.radians(yaw_deg)):
                self.prefiltered_starts += 1
                if (state_index + 1) % self.args.checkpoint_interval == 0:
                    self._save_checkpoint(
                        checkpoint, state_index + 1, state_count, mask,
                        goal_ids, path_lengths)
                continue
            result = self.query(x, y, math.radians(yaw_deg))
            if result is not None:
                linear = (iyaw * len(ys) + iy) * len(xs) + ix
                mask[linear >> 3] |= 1 << (linear & 7)
                goal_ids[linear] = result['goal_id']
                path_lengths[linear] = result['path_length_m']
                self.feasible += 1
                with self.args.paths.open('a', encoding='utf-8') as stream:
                    stream.write(json.dumps({
                        'ix': ix, 'iy': iy, 'iyaw': iyaw,
                        'x': x, 'y': y, 'yaw_deg': yaw_deg,
                        'goal_id': result['goal_id'],
                        'goal_x': result['goal_x'],
                        'goal_y': result['goal_y'],
                        'goal_yaw_deg': result['goal_yaw_deg'],
                        'path_length_m': result['path_length_m'],
                        'max_footprint_cost': result['max_footprint_cost'],
                        'max_geometric_curvature_per_m': result['max_geometric_curvature_per_m'],
                        'path_hard_safe': result['path_hard_safe'],
                        'clearance_margin_m': result['clearance_margin_m'],
                        'path': result['path'],
                    }, separators=(',', ':')) + '\n')
            if (state_index + 1) % self.args.checkpoint_interval == 0:
                self._save_checkpoint(
                    checkpoint, state_index + 1, state_count, mask,
                    goal_ids, path_lengths)
            if self.total % 100 == 0:
                elapsed = time.monotonic() - started
                self.get_logger().info(
                    f'progress={self.total}/{len(xs) * len(ys) * len(yaws)} '
                    f'feasible={self.feasible} elapsed={elapsed:.1f}s')
        self._save_checkpoint(
            checkpoint, state_count, state_count, mask, goal_ids, path_lengths)
        header = {
            'version': 1,
            'frame': 'map',
            'x_min_m': self.args.xmin,
            'x_max_m': self.args.xmax,
            'y_min_m': self.args.ymin,
            'y_max_m': self.args.ymax,
            'xy_resolution_m': self.args.xy_step,
            'yaw_min_deg': self.args.yaw_min,
            'yaw_max_deg': self.args.yaw_max,
            'yaw_resolution_deg': self.args.yaw_step,
            'size_x': len(xs), 'size_y': len(ys), 'yaw_bins': len(yaws),
            'gate_goal_xs_m': self.args.goal_xs,
            'gate_goal_ys_m': self.args.goal_ys,
            'gate_goal_yaws_deg': self.args.goal_yaws,
            'gate_goal_specs': [
                {'goal_id': i, 'x_m': x, 'y_m': y, 'yaw_deg': yaw}
                for i, (x, y, yaw) in enumerate(self.goal_specs)],
            'handoff_region': {
                'x_min_m': min(self.args.goal_xs),
                'x_max_m': max(self.args.goal_xs),
                'y_min_m': min(self.args.goal_ys),
                'y_max_m': max(self.args.goal_ys),
                'yaw_min_deg': min(self.args.goal_yaws),
                'yaw_max_deg': max(self.args.goal_yaws),
                'semantic': 'Tube common forward handoff region',
            },
            'minimum_turning_radius_m': 0.35,
            'planner_id': 'GridBased',
            'motion_model': 'SmacPlannerHybrid/Dubins',
            'cone_input': 'disabled',
            'queries': self.total,
            'feasible': self.feasible,
            'timeouts': self.timeouts,
            'failures': self.failures,
            'prefiltered_lethal_starts': self.prefiltered_starts,
            'global_costmap_frame': self.global_costmap.header.frame_id,
            'global_costmap_resolution_m': self.global_costmap.metadata.resolution,
            'path_audit_rejections': self.path_audit_rejections,
            'path_audit': {
                'hard_lethal_cost': self.args.lethal_cost,
                'footprint_padding_m': self.args.footprint_padding_m,
                'clearance_margin_m': self.args.clearance_margin_m,
                'footprint_step_m': self.args.footprint_step_m,
                'path_step_m': self.args.path_step_m,
                'max_detour_ratio': self.args.max_detour_ratio,
                'max_goal_backtrack_m': self.args.max_goal_backtrack_m,
                'preferred_max_footprint_cost':
                    self.args.preferred_max_footprint_cost,
                'curvature': 'diagnostic_only',
            },
        }
        with self.args.meta.open('w', encoding='utf-8') as stream:
            json.dump(header, stream, indent=2)
        with self.args.out.open('wb') as stream:
            # RGE2 is self-describing so the runtime cannot silently query a
            # mask with the wrong lattice.  The header is little-endian:
            # magic[4], size_x/y/yaw_bins/mask_len, x_min/y_min/xy_resolution,
            # yaw_min_deg/yaw_step_deg, followed by the bit mask.
            stream.write(b'RGE2')
            stream.write(struct.pack(
                '<IIIIdddii', len(xs), len(ys), len(yaws), len(mask),
                self.args.xmin, self.args.ymin, self.args.xy_step,
                self.args.yaw_min, self.args.yaw_step))
            stream.write(mask)
        with self.args.goal_ids.open('wb') as stream:
            # Goal table precedes state ids so post-Reverse can recover the
            # concrete handoff pose selected by the offline Smac witness.
            stream.write(b'RGEG2')
            stream.write(struct.pack('<II', state_count, len(self.goal_specs)))
            for goal_x, goal_y, goal_yaw_deg in self.goal_specs:
                stream.write(struct.pack('<ddd', goal_x, goal_y, goal_yaw_deg))
            stream.write(goal_ids)
        with self.args.path_lengths.open('wb') as stream:
            stream.write(b'RGEL1')
            stream.write(struct.pack('<I', state_count))
            stream.write(b''.join(struct.pack('<f', value)
                                   for value in path_lengths))
        self.get_logger().info(
            f'done queries={self.total} feasible={self.feasible} '
            f'mask={self.args.out} goal_ids={self.args.goal_ids} '
            f'path_lengths={self.args.path_lengths} paths={self.args.paths}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xmin', type=float, default=2.00)
    parser.add_argument('--xmax', type=float, default=3.00)
    parser.add_argument('--ymin', type=float, default=0.30)
    parser.add_argument('--ymax', type=float, default=2.00)
    parser.add_argument('--xy-step', type=float, default=0.10)
    parser.add_argument('--yaw-min', type=int, default=0)
    parser.add_argument('--yaw-max', type=int, default=90)
    parser.add_argument('--yaw-step', type=int, default=5)
    parser.add_argument('--goal-xs', type=float, nargs='+', default=[2.50])
    # Tube/Gate handoff region: concrete Smac goals are only an offline
    # representation of this tolerant region.  Runtime lookup uses the two
    # compact binary tables; it does not invoke Smac for every reverse probe.
    parser.add_argument(
        '--goal-ys', type=float, nargs='+',
        default=[2.30, 2.35, 2.40, 2.45, 2.50,
                 2.55, 2.60, 2.65, 2.70])
    parser.add_argument(
        '--goal-yaws', type=int, nargs='+',
        default=[75, 80, 85, 90, 95, 100, 105])
    parser.add_argument('--timeout', type=float, default=2.0)
    parser.add_argument('--lethal-cost', type=int, default=253)
    parser.add_argument('--footprint-padding-m', type=float, default=0.01)
    parser.add_argument('--clearance-margin-m', type=float, default=0.05)
    parser.add_argument('--footprint-step-m', type=float, default=0.025)
    parser.add_argument('--path-step-m', type=float, default=0.015)
    parser.add_argument('--max-detour-ratio', type=float, default=2.0)
    parser.add_argument('--max-goal-backtrack-m', type=float, default=0.30)
    parser.add_argument('--preferred-max-footprint-cost', type=int, default=200)
    parser.add_argument('--out', type=Path,
                        default=Path('/tmp/reverse_gate_entry.bin'))
    parser.add_argument('--paths', type=Path,
                        default=Path('/tmp/reverse_gate_entry_paths.jsonl'))
    parser.add_argument('--meta', type=Path,
                        default=Path('/tmp/reverse_gate_entry.json'))
    parser.add_argument('--goal-ids', type=Path,
                        default=Path('/tmp/reverse_gate_goal_id.bin'))
    parser.add_argument('--path-lengths', type=Path,
                        default=Path('/tmp/reverse_gate_path_length.bin'))
    parser.add_argument('--checkpoint', type=Path, default=None)
    parser.add_argument('--checkpoint-interval', type=int, default=5)
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = Generator(args)
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
