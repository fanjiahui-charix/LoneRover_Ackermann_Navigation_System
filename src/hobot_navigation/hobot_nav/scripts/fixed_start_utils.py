#!/usr/bin/env python3

import math
from pathlib import Path

import yaml


class FixedStartConfigError(ValueError):
    pass


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def compute_map_to_odom(map_base_pose, odom_base_pose):
    """Return SE(2) map->odom from map->base and odom->base poses."""
    map_x, map_y, map_yaw = map_base_pose
    odom_x, odom_y, odom_yaw = odom_base_pose
    yaw = normalize_angle(map_yaw - odom_yaw)
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x = map_x - (cos_yaw * odom_x - sin_yaw * odom_y)
    y = map_y - (sin_yaw * odom_x + cos_yaw * odom_y)
    return x, y, yaw


def mean_pose(samples):
    if not samples:
        raise ValueError('samples must not be empty')
    x = sum(sample[1] for sample in samples) / len(samples)
    y = sum(sample[2] for sample in samples) / len(samples)
    sin_sum = sum(math.sin(sample[3]) for sample in samples)
    cos_sum = sum(math.cos(sample[3]) for sample in samples)
    yaw = math.atan2(sin_sum, cos_sum)
    return x, y, yaw


def stable_pose_from_samples(
        samples, min_duration_sec, max_pose_drift_m, max_yaw_drift_rad,
        max_linear_speed_mps, max_angular_speed_rps):
    """Return the averaged pose when a received-odom window is stationary."""
    if len(samples) < 2:
        return None
    if samples[-1][0] - samples[0][0] < min_duration_sec:
        return None

    reference = samples[0]
    for sample in samples:
        dx = sample[1] - reference[1]
        dy = sample[2] - reference[2]
        if math.hypot(dx, dy) > max_pose_drift_m:
            return None
        if abs(normalize_angle(sample[3] - reference[3])) > max_yaw_drift_rad:
            return None
        if math.hypot(sample[4], sample[5]) > max_linear_speed_mps:
            return None
        if abs(sample[6]) > max_angular_speed_rps:
            return None
    return mean_pose(samples)


def _finite_pose(points, name):
    values = points.get(name)
    if not isinstance(values, (list, tuple)) or len(values) != 3:
        raise FixedStartConfigError(f'point {name} must be [x, y, yaw]')
    try:
        pose = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise FixedStartConfigError(
            f'point {name} contains an unset/non-numeric value') from exc
    if not all(math.isfinite(value) for value in pose):
        raise FixedStartConfigError(f'point {name} must contain finite values')
    return pose


def load_fixed_start_points(path, require_calibrated=True):
    config_path = Path(path).expanduser()
    if not config_path.is_file():
        raise FixedStartConfigError(f'points file does not exist: {config_path}')
    with config_path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}

    params = data.get('mission_points', {}).get('ros__parameters', {})
    if require_calibrated and params.get('calibrated') is not True:
        raise FixedStartConfigError(
            'points are not approved: set calibrated: true only after P and QR_NAV are measured')
    frame_id = str(params.get('frame_id', '')).strip()
    if frame_id != 'map':
        raise FixedStartConfigError('fixed-start points must use frame_id: map')

    points = params.get('points', {})
    p_pose = _finite_pose(points, 'P')
    qr_pose = _finite_pose(points, 'QR_NAV')
    route = params.get('routes', {}).get('fixed_start')
    if route != ['QR_NAV']:
        raise FixedStartConfigError(
            'route fixed_start must contain only QR_NAV; P is alignment-only')
    return {
        'path': config_path,
        'frame_id': frame_id,
        'P': p_pose,
        'QR_NAV': qr_pose,
    }
