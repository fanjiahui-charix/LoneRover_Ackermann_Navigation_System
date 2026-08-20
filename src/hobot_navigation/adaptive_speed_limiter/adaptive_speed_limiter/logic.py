import math
from typing import Iterable

import numpy as np


def stop_speed(distance: float, decel: float, reaction: float) -> float:
    """Return the maximum speed that can stop within the supplied distance."""
    distance = max(0.0, float(distance))
    decel = max(1e-6, float(decel))
    reaction = max(0.0, float(reaction))
    delayed_speed = decel * reaction
    return max(
        0.0,
        -delayed_speed
        + math.sqrt(delayed_speed * delayed_speed + 2.0 * decel * distance),
    )


def goal_speed_limit(
    remaining: float,
    decel: float,
    reaction: float,
    buffer: float,
    goal_tolerance: float,
    creep_speed: float,
    creep_distance: float,
) -> float:
    """Return a braking envelope that does not create a limiter deadlock."""
    remaining = max(0.0, float(remaining))
    tolerance = max(0.0, float(goal_tolerance))
    if remaining <= tolerance:
        return 0.0

    speed = stop_speed(
        max(0.0, remaining - max(0.0, float(buffer))),
        decel,
        reaction,
    )
    if remaining <= max(float(creep_distance), tolerance):
        speed = max(speed, max(0.0, float(creep_speed)))
    return speed


def cost_to_clearance(
    cost: int,
    scaling: float,
    inscribed_radius: float,
    inflation_radius: float,
) -> float:
    """Approximately invert the Nav2 exponential inflation cost."""
    cost = int(cost)
    if cost >= 253:
        return 0.0
    if cost <= 0:
        return max(float(inflation_radius), float(inscribed_radius))
    ratio = max(1.0 / 252.0, min(1.0, float(cost) / 252.0))
    return max(
        0.0,
        float(inscribed_radius)
        - math.log(ratio) / max(float(scaling), 1e-6),
    )


def rectangle_samples(
    front: float,
    rear: float,
    half_width: float,
    padding: float,
    step: float,
) -> np.ndarray:
    """Return dense samples covering the padded rectangular footprint."""
    x_min = float(rear) - float(padding)
    x_max = float(front) + float(padding)
    y_min = -float(half_width) - float(padding)
    y_max = float(half_width) + float(padding)
    spacing = max(0.005, float(step))

    x_count = max(2, int(math.ceil((x_max - x_min) / spacing)) + 1)
    y_count = max(2, int(math.ceil((y_max - y_min) / spacing)) + 1)
    xs = np.linspace(x_min, x_max, x_count, dtype=np.float64)
    ys = np.linspace(y_min, y_max, y_count, dtype=np.float64)
    xx, yy = np.meshgrid(xs, ys)
    return np.column_stack((xx.ravel(), yy.ravel()))


def bounded_monotonic_closest_index(
    plan_xy: np.ndarray,
    robot_xy: np.ndarray,
    previous_index: int,
    forward_distance: float,
) -> int:
    """Find a closest path point without jumping to a nearby later branch."""
    points = np.asarray(plan_xy, dtype=np.float64)
    robot = np.asarray(robot_xy, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) == 0:
        raise ValueError('plan_xy must be a non-empty Nx2 array')
    if robot.shape != (2,):
        raise ValueError('robot_xy must contain exactly two coordinates')

    start = max(0, min(int(previous_index), len(points) - 1))
    distance = max(0.0, float(forward_distance))
    segment_lengths = np.linalg.norm(
        np.diff(points[start:], axis=0), axis=1)
    cumulative = np.r_[0.0, np.cumsum(segment_lengths)]
    window_size = max(
        1, int(np.searchsorted(cumulative, distance, side='right')))
    stop = min(len(points), start + window_size)
    window = points[start:stop]
    offset = int(np.argmin(
        np.sum((window - robot[None, :]) ** 2, axis=1)))
    return start + offset


def conservative_clearance(
    previous: float | None,
    observed: float,
    recovery_alpha: float,
) -> float:
    """Apply dangerous clearance drops immediately and smooth only recovery."""
    current = max(0.0, float(observed))
    if previous is None or not math.isfinite(float(previous)):
        return current
    prior = max(0.0, float(previous))
    if current <= prior:
        return current
    alpha = max(0.0, min(1.0, float(recovery_alpha)))
    return prior + alpha * (current - prior)


def min_valid(values: Iterable[float], default: float = 0.0) -> float:
    """Return the smallest finite value or a supplied default."""
    finite_values = [float(value) for value in values if math.isfinite(float(value))]
    return min(finite_values) if finite_values else float(default)


def limited_signed_speed(
    commanded_speed: float,
    forward_limit: float,
    reverse_limit: float,
) -> float:
    """Limit speed magnitude while retaining the commanded travel direction."""
    commanded = float(commanded_speed)
    if commanded >= 0.0:
        return min(commanded, max(0.0, float(forward_limit)))
    return -min(-commanded, max(0.0, float(reverse_limit)))


def apply_minimum_drive_speed(
    commanded_speed: float,
    limited_speed: float,
    active_speed_limit: float,
    minimum_drive_speed: float,
) -> float:
    """Raise a nonzero command only when the active safety envelope permits it."""
    commanded = float(commanded_speed)
    limited = float(limited_speed)
    active_limit = max(0.0, float(active_speed_limit))
    minimum = max(0.0, float(minimum_drive_speed))
    if abs(commanded) <= 1e-6 or abs(limited) <= 1e-6 or minimum <= 0.0:
        return limited
    if active_limit + 1e-9 < minimum or abs(limited) >= minimum:
        return limited
    return math.copysign(minimum, commanded)
