import math

import numpy as np
import pytest

from adaptive_speed_limiter.logic import (
    apply_minimum_drive_speed,
    bounded_monotonic_closest_index,
    conservative_clearance,
    cost_to_clearance,
    goal_speed_limit,
    limited_signed_speed,
    min_valid,
    rectangle_samples,
    stop_speed,
)


def test_stop_speed_increases_with_distance_and_rejects_negative_distance():
    assert stop_speed(0.4, 0.9, 0.25) > stop_speed(0.2, 0.9, 0.25)
    assert stop_speed(-1.0, 0.9, 0.25) == 0.0


def test_goal_creep_keeps_positive_envelope_outside_tolerance():
    speed = goal_speed_limit(
        remaining=0.11,
        decel=0.9,
        reaction=0.25,
        buffer=0.08,
        goal_tolerance=0.08,
        creep_speed=0.06,
        creep_distance=0.22,
    )
    assert speed >= 0.06


def test_goal_speed_is_zero_inside_tolerance():
    assert goal_speed_limit(
        0.07, 0.9, 0.25, 0.05, 0.08, 0.06, 0.22) == 0.0


def test_inflation_inverse_uses_inscribed_radius_and_blocks_lethal_cost():
    clearance = cost_to_clearance(
        100,
        scaling=17.0,
        inscribed_radius=0.11,
        inflation_radius=0.25,
    )
    expected = 0.11 - math.log(100.0 / 252.0) / 17.0
    assert clearance == pytest.approx(expected)
    assert clearance > 0.11
    assert cost_to_clearance(253, 17.0, 0.11, 0.25) == 0.0
    assert cost_to_clearance(255, 17.0, 0.11, 0.25) == 0.0


def test_rectangle_samples_cover_center_and_padded_bounds():
    points = rectangle_samples(0.27, -0.10, 0.12, 0.01, 0.03)
    assert len(points) > 50
    assert points[:, 0].min() <= -0.11
    assert points[:, 0].max() >= 0.28
    assert points[:, 1].min() <= -0.13
    assert points[:, 1].max() >= 0.13
    assert (
        (abs(points[:, 0]) < 0.02)
        & (abs(points[:, 1]) < 0.02)
    ).any()


def test_min_valid_ignores_non_finite_values():
    assert min_valid([math.nan, 0.3, math.inf, 0.2]) == 0.2
    assert min_valid([math.nan, math.inf], default=0.7) == 0.7


def test_closest_index_is_bounded_and_monotonic_on_nearby_return_path():
    plan = np.asarray([
        [0.0, 0.0],
        [1.0, 0.0],
        [2.0, 0.0],
        [3.0, 0.0],
        [3.0, 0.1],
        [2.0, 0.1],
        [1.0, 0.1],
        [0.0, 0.1],
    ])

    # A whole-path argmin would choose the later return branch near index 6.
    index = bounded_monotonic_closest_index(
        plan, np.asarray([1.0, 0.09]), 0, 1.5)
    assert index == 1

    index = bounded_monotonic_closest_index(
        plan, np.asarray([2.1, 0.0]), index, 1.5)
    assert index == 2
    assert bounded_monotonic_closest_index(
        plan, np.asarray([1.8, 0.0]), index, 1.5) >= index


def test_clearance_drops_immediately_and_recovers_gradually():
    assert conservative_clearance(0.20, 0.04, 0.35) == 0.04
    recovered = conservative_clearance(0.04, 0.20, 0.35)
    assert 0.04 < recovered < 0.20
    assert conservative_clearance(None, 0.12, 0.35) == 0.12


def test_signed_speed_limit_preserves_direction_and_caps_reverse():
    assert limited_signed_speed(0.4, 0.3, 0.15) == pytest.approx(0.3)
    assert limited_signed_speed(-0.4, 0.3, 0.15) == pytest.approx(-0.15)
    assert limited_signed_speed(-0.1, 0.3, 0.15) == pytest.approx(-0.1)
    assert limited_signed_speed(-0.1, 0.3, 0.0) == 0.0


def test_minimum_drive_floor_preserves_sign_and_respects_safety_limit():
    assert apply_minimum_drive_speed(
        0.05, 0.05, 0.40, 0.15) == pytest.approx(0.15)
    assert apply_minimum_drive_speed(
        -0.05, -0.05, 0.15, 0.15) == pytest.approx(-0.15)
    assert apply_minimum_drive_speed(
        0.05, 0.05, 0.10, 0.15) == pytest.approx(0.05)
    assert apply_minimum_drive_speed(
        0.0, 0.0, 0.40, 0.15) == 0.0
    assert apply_minimum_drive_speed(
        0.20, 0.20, 0.40, 0.15) == pytest.approx(0.20)
