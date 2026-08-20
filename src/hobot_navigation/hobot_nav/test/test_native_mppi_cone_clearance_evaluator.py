import math
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / "tools"))

from evaluate_native_run import point_to_footprint_distance  # noqa: E402


def test_exact_distance_to_frozen_asymmetric_footprint_at_zero_yaw():
    assert point_to_footprint_distance(1.0, 2.0, 0.0, 1.28, 2.0) == 0.0
    assert math.isclose(
        point_to_footprint_distance(1.0, 2.0, 0.0, 1.38, 2.0),
        0.10,
        abs_tol=1.0e-12)
    assert math.isclose(
        point_to_footprint_distance(1.0, 2.0, 0.0, 0.79, 2.0),
        0.10,
        abs_tol=1.0e-12)


def test_exact_distance_rotates_point_into_vehicle_frame():
    yaw = math.pi / 2.0
    assert math.isclose(
        point_to_footprint_distance(0.0, 0.0, yaw, 0.0, 0.38),
        0.10,
        abs_tol=1.0e-12)
    expected_corner = math.hypot(0.10, 0.10)
    assert math.isclose(
        point_to_footprint_distance(0.0, 0.0, yaw, -0.23, 0.38),
        expected_corner,
        abs_tol=1.0e-12)
