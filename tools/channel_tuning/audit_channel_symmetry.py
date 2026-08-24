#!/usr/bin/env python3
"""Audit directional Tube and Ackermann vehicle-response symmetry without ROS.

The STM32 calibration is a right-front-wheel angle/PWM map.  This tool keeps
that physical layer separate from the identified actuator gain and reports
both the exact kinematic mirror and the current identified response mirror.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import sys

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))
VEHICLE_MODEL = CHANNEL_TUNING.parent / "vehicle_model"
if str(VEHICLE_MODEL) not in sys.path:
    sys.path.insert(0, str(VEHICLE_MODEL))

from ackermann_vehicle_model import (  # noqa: E402
    WHEELBASE_M,
    TRACK_WIDTH_M,
    center_yaw_rate_from_right_wheel_angle,
    twist_to_right_wheel_angle,
)


def wrap(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def rows(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [{key: float(row[key]) for key in (
            "x_m", "y_m", "yaw_rad", "kappa_1pm")}
                for row in csv.DictReader(stream)]


def plant_microtest(gain: float) -> list[dict[str, float]]:
    output = []
    for command_w in (0.45, 0.625):
        positive_angle = twist_to_right_wheel_angle(0.30, command_w)
        negative_angle = twist_to_right_wheel_angle(0.30, -command_w)
        positive_actual = center_yaw_rate_from_right_wheel_angle(
            0.30, positive_angle * gain)
        negative_actual = center_yaw_rate_from_right_wheel_angle(
            0.30, negative_angle * gain)
        output.append({
            "command_w_radps": command_w,
            "exact_positive_angle_rad": positive_angle,
            "exact_negative_angle_rad": negative_angle,
            "gain_model_positive_kappa_1pm": positive_actual / 0.30,
            "gain_model_negative_kappa_1pm": negative_actual / 0.30,
            "gain_model_mirror_error_1pm": (
                positive_actual + negative_actual) / 0.30,
        })
    return output


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cw", type=Path, required=True)
    parser.add_argument("--ccw", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    cw, ccw = rows(args.cw), rows(args.ccw)
    if len(cw) != len(ccw) or len(cw) < 2:
        raise SystemExit("CW/CCW Tube sample counts must match and be non-trivial")
    n = len(cw)
    xy_error = []
    yaw_error = []
    kappa_magnitude_error = []
    for index, point in enumerate(cw):
        reverse = ccw[n - 1 - index]
        xy_error.append(math.hypot(
            point["x_m"] - reverse["x_m"],
            point["y_m"] - reverse["y_m"],
        ))
        yaw_error.append(abs(wrap(
            reverse["yaw_rad"] - point["yaw_rad"] + math.pi)))
        kappa_magnitude_error.append(abs(
            abs(point["kappa_1pm"]) - abs(reverse["kappa_1pm"])))
    report = {
        "schema_version": 1,
        "tube": {
            "cw": str(args.cw),
            "ccw": str(args.ccw),
            "samples": n,
            "max_xy_mismatch_m": max(xy_error),
            "max_reverse_yaw_mismatch_rad": max(yaw_error),
            "max_curvature_magnitude_mismatch_1pm": max(kappa_magnitude_error),
            "pass": max(xy_error) <= 1.0e-3 and max(yaw_error) <= 1.0e-3
            and max(kappa_magnitude_error) <= 1.0e-3,
        },
        "physical_calibration_semantics": {
            "layer": "STM32 right-front-wheel steering angle and PWM map",
            "wheelbase_m": WHEELBASE_M,
            "track_width_m": TRACK_WIDTH_M,
            "center_curvature_gain_is_not_calibrated_servo_data": True,
            "source": "/root/RDKX5_OrigincarPro小车STM32源码下位机/BALANCE/servo_fit.c and HARDWARE/usartx.c",
        },
        "shadow_gain_microtest": {
            "identified_common_gain": 0.96061,
            "cases": plant_microtest(0.96061),
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["tube"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
