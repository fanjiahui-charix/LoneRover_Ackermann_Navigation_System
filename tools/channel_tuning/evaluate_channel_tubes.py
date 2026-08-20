#!/usr/bin/env python3
"""Independent geometry and swept-footprint admission for six Channel Tubes."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

TOOLS = Path(__file__).resolve().parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from channel_asset_common import (  # noqa: E402
    HARD_MINIMUM_RADIUS_M,
    audit_path,
    load_map,
    normalize,
    read_path_csv,
    sha256_file,
)


EXPECTED = tuple(
    f"tube_{lane}_{direction}.csv"
    for direction in ("cw", "ccw")
    for lane in ("center", "inner", "outer")
)


def segment_lengths(rows):
    return [
        math.hypot(
            float(rows[(index + 1) % len(rows)]["x"]) - float(rows[index]["x"]),
            float(rows[(index + 1) % len(rows)]["y"]) - float(rows[index]["y"]),
        ) for index in range(len(rows))
    ]


def independent_curvature(rows):
    output = []
    for index in range(len(rows)):
        a = rows[(index - 1) % len(rows)]
        b = rows[index]
        c = rows[(index + 1) % len(rows)]
        ab_x = float(b["x"]) - float(a["x"])
        ab_y = float(b["y"]) - float(a["y"])
        bc_x = float(c["x"]) - float(b["x"])
        bc_y = float(c["y"]) - float(b["y"])
        ac_x = float(c["x"]) - float(a["x"])
        ac_y = float(c["y"]) - float(a["y"])
        denominator = (
            math.hypot(ab_x, ab_y) * math.hypot(bc_x, bc_y) *
            math.hypot(ac_x, ac_y)
        )
        output.append(
            2.0 * (ab_x * bc_y - ab_y * bc_x) / denominator
            if denominator > 1.0e-12 else 0.0
        )
    return output


def evaluate(args) -> dict:
    grid = load_map(args.map)
    files = sorted(path.name for path in args.tube_dir.glob("tube_*.csv"))
    missing = sorted(set(EXPECTED) - set(files))
    extra = sorted(set(files) - set(EXPECTED))
    report: dict[str, object] = {
        "schema_version": 2,
        "evaluator": str(Path(__file__).resolve()),
        "map": {"path": str(args.map), "sha256": sha256_file(args.map)},
        "tube_dir": str(args.tube_dir),
        "required_margin_m": args.required_margin,
        "missing": missing,
        "extra": extra,
        "tubes": {},
        "cross_tube": {},
    }
    loaded = {}
    for filename in EXPECTED:
        path = args.tube_dir / filename
        if not path.exists():
            continue
        rows = read_path_csv(path)
        loaded[path.stem] = rows
        spacing = segment_lengths(rows)
        curvature = independent_curvature(rows)
        dense = audit_path(grid, rows, closed=True, dense_step_m=args.dense_step)
        maximum_curvature = max(abs(value) for value in curvature)
        minimum_radius = 1.0 / maximum_curvature if maximum_curvature > 1.0e-12 else math.inf
        lengths = np.asarray(spacing)
        radii = np.asarray([
            1.0 / abs(value) if abs(value) > 1.0e-12 else math.inf
            for value in curvature
        ])
        below_preferred = radii + 1.0e-6 < args.preferred_rmin
        below_preferred_length = float(np.sum(lengths[below_preferred]))
        minimum_radius_index = int(np.argmin(radii))
        curvature_wrap_jump = abs(curvature[-1] - curvature[0])
        stored_curvature_error = max(
            abs(float(row["kappa"]) - value)
            for row, value in zip(rows, curvature)
        )
        checks = {
            "sample_count_at_least_400": len(rows) >= 400,
            "closed_xy_le_0p015": float(dense["closure_xy_m"]) <= 0.015,
            "closed_yaw_le_0p03": float(dense["closure_yaw_rad"]) <= 0.03,
            "spacing_min_ge_0p002": min(spacing) >= 0.002,
            "spacing_p99_le_0p015": float(np.percentile(spacing, 99)) <= 0.015,
            "hard_rmin_ge_0p35": minimum_radius + 1.0e-6 >= HARD_MINIMUM_RADIUS_M,
            "zero_dense_static_collision": int(dense["hard_violation_count"]) == 0,
            "required_margin_met": (
                args.required_margin is None or
                float(dense["minimum_static_footprint_clearance_m"]) + 1.0e-6 >=
                args.required_margin
            ),
            "curvature_wrap_jump_le_0p25": curvature_wrap_jump <= 0.25,
            "stored_curvature_matches_geometry": stored_curvature_error <= 0.05,
        }
        report["tubes"][path.stem] = {
            "sha256": sha256_file(path),
            "samples": len(rows),
            "spacing_min_m": round(min(spacing), 6),
            "spacing_p50_m": round(float(np.percentile(spacing, 50)), 6),
            "spacing_p99_m": round(float(np.percentile(spacing, 99)), 6),
            "spacing_max_m": round(max(spacing), 6),
            "independent_max_abs_curvature_1pm": round(maximum_curvature, 9),
            "independent_minimum_radius_m": round(minimum_radius, 6),
            "preferred_minimum_radius_m": args.preferred_rmin,
            "minimum_radius_index": minimum_radius_index,
            "minimum_radius_s_m": round(float(rows[minimum_radius_index]["s"]), 6),
            "length_with_radius_below_preferred_m": round(below_preferred_length, 6),
            "percentage_with_radius_below_preferred": round(
                100.0 * below_preferred_length / max(float(np.sum(lengths)), 1.0e-9), 4,
            ),
            "curvature_wrap_jump_1pm": round(curvature_wrap_jump, 9),
            "stored_curvature_max_error_1pm": round(stored_curvature_error, 9),
            "dense_full_footprint": dense,
            "green_intrusion_count": int(dense["hard_violation_count"]),
            "yellow_legality": int(dense["hard_violation_count"]) == 0,
            "checks": checks,
            "pass": all(checks.values()),
        }

    for direction in ("cw", "ccw"):
        center = loaded.get(f"tube_center_{direction}")
        if not center:
            continue
        inner_expected_sign = 1.0 if direction == "ccw" else -1.0
        for lane, expected_sign in (("inner", inner_expected_sign),
                                    ("outer", -inner_expected_sign)):
            side = loaded.get(f"tube_{lane}_{direction}")
            key = f"{lane}_{direction}"
            if not side or len(side) != len(center):
                report["cross_tube"][key] = {"pass": False, "reason": "sample_count_mismatch"}
                continue
            lateral = []
            corner_lateral = []
            for base, shifted in zip(center, side):
                dx = float(shifted["x"]) - float(base["x"])
                dy = float(shifted["y"]) - float(base["y"])
                value = -math.sin(float(base["yaw"])) * dx + math.cos(float(base["yaw"])) * dy
                lateral.append(value)
                if abs(float(base["kappa"])) > 0.15:
                    corner_lateral.append(value)
            signed = [expected_sign * value for value in lateral]
            signed_corner = [expected_sign * value for value in corner_lateral]
            checks = {
                "correct_side_all_samples": min(signed) > 0.002,
                "distinct_through_corners": min(signed_corner) > 0.002,
                "corner_mean_offset_gt_0p005": float(np.mean(signed_corner)) > 0.005,
            }
            report["cross_tube"][key] = {
                "signed_offset_min_m": round(min(signed), 6),
                "signed_offset_mean_m": round(float(np.mean(signed)), 6),
                "signed_offset_max_m": round(max(signed), 6),
                "corner_signed_offset_min_m": round(min(signed_corner), 6),
                "corner_signed_offset_mean_m": round(float(np.mean(signed_corner)), 6),
                "checks": checks,
                "pass": all(checks.values()),
            }

    report["pass"] = (
        not missing and not extra and len(report["tubes"]) == 6 and
        all(bool(value["pass"]) for value in report["tubes"].values()) and
        len(report["cross_tube"]) == 4 and
        all(bool(value["pass"]) for value in report["cross_tube"].values())
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--required-margin", type=float)
    parser.add_argument("--dense-step", type=float, default=0.003)
    parser.add_argument("--preferred-rmin", type=float, default=0.40)
    args = parser.parse_args()
    report = evaluate(args)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    csv_path = args.out.with_suffix(".csv")
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("tube", "pass", "rmin_m", "clearance_m", "collisions"))
        for name, value in sorted(report["tubes"].items()):
            dense = value["dense_full_footprint"]
            writer.writerow((name, str(value["pass"]).lower(),
                             value["independent_minimum_radius_m"],
                             dense["minimum_static_footprint_clearance_m"],
                             dense["hard_violation_count"]))
    print(json.dumps({"pass": report["pass"], "report": str(args.out)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
