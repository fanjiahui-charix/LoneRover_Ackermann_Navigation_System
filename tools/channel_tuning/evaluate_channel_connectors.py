#!/usr/bin/env python3
"""Independent connector-library geometry, endpoint and map admission."""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import audit_path, load_map, read_path_csv, sha256_file  # noqa: E402


def pose_error(row, expected):
    return math.hypot(float(row["x"]) - float(expected[0]), float(row["y"]) - float(expected[1]))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-dir", type=Path, required=True)
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    index = json.loads((args.candidate_dir / "connector_index.json").read_text())
    grid = load_map(args.map)
    tubes = {path.stem: read_path_csv(path) for path in args.tube_dir.glob("tube_*.csv")}
    results = {}
    coverage = Counter()
    segment_anchors = defaultdict(list)
    for entry in index["entries"]:
        path = args.candidate_dir / entry["file"]
        rows = read_path_csv(path)
        source = tubes[f"tube_{entry['source_lane']}_{entry['direction']}"]
        target = tubes[f"tube_{entry['target_lane']}_{entry['direction']}"]
        audit = audit_path(grid, rows, closed=False, dense_step_m=0.0025)
        start_error = pose_error(rows[0], (
            source[entry["source_index"]]["x"], source[entry["source_index"]]["y"], 0))
        finish_error = pose_error(rows[-1], (
            target[entry["target_index"]]["x"], target[entry["target_index"]]["y"], 0))
        checks = {
            "hash_matches_index": sha256_file(path) == entry["sha256"],
            "sample_count_ge_80": len(rows) >= 80,
            "start_matches_source_tube": start_error <= 2.0e-6,
            "finish_matches_target_tube": finish_error <= 2.0e-6,
            "zero_dense_static_collision": audit["hard_violation_count"] == 0,
            "hard_rmin_ge_0p35": audit["minimum_radius_m"] + 1.0e-6 >= 0.35,
            "minimum_clearance_ge_0p01": audit["minimum_static_footprint_clearance_m"] + 1.0e-6 >= 0.01,
            "source_s_matches_index": abs(
                float(entry["source_s_m"]) - float(source[entry["source_index"]]["s"])
            ) <= 1.0e-6,
            "target_s_matches_index": abs(
                float(entry["target_s_m"]) - float(target[entry["target_index"]]["s"])
            ) <= 1.0e-6,
            "source_anchor_is_low_curvature": abs(
                float(source[entry["source_index"]]["kappa"])
            ) < float(index["source_curvature_threshold_1pm"]) + 1.0e-9,
        }
        coverage[(entry["direction"], entry["source_lane"], entry["target_lane"])] += 1
        segment_anchors[(
            entry["direction"], entry["source_lane"], entry["target_lane"],
            int(entry["source_low_curvature_segment_id"]),
        )].append(float(entry["source_s_m"]))
        results[entry["id"]] = {
            "pass": all(checks.values()), "checks": checks,
            "start_error_m": start_error, "finish_error_m": finish_error,
            "audit": audit,
        }
    required = {
        (direction, source, target)
        for direction in ("cw", "ccw")
        for source, target in (("center", "inner"), ("inner", "center"),
                               ("center", "outer"), ("outer", "center"))
    }
    spacing_checks = {}
    configured_spacing = float(index["anchor_spacing_m"])
    rejected_by_segment = defaultdict(list)
    for entry in index.get("rejected_anchors", []):
        rejected_by_segment[(
            entry["direction"], entry["source_lane"], entry["target_lane"],
            int(entry["source_low_curvature_segment_id"]),
        )].append(float(entry["source_s_m"]))
    for key, positions in sorted(segment_anchors.items()):
        ordered = sorted(positions)
        gaps = [b - a for a, b in zip(ordered, ordered[1:])]
        excess_gaps = [
            (a, b) for a, b in zip(ordered, ordered[1:])
            if b - a > configured_spacing + 0.02
        ]
        unexplained_gaps = [
            (a, b) for a, b in excess_gaps
            if not any(a < rejected < b for rejected in rejected_by_segment[key])
        ]
        spacing_checks["/".join(map(str, key))] = {
            "anchor_count": len(ordered),
            "minimum_gap_m": round(min(gaps), 6) if gaps else None,
            "maximum_gap_m": round(max(gaps), 6) if gaps else None,
            "excess_gap_count": len(excess_gaps),
            "unexplained_excess_gap_count": len(unexplained_gaps),
            "pass": not gaps or (
                min(gaps) + 0.002 >= configured_spacing and
                not unexplained_gaps
            ),
        }
    report = {
        "schema_version": 2,
        "pass": bool(results) and all(value["pass"] for value in results.values()) and
        all(coverage[key] >= 2 for key in required),
        "coverage": {"/".join(key): coverage[key] for key in sorted(required)},
        "anchor_spacing_m": configured_spacing,
        "source_curvature_threshold_1pm": float(
            index["source_curvature_threshold_1pm"]
        ),
        "rejected_anchor_count": len(index.get("rejected_anchors", [])),
        "per_low_curvature_segment_spacing": spacing_checks,
        "connectors": results,
    }
    report["pass"] = bool(report["pass"]) and all(
        value["pass"] for value in spacing_checks.values()
    )
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    with args.out.with_suffix(".csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("connector", "pass", "rmin_m", "clearance_m", "collisions"))
        for name, value in sorted(results.items()):
            writer.writerow((name, str(value["pass"]).lower(), value["audit"]["minimum_radius_m"],
                             value["audit"]["minimum_static_footprint_clearance_m"],
                             value["audit"]["hard_violation_count"]))
    print(json.dumps({"pass": report["pass"], "connectors": len(results), "out": str(args.out)}, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
