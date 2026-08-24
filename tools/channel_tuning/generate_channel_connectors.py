#!/usr/bin/env python3
"""Generate map-registered Center<->Side connector libraries from final Tubes."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import (  # noqa: E402
    audit_path, load_map, manifest_file_rows, open_geometry, read_path_csv,
    sha256_file, write_connector,
)


def contiguous_segments(indices: list[int], count: int) -> list[list[int]]:
    if not indices:
        return []
    result = [[indices[0]]]
    for index in indices[1:]:
        if index == result[-1][-1] + 1:
            result[-1].append(index)
        else:
            result.append([index])
    if len(result) > 1 and result[0][0] == 0:
        # Closed Tube: merge a low-curvature run split only by the CSV seam.
        if result[-1][-1] == count - 1:
            result[0] = result[-1] + result[0]
            result.pop()
    return result


def distance(a, b):
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def advance(rows, start: int, length_m: float) -> tuple[int, float]:
    index = start
    travelled = 0.0
    for _ in range(len(rows)):
        following = (index + 1) % len(rows)
        travelled += distance(rows[index], rows[following])
        index = following
        if travelled >= length_m:
            return index, travelled
    raise RuntimeError("connector advance exceeded one Tube loop")


def bezier_point(control, u):
    one = 1.0 - u
    weights = (
        one**5, 5 * one**4 * u, 10 * one**3 * u**2,
        10 * one**2 * u**3, 5 * one * u**4, u**5,
    )
    return (
        sum(weight * point[0] for weight, point in zip(weights, control)),
        sum(weight * point[1] for weight, point in zip(weights, control)),
    )


def connector_rows(start, finish, nominal_length_m: float):
    tangent_scale = nominal_length_m / 5.0
    start_tangent = (math.cos(float(start["yaw"])), math.sin(float(start["yaw"])))
    finish_tangent = (math.cos(float(finish["yaw"])), math.sin(float(finish["yaw"])))
    p0 = (float(start["x"]), float(start["y"]))
    p5 = (float(finish["x"]), float(finish["y"]))
    control = (
        p0,
        (p0[0] + tangent_scale * start_tangent[0], p0[1] + tangent_scale * start_tangent[1]),
        (p0[0] + 2.0 * tangent_scale * start_tangent[0], p0[1] + 2.0 * tangent_scale * start_tangent[1]),
        (p5[0] - 2.0 * tangent_scale * finish_tangent[0], p5[1] - 2.0 * tangent_scale * finish_tangent[1]),
        (p5[0] - tangent_scale * finish_tangent[0], p5[1] - tangent_scale * finish_tangent[1]),
        p5,
    )
    sample_count = max(91, int(math.ceil(nominal_length_m / 0.005)) + 1)
    xy = [bezier_point(control, index / (sample_count - 1)) for index in range(sample_count)]
    return open_geometry([point[0] for point in xy], [point[1] for point in xy])


def eligible_anchor_indices(rows, length_m: float, anchor_spacing_m: float,
                            transition_buffer_m: float,
                            curvature_threshold_1pm: float) -> list[tuple[int, int]]:
    stable = [
        index for index, row in enumerate(rows)
        if abs(float(row["kappa"])) < curvature_threshold_1pm
    ]
    selected: list[tuple[int, int]] = []
    for segment_id, segment in enumerate(contiguous_segments(stable, len(rows))):
        if len(segment) < 4:
            continue
        progress = [0.0]
        for previous, following in zip(segment, segment[1:]):
            progress.append(progress[-1] + distance(rows[previous], rows[following]))
        total_length = progress[-1]
        if total_length < length_m + 2.0 * transition_buffer_m:
            continue
        last_selected_s = -math.inf
        for position, index in zip(progress, segment):
            if position + 1.0e-9 < transition_buffer_m:
                continue
            if position + length_m + transition_buffer_m > total_length + 1.0e-9:
                break
            if position - last_selected_s + 1.0e-9 < anchor_spacing_m:
                continue
            selected.append((index, segment_id))
            last_selected_s = position
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--length", type=float, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--anchor-spacing", type=float, default=0.05)
    parser.add_argument("--transition-buffer", type=float, default=0.05)
    parser.add_argument("--source-curvature-threshold", type=float, default=0.25)
    args = parser.parse_args()
    if args.out_dir.exists():
        raise SystemExit(f"refusing to overwrite connector candidate: {args.out_dir}")
    if not 0.40 <= args.length <= 0.80:
        raise SystemExit("connector length must be within 0.40..0.80 m")
    if not 0.019 <= args.anchor_spacing <= 0.101:
        raise SystemExit("anchor spacing must be within 0.02..0.10 m")
    if not 0.03 <= args.transition_buffer <= 0.20:
        raise SystemExit("transition buffer must be within 0.03..0.20 m")
    if not 0.02 <= args.source_curvature_threshold <= 0.30:
        raise SystemExit("source curvature threshold must be within 0.02..0.30 1/m")
    grid = load_map(args.map)
    tubes = {
        path.stem: read_path_csv(path)
        for path in sorted(args.tube_dir.glob("tube_*.csv"))
    }
    if len(tubes) != 6:
        raise SystemExit("six Tube inputs are required")
    connector_dir = args.out_dir / "connectors"
    connector_dir.mkdir(parents=True)
    entries = []
    rejected_anchors = []
    audits = {}
    pairs = (("center", "inner"), ("inner", "center"),
             ("center", "outer"), ("outer", "center"))
    for direction in ("cw", "ccw"):
        for source_lane, target_lane in pairs:
            source = tubes[f"tube_{source_lane}_{direction}"]
            target = tubes[f"tube_{target_lane}_{direction}"]
            anchors = eligible_anchor_indices(
                source, args.length, args.anchor_spacing, args.transition_buffer,
                args.source_curvature_threshold,
            )
            for anchor_number, (source_index, segment_id) in enumerate(anchors):
                target_index, source_advance = advance(source, source_index, args.length)
                rows = connector_rows(source[source_index], target[target_index], args.length)
                anchor_id = f"{direction}_{source_lane}_to_{target_lane}_{anchor_number:04d}"
                path = connector_dir / f"connector_{anchor_id}.csv"
                audit = audit_path(grid, rows, closed=False, dense_step_m=0.003)
                metadata = {
                    "id": anchor_id,
                    "direction": direction,
                    "source_lane": source_lane,
                    "target_lane": target_lane,
                    "source_index": source_index,
                    "target_index": target_index,
                    "source_s_m": float(source[source_index]["s"]),
                    "target_s_m": float(target[target_index]["s"]),
                    "source_low_curvature_segment_id": segment_id,
                    "source_heading_rad": float(source[source_index]["yaw"]),
                    "source_advance_m": source_advance,
                    "nominal_length_m": args.length,
                    "actual_length_m": rows[-1]["s"],
                    "start": [rows[0]["x"], rows[0]["y"], rows[0]["yaw"]],
                    "finish": [rows[-1]["x"], rows[-1]["y"], rows[-1]["yaw"]],
                }
                if not audit["admitted"]:
                    rejected_anchors.append({
                        **metadata, "runtime_eligible": False,
                        "rejection": "exact_3mm_footprint_or_hard_rmin",
                        "audit": audit,
                    })
                    continue
                write_connector(
                    path, rows, direction=direction, source_lane=source_lane,
                    target_lane=target_lane, anchor_id=anchor_id,
                )
                entries.append({
                    **metadata,
                    "runtime_eligible": True,
                    "file": str(path.relative_to(args.out_dir)),
                    "sha256": sha256_file(path),
                })
                audits[anchor_id] = audit
    if not entries:
        raise SystemExit("no connector anchors generated")
    index = {
        "schema_version": 2,
        "candidate_only": True,
        "nominal_length_m": args.length,
        "anchor_spacing_m": args.anchor_spacing,
        "transition_buffer_m": args.transition_buffer,
        "source_curvature_threshold_1pm": args.source_curvature_threshold,
        "tube_dir": str(args.tube_dir),
        "map_sha256": sha256_file(args.map),
        "entries": entries,
        "rejected_anchors": rejected_anchors,
    }
    index_path = args.out_dir / "connector_index.json"
    index_path.write_text(json.dumps(index, indent=2, sort_keys=True) + "\n")
    report = {
        "schema_version": 2,
        "connectors": audits,
        "rejected_anchor_count": len(rejected_anchors),
        "rejected_anchors": rejected_anchors,
        "all_generator_checks_pass": bool(audits) and all(
            value["admitted"] for value in audits.values()
        ),
        "runtime_promotion_allowed": False,
        "remaining_gates": ["independent_connector_evaluator", "native_RPP_connector_sweep"],
    }
    report_path = args.out_dir / "geometry_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    manifest = {
        "schema_version": 2,
        "asset_kind": "channel_connector_candidate",
        "candidate_only": True,
        "map_sha256": sha256_file(args.map),
        "tube_hashes": {
            path.name: sha256_file(path) for path in sorted(args.tube_dir.glob("tube_*.csv"))
        },
        "generator_sha256": sha256_file(Path(__file__).resolve()),
        "files": manifest_file_rows(args.out_dir, [
            *connector_dir.glob("*.csv"), index_path, report_path,
        ]),
    }
    (args.out_dir / "manifest.json").write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps({"connectors": len(entries), "rejected_anchors": len(rejected_anchors),
                      "generator_pass": report["all_generator_checks_pass"],
                      "out": str(args.out_dir)}, indent=2))
    return 0 if report["all_generator_checks_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
