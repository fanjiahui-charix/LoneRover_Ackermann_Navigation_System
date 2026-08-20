#!/usr/bin/env python3
"""Materialize deterministic single/two-cone Channel-v2 state-machine paths."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from channel_asset_common import (  # noqa: E402
    FOOTPRINT_FRONT_M, FOOTPRINT_HALF_WIDTH_M, FOOTPRINT_REAR_M,
    audit_path, cyclic_geometry, load_map, normalize, read_path_csv,
    sha256_file, write_tube,
)


SCENARIOS = {
    "single-inner": ((0.40, "inner", 0.19),),
    "single-outer": ((0.40, "outer", 0.19),),
    "single-center": ((0.40, "inner", 0.04),),
    "two-cones": ((0.32, "inner", 0.19), (0.67, "outer", 0.19)),
    # Two physically separated cones on one side.  This exercises two
    # independent outbound/return decisions without pretending that an
    # opposite-side pair always has a legal corridor.
    "two-cones-same-inner": ((0.18, "inner", 0.19), (0.58, "inner", 0.19)),
    "two-cones-same-outer": ((0.18, "outer", 0.19), (0.58, "outer", 0.19)),
}


def distance(a, b) -> float:
    return math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))


def loop_length(rows) -> float:
    return sum(distance(rows[index], rows[(index + 1) % len(rows)]) for index in range(len(rows)))


def forward_s(rows, start: int, finish: int) -> float:
    total = 0.0
    index = start
    for _ in range(len(rows) + 1):
        if index == finish:
            return total
        following = (index + 1) % len(rows)
        total += distance(rows[index], rows[following])
        index = following
    raise RuntimeError("cyclic traversal failed")


def advance(rows, start: int, amount_m: float) -> int:
    index = start
    travelled = 0.0
    while travelled < amount_m:
        following = (index + 1) % len(rows)
        travelled += distance(rows[index], rows[following])
        index = following
    return index


def retreat(rows, start: int, amount_m: float) -> int:
    index = start
    travelled = 0.0
    while travelled < amount_m:
        previous = (index - 1) % len(rows)
        travelled += distance(rows[previous], rows[index])
        index = previous
    return index


def segment(rows, start: int, finish: int) -> list[dict]:
    output = []
    index = start
    for _ in range(len(rows) + 1):
        output.append(rows[index])
        if index == finish:
            return output
        index = (index + 1) % len(rows)
    raise RuntimeError("segment traversal failed")


def append_rows(output: list[dict], additions: list[dict]) -> None:
    for row in additions:
        if output and distance(output[-1], row) < 0.001:
            continue
        output.append(row)


def select_connector(entries, rows, source_index: int, source_lane: str,
                     target_lane: str, direction: str,
                     maximum_forward_gap_m: float | None = 0.12) -> dict:
    candidates = [
        entry for entry in entries
        if entry["direction"] == direction and entry["source_lane"] == source_lane and
        entry["target_lane"] == target_lane
    ]
    if not candidates:
        raise RuntimeError(f"no {direction} {source_lane}->{target_lane} connectors")
    selected = min(candidates, key=lambda entry: forward_s(
        rows, source_index, int(entry["source_index"]),
    ))
    gap = forward_s(rows, source_index, int(selected["source_index"]))
    if maximum_forward_gap_m is not None and gap > maximum_forward_gap_m:
        raise RuntimeError(f"nearest forward connector gap {gap:.3f} m exceeds runtime gate")
    return selected


def select_outbound(entries, center, current_index: int, cone_index: int,
                    target_lane: str, direction: str,
                    trigger_lookahead_m: float) -> dict:
    distance_to_cone = forward_s(center, current_index, cone_index)
    candidates = []
    for entry in entries:
        if entry["direction"] != direction or entry["source_lane"] != "center" or \
                entry["target_lane"] != target_lane:
            continue
        source_index = int(entry["source_index"])
        progress = forward_s(center, current_index, source_index)
        cone_forward = forward_s(center, source_index, cone_index)
        if progress <= distance_to_cone and cone_forward <= trigger_lookahead_m:
            candidates.append((abs(cone_forward - 0.80 * trigger_lookahead_m), entry))
    if not candidates:
        raise RuntimeError("no outbound connector can react within configured trigger lookahead")
    return min(candidates, key=lambda item: item[0])[1]


def cone_clearance(rows, cone_x: float, cone_y: float) -> float:
    minimum = math.inf
    for row in rows:
        dx = cone_x - float(row["x"])
        dy = cone_y - float(row["y"])
        cosine = math.cos(float(row["yaw"]))
        sine = math.sin(float(row["yaw"]))
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        outside_x = max(-FOOTPRINT_REAR_M - local_x, 0.0, local_x - FOOTPRINT_FRONT_M)
        outside_y = max(abs(local_y) - FOOTPRINT_HALF_WIDTH_M, 0.0)
        minimum = min(minimum, math.hypot(outside_x, outside_y))
    return minimum


def lut_code(path: Path, x_m: float, y_m: float, width: int = 500,
             height: int = 500, resolution_m: float = 0.01) -> int:
    ix = int(math.floor(x_m / resolution_m))
    iy = int(math.floor(y_m / resolution_m))
    if ix < 0 or iy < 0 or ix >= width or iy >= height:
        return 0
    return path.read_bytes()[iy * width + ix]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--connector-dir", type=Path, required=True)
    parser.add_argument("--avoid-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--direction", choices=("cw", "ccw"), required=True)
    parser.add_argument("--scenario", choices=tuple(SCENARIOS), required=True)
    parser.add_argument("--trigger-lookahead", type=float, default=1.25)
    parser.add_argument("--return-margin", type=float, default=0.45)
    parser.add_argument(
        "--fixed-center-index", type=int,
        help=("deterministic shadow-test cone location on the selected Center "
              "Tube; omit to use the scenario's fractional locations"),
    )
    parser.add_argument(
        "--fixed-center-indices",
        help=("comma-separated deterministic Center indices, one per cone; "
              "takes precedence over --fixed-center-index"),
    )
    parser.add_argument("--cone-radius", type=float, default=0.13)
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    fixed_indices = None
    if args.fixed_center_indices:
        try:
            fixed_indices = [int(value.strip()) for value in
                             args.fixed_center_indices.split(",") if value.strip()]
        except ValueError as exc:
            raise SystemExit("fixed-center-indices must be comma-separated integers") from exc
        if not fixed_indices:
            raise SystemExit("fixed-center-indices cannot be empty")
    if args.out_dir.exists():
        raise SystemExit(f"refusing to overwrite scenario: {args.out_dir}")
    args.out_dir.mkdir(parents=True)
    tubes = {
        lane: read_path_csv(args.tube_dir / f"tube_{lane}_{args.direction}.csv")
        for lane in ("center", "inner", "outer")
    }
    index = json.loads((args.connector_dir / "connector_index.json").read_text())
    entries = index["entries"]
    center = tubes["center"]
    cones = []
    for cone_number, (fraction, side, lateral_m) in enumerate(SCENARIOS[args.scenario]):
        selected_fixed_index = (
            fixed_indices[cone_number % len(fixed_indices)]
            if fixed_indices is not None else args.fixed_center_index
        )
        cone_index = (
            int(selected_fixed_index) % len(center)
            if selected_fixed_index is not None else
            advance(center, 0, fraction * loop_length(center))
        )
        row = center[cone_index]
        inner_left_sign = 1.0 if args.direction == "ccw" else -1.0
        signed_left = inner_left_sign * lateral_m if side == "inner" else -inner_left_sign * lateral_m
        x_m = float(row["x"]) - signed_left * math.sin(float(row["yaw"]))
        y_m = float(row["y"]) + signed_left * math.cos(float(row["yaw"]))
        code = lut_code(
            args.avoid_dir / f"channel_cone_avoid_lane_1cm_{args.direction}.bin",
            x_m, y_m,
        )
        requested_lane = "outer" if code == 1 else ("inner" if code == 2 else "invalid")
        cones.append({
            "fraction": fraction, "center_index": cone_index, "declared_side": side,
            "x_m": x_m, "y_m": y_m, "lut_code": code,
            "requested_lane": requested_lane, "radius_m": args.cone_radius,
        })
    if any(cone["requested_lane"] == "invalid" for cone in cones):
        raise SystemExit("scenario cone classified OUTSIDE_OR_INVALID")

    stitched: list[dict] = []
    events = []
    current_center_index = 0
    for cone in cones:
        outbound = select_outbound(
            entries, center, current_center_index, int(cone["center_index"]),
            cone["requested_lane"], args.direction, args.trigger_lookahead,
        )
        trigger_index = int(outbound["source_index"])
        append_rows(stitched, segment(
            center, current_center_index, int(outbound["source_index"]),
        ))
        append_rows(stitched, read_path_csv(args.connector_dir / outbound["file"]))
        side = tubes[cone["requested_lane"]]
        return_target = advance(
            side, int(cone["center_index"]), args.return_margin,
        )
        inbound = select_connector(
            entries, side, return_target, cone["requested_lane"], "center",
            args.direction, maximum_forward_gap_m=None,
        )
        append_rows(stitched, segment(
            side, int(outbound["target_index"]), int(inbound["source_index"]),
        ))
        append_rows(stitched, read_path_csv(args.connector_dir / inbound["file"]))
        current_center_index = int(inbound["target_index"])
        events.append({
            "cone_center_index": cone["center_index"],
            "trigger_index": trigger_index,
            "outbound_connector": outbound["id"],
            "selected_lane": cone["requested_lane"],
            "return_connector": inbound["id"],
            "return_deferred_distance_m": round(forward_s(
                side, return_target, int(inbound["source_index"])), 6),
        })
    append_rows(stitched, segment(center, current_center_index, 0))
    if len(stitched) > 2 and distance(stitched[0], stitched[-1]) < 0.001:
        stitched.pop()
    geometry = cyclic_geometry(
        [float(row["x"]) for row in stitched],
        [float(row["y"]) for row in stitched],
    )
    path = args.out_dir / "scenario_path.csv"
    write_tube(path, geometry, "scenario", args.direction)
    grid = load_map(args.map)
    static_audit = audit_path(grid, geometry, closed=True, dense_step_m=0.003)
    for cone in cones:
        cone["minimum_swept_footprint_clearance_m"] = round(
            cone_clearance(geometry, cone["x_m"], cone["y_m"]), 6,
        )
        cone["dynamic_clearance_pass"] = (
            cone["minimum_swept_footprint_clearance_m"] + 1.0e-6 >= args.cone_radius
        )
    dynamic_path = args.out_dir / "dynamic_obstacles.json"
    dynamic_path.write_text(json.dumps({
        "schema_version": 1, "obstacles": cones,
    }, indent=2, sort_keys=True) + "\n")
    report = {
        "schema_version": 2,
        "scenario": args.scenario,
        "direction": args.direction,
        "trigger_lookahead_m": args.trigger_lookahead,
        "return_margin_m": args.return_margin,
        "state_machine_events": events,
        "cones": cones,
        "static_audit": static_audit,
        "pass": bool(static_audit["admitted"]) and all(
            cone["dynamic_clearance_pass"] for cone in cones
        ),
        "path": {"file": path.name, "sha256": sha256_file(path)},
        "dynamic_obstacles": {
            "file": dynamic_path.name, "sha256": sha256_file(dynamic_path),
        },
    }
    (args.out_dir / "scenario_report.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n"
    )
    print(json.dumps({
        "pass": report["pass"], "out": str(args.out_dir), "events": events,
    }, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
