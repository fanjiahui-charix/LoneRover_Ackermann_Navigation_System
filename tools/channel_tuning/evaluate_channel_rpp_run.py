#!/usr/bin/env python3
"""Score one native Channel RPP virtual-vehicle run and emit PASS/FAIL artifacts."""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import (  # noqa: E402
    FOOTPRINT_FRONT_M, FOOTPRINT_HALF_WIDTH_M, FOOTPRINT_REAR_M,
    audit_path, load_map, pose_clearance, read_path_csv,
)


def percentile(values, q):
    return float(np.percentile(values, q)) if values else None


def rounded(value, digits=6):
    return None if value is None else round(value, digits)


def json_default(value):
    """Convert NumPy scalar flags/numbers from map audits to JSON scalars."""
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f"Object of type {type(value).__name__} is not JSON serializable")


def maximum_duration(rows, predicate) -> float:
    start = None
    maximum = 0.0
    for row in rows:
        time = row["t"]
        if predicate(row):
            start = time if start is None else start
            maximum = max(maximum, time - start)
        else:
            start = None
    return maximum


def reference_projection(reference, samples):
    x = np.asarray([float(row["x"]) for row in reference])
    y = np.asarray([float(row["y"]) for row in reference])
    cumulative = np.zeros(len(reference))
    for index in range(1, len(reference)):
        cumulative[index] = cumulative[index - 1] + math.hypot(
            x[index] - x[index - 1], y[index] - y[index - 1])
    loop_length = cumulative[-1] + math.hypot(x[0] - x[-1], y[0] - y[-1])
    previous_s = None
    unwrapped = 0.0
    output = []
    for sample in samples:
        distances = (x - sample["x"]) ** 2 + (y - sample["y"]) ** 2
        index = int(np.argmin(distances))
        s_mod = float(cumulative[index])
        if previous_s is not None:
            delta = s_mod - previous_s
            if delta < -loop_length * 0.5:
                delta += loop_length
            elif delta > loop_length * 0.5:
                delta -= loop_length
            if delta >= -0.03:
                unwrapped += max(0.0, delta)
        previous_s = s_mod
        output.append({
            "index": index, "cte": math.sqrt(float(distances[index])),
            "s_mod": s_mod, "s_unwrapped": unwrapped,
            "reference_kappa": float(reference[index]["kappa"]),
        })
    return output, loop_length


def steering_oscillations(rows) -> int:
    signs = []
    for row in rows:
        steering = row["steering_deg"]
        if abs(steering) < 1.0 or abs(row["v"]) < 0.05:
            continue
        sign = 1 if steering > 0.0 else -1
        if not signs or signs[-1] != sign:
            signs.append(sign)
    return max(0, len(signs) - 1)


def obstacle_clearance(rows, obstacle) -> float:
    minimum = math.inf
    for row in rows:
        yaw = math.radians(row["yaw_deg"])
        dx = float(obstacle["x_m"]) - row["x"]
        dy = float(obstacle["y_m"]) - row["y"]
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        outside_x = max(-FOOTPRINT_REAR_M - local_x, 0.0, local_x - FOOTPRINT_FRONT_M)
        outside_y = max(abs(local_y) - FOOTPRINT_HALF_WIDTH_M, 0.0)
        minimum = min(minimum, math.hypot(outside_x, outside_y))
    return minimum


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--safety-budget", type=Path)
    parser.add_argument("--dynamic-obstacles", type=Path)
    parser.add_argument(
        "--require-full-lap", action="store_true",
        help="require a complete closed-Tube lap instead of the configured action window",
    )
    args = parser.parse_args()
    csv_path = args.run_dir / "shadow_run.csv"
    native_summary_path = args.run_dir / "shadow_run_summary.json"
    if not csv_path.exists() or not native_summary_path.exists():
        raise SystemExit("missing shadow_run.csv or shadow_run_summary.json")
    with csv_path.open(newline="", encoding="utf-8") as stream:
        source_rows = list(csv.DictReader(stream))
    numeric = {}
    rows = []
    for source in source_rows:
        for key, value in source.items():
            if key not in numeric:
                numeric[key] = True
        rows.append({key: float(value) for key, value in source.items()})
    if len(rows) < 10:
        raise SystemExit("too few shadow samples")
    reference = read_path_csv(args.reference)
    projections, loop_length = reference_projection(reference, rows)
    grid = load_map(args.map)
    clearances = []
    collisions = 0
    corner_cte: dict[str, list[float]] = {
        "bottom_left": [], "top_left": [], "top_right": [], "bottom_right": [],
    }
    center_x = float(np.mean([float(row["x"]) for row in reference]))
    center_y = float(np.mean([float(row["y"]) for row in reference]))
    requested_curvature = []
    actual_curvature = []
    for row, projection in zip(rows, projections):
        free, clearance = pose_clearance(grid, row["x"], row["y"], math.radians(row["yaw_deg"]))
        clearances.append(clearance)
        collisions += int(not free)
        if abs(projection["reference_kappa"]) > 0.15:
            horizontal = "left" if row["x"] < center_x else "right"
            vertical = "bottom" if row["y"] < center_y else "top"
            corner_cte[f"{vertical}_{horizontal}"].append(projection["cte"])
        if abs(row["raw_v"]) > 0.03:
            requested_curvature.append(row["raw_w"] / row["raw_v"])
        if abs(row["v"]) > 0.03:
            actual_curvature.append(row["w"] / row["v"])
    cte = [value["cte"] for value in projections]
    counter_duration = maximum_duration(rows, lambda row: (
        abs(row["raw_v"]) > 0.05 and abs(row["v"]) > 0.05 and
        abs(row["raw_w"] / row["raw_v"]) > 0.20 and
        abs(row["w"] / row["v"]) > 0.20 and
        (row["raw_w"] / row["raw_v"]) * (row["w"] / row["v"]) < 0.0
    ))
    stall_duration = maximum_duration(rows, lambda row: (
        row["safe_v"] > 0.05 and abs(row["v"]) < 0.02
    ))
    straight_speeds = [
        abs(row["v"]) for row, projection in zip(rows, projections)
        if abs(projection["reference_kappa"]) <= 0.15
    ]
    corner_speeds = [
        abs(row["v"]) for row, projection in zip(rows, projections)
        if abs(projection["reference_kappa"]) > 0.15
    ]
    lap_time = None
    initial_time = rows[0]["t"]
    for row, projection in zip(rows, projections):
        if projection["s_unwrapped"] >= loop_length:
            lap_time = row["t"] - initial_time
            break
    nav2_log = (args.run_dir / "nav2.log").read_text(errors="replace") if (
        args.run_dir / "nav2.log").exists() else ""
    collision_ahead = len(re.findall(r"collision ahead", nav2_log, flags=re.IGNORECASE))
    patience_abort = len(re.findall(r"Controller patience exceeded", nav2_log))
    native = json.loads(native_summary_path.read_text())
    result = native.get("result", "unknown")
    rpp = native.get("channel_rpp", {})
    extra_goal_distance = float(rpp.get("extra_goal_distance_m", 0.0) or 0.0)
    full_lap_tolerance_m = 0.05
    metrics = {
        "result": result,
        "samples": len(rows),
        "reference_loop_length_m": round(loop_length, 6),
        "completed_unwrapped_s_m": round(projections[-1]["s_unwrapped"], 6),
        "configured_action_window_m": round(extra_goal_distance, 6),
        "full_lap_completion_tolerance_m": full_lap_tolerance_m,
        "lap_completion_gap_m": round(
            max(0.0, loop_length - projections[-1]["s_unwrapped"]), 6),
        "lap_time_sec": None if lap_time is None else round(lap_time, 6),
        "cte_rms_m": round(math.sqrt(float(np.mean(np.square(cte)))), 6),
        "cte_p95_m": round(percentile(cte, 95), 6),
        "cte_max_m": round(max(cte), 6),
        "four_corner_max_cte_m": {
            name: None if not values else round(max(values), 6)
            for name, values in corner_cte.items()
        },
        "minimum_footprint_clearance_m": round(min(clearances), 6),
        "static_collision_count": collisions,
        "green_intrusion_count": collisions,
        "collision_ahead_count": collision_ahead,
        "controller_patience_abort_count": patience_abort,
        "maximum_stall_duration_sec": round(stall_duration, 6),
        "requested_curvature_abs_p95_1pm": round(percentile([abs(v) for v in requested_curvature], 95), 6) if requested_curvature else None,
        "actual_curvature_abs_p95_1pm": round(percentile([abs(v) for v in actual_curvature], 95), 6) if actual_curvature else None,
        "sustained_counter_steer_max_sec": round(counter_duration, 6),
        "steering_oscillation_count": steering_oscillations(rows),
        "straight_speed_p95_mps": rounded(percentile(straight_speeds, 95)),
        "corner_speed_p95_mps": rounded(percentile(corner_speeds, 95)),
        "controller_output_interval_p95_sec": rpp.get("raw_command_interval_p95_sec"),
        "controller_output_interval_p99_sec": rpp.get("raw_command_interval_p99_sec"),
    }
    dynamic_clearances = []
    if args.dynamic_obstacles is not None:
        obstacle_document = json.loads(args.dynamic_obstacles.read_text())
        for obstacle in obstacle_document.get("obstacles", []):
            clearance = obstacle_clearance(rows, obstacle)
            dynamic_clearances.append({
                "x_m": float(obstacle["x_m"]), "y_m": float(obstacle["y_m"]),
                "required_radius_m": float(obstacle["radius_m"]),
                "minimum_actual_swept_footprint_clearance_m": round(clearance, 6),
                "pass": clearance + 1.0e-6 >= float(obstacle["radius_m"]),
            })
        metrics["dynamic_obstacle_clearances"] = dynamic_clearances
    is_side_tube = args.reference.stem.startswith(("tube_inner_", "tube_outer_"))
    budget = None
    side_budget_metrics = None
    if is_side_tube:
        if args.safety_budget is None or not args.safety_budget.is_file():
            raise SystemExit("Side Tube evaluation requires --safety-budget")
        budget = json.loads(args.safety_budget.read_text())
        if budget.get("schema_version") != 2:
            raise SystemExit("Side Tube safety budget schema_version must be 2")
        if budget.get("measured") is not True:
            raise SystemExit(
                "Side Tube safety budget is not measured; run finalize-side-tubes "
                "after Center native RPP before evaluating Side Tubes"
            )
        nominal_clearance = float(audit_path(
            grid, reference, closed=True, dense_step_m=0.003,
        )["minimum_static_footprint_clearance_m"])
        measured_tracking_envelope = max(
            float(metrics["cte_max_m"]), 1.25 * float(metrics["cte_p95_m"]),
        )
        measured_required_margin = (
            measured_tracking_envelope + float(budget["localization_margin_m"]) +
            float(budget["residual_margin_m"])
        )
        side_budget_metrics = {
            "nominal_tube_static_clearance_m": round(nominal_clearance, 6),
            "measured_tracking_envelope_m": round(measured_tracking_envelope, 6),
            "measured_required_margin_m": round(measured_required_margin, 6),
            "budget_tracking_envelope_m": float(budget["tracking_envelope_m"]),
        }
        metrics["side_safety_budget"] = side_budget_metrics
    checks = {
        "follow_path_succeeded": result == "status_4",
        # A closed CSV contains the start pose again at the end.  Nav2's
        # goal checker quite correctly stops at the first physical occurrence
        # of a repeated goal, so a single FollowPath goal cannot express a
        # complete lap without inventing a synthetic off-track tail.  The
        # The channel action is a bounded Tube/connector segment;
        # score that configured window by default and offer an explicit full
        # lap gate for campaigns that use segmented goals.
        "action_window_completed": (
            extra_goal_distance <= 0.0 or
            projections[-1]["s_unwrapped"] + 0.02 >= extra_goal_distance
        ),
        "zero_static_collision": collisions == 0,
        "zero_collision_ahead": collision_ahead == 0,
        "zero_patience_abort": patience_abort == 0,
        "stall_lt_1p0sec": stall_duration < 1.0,
        "counter_steer_lt_0p8sec": counter_duration < 0.8,
        "controller_output_p99_le_0p18sec": (
            metrics["controller_output_interval_p99_sec"] is not None and
            metrics["controller_output_interval_p99_sec"] <= 0.18
        ),
    }
    if args.require_full_lap:
        checks["one_lap_completed"] = (
            projections[-1]["s_unwrapped"] + full_lap_tolerance_m >= loop_length)
    if is_side_tube:
        checks.update({
            "side_tracking_within_bound_safety_budget": (
                side_budget_metrics["measured_tracking_envelope_m"] <=
                side_budget_metrics["budget_tracking_envelope_m"] + 1.0e-6
            ),
            "side_unique_margin_fits_nominal_static_clearance": (
                side_budget_metrics["measured_required_margin_m"] <=
                side_budget_metrics["nominal_tube_static_clearance_m"] + 1.0e-6
            ),
        })
    else:
        checks.update({
            "minimum_clearance_ge_0p01": min(clearances) + 1.0e-6 >= 0.01,
            "cte_p95_le_0p06": metrics["cte_p95_m"] <= 0.06,
            "cte_max_le_0p10": metrics["cte_max_m"] <= 0.10,
        })
    if args.dynamic_obstacles is not None:
        checks["all_dynamic_cone_lethal_disks_cleared"] = (
            bool(dynamic_clearances) and all(item["pass"] for item in dynamic_clearances)
        )
    summary = {
        "schema_version": 1,
        "pass": all(checks.values()),
        "checks": checks,
        "metrics": metrics,
        "threshold_note": (
            "Side Tube uses SAFETY_BUDGET.json; Center/connector retain provisional "
            "shadow gates until the measured envelope is frozen. Closed Tube runs "
            "score the configured action window unless --require-full-lap is set."
        ),
        "safety_budget": None if args.safety_budget is None else {
            "path": str(args.safety_budget), "schema_version": budget.get("schema_version")
            if budget else None,
        },
    }
    (args.run_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True, default=json_default) + "\n")
    with (args.run_dir / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(("pass", *metrics.keys()))
        writer.writerow((str(summary["pass"]).lower(), *[
            json.dumps(value, sort_keys=True) if isinstance(value, dict) else value
            for value in metrics.values()
        ]))

    figure, axes = plt.subplots(2, 2, figsize=(12, 9), constrained_layout=True)
    extent = (0.0, grid.width * grid.resolution_m, 0.0, grid.height * grid.resolution_m)
    axes[0, 0].imshow(grid.pixels, cmap="gray", origin="upper", extent=extent)
    axes[0, 0].plot([float(row["x"]) for row in reference],
                    [float(row["y"]) for row in reference], "y-", label="target")
    axes[0, 0].plot([row["x"] for row in rows], [row["y"] for row in rows],
                    "c-", linewidth=1.0, label="actual")
    axes[0, 0].set_aspect("equal"); axes[0, 0].legend(); axes[0, 0].set_title("map path")
    elapsed = [row["t"] - initial_time for row in rows]
    axes[0, 1].plot(elapsed, cte); axes[0, 1].axhline(0.06, color="r", linestyle=":")
    axes[0, 1].set_title("cross-track error"); axes[0, 1].set_ylabel("m")
    axes[1, 0].plot(elapsed, [row["raw_v"] for row in rows], label="raw")
    axes[1, 0].plot(elapsed, [row["nav_v"] for row in rows], label="nav")
    axes[1, 0].plot(elapsed, [row["safe_v"] for row in rows], label="safe")
    axes[1, 0].plot(elapsed, [row["v"] for row in rows], label="actual")
    axes[1, 0].legend(); axes[1, 0].set_title("speed chain")
    axes[1, 1].plot(elapsed, [row["steering_deg"] for row in rows], label="actual steering")
    axes[1, 1].plot(elapsed, [row["steering_target_deg"] for row in rows], label="target steering")
    axes[1, 1].legend(); axes[1, 1].set_title("steering")
    figure.suptitle("PASS" if summary["pass"] else "FAIL")
    figure.savefig(args.run_dir / "result.png", dpi=160)
    plt.close(figure)
    print(json.dumps({"pass": summary["pass"], "summary": str(args.run_dir / 'summary.json')}, indent=2))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
