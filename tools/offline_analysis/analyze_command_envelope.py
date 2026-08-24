#!/usr/bin/env python3
"""Measure command-envelope behavior from a native virtual-vehicle trace.

The report deliberately separates command slew from calibrated-plant response.
It is used to reject settings which make nav/safe commands more aggressive
without improving actual rise, braking, completion, or steering behavior.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import fmean
from typing import Iterable


def finite(value: str | float | None) -> float | None:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def percentile(values: Iterable[float], q: float) -> float | None:
    ordered = sorted(v for v in values if math.isfinite(v))
    if not ordered:
        return None
    index = (len(ordered) - 1) * q
    lo = int(math.floor(index))
    hi = int(math.ceil(index))
    if lo == hi:
        return ordered[lo]
    weight = index - lo
    return ordered[lo] * (1.0 - weight) + ordered[hi] * weight


def first_crossing(rows: list[dict[str, float]], field: str, threshold: float,
                   start_time: float) -> dict[str, float] | None:
    for row in rows:
        if row["t"] >= start_time and row[field] >= threshold:
            return {"t": row["t"], "delay_sec": row["t"] - start_time,
                    "x": row["x"], "y": row["y"]}
    return None


def first_sustained_below(rows: list[dict[str, float]], field: str,
                          threshold: float, start_index: int,
                          sustain_sec: float = 0.20) -> int | None:
    for i in range(start_index, len(rows)):
        if abs(rows[i][field]) >= threshold:
            continue
        limit = rows[i]["t"] + sustain_sec
        j = i
        while j < len(rows) and rows[j]["t"] <= limit:
            if abs(rows[j][field]) >= threshold:
                break
            j += 1
        if j < len(rows) and rows[j]["t"] > limit:
            return i
        if j == len(rows) and rows[-1]["t"] >= limit:
            return i
    return None


def rates(rows: list[dict[str, float]], field: str,
          held_command: bool = False) -> list[float]:
    if held_command:
        result: list[float] = []
        previous_t = rows[0]["t"]
        previous_value = rows[0][field]
        for row in rows[1:]:
            value = row[field]
            if math.isclose(value, previous_value, rel_tol=0.0, abs_tol=1.0e-12):
                continue
            dt = row["t"] - previous_t
            if 0.002 <= dt <= 0.5:
                result.append((value - previous_value) / dt)
            previous_t = row["t"]
            previous_value = value
        return result
    result: list[float] = []
    for left, right in zip(rows, rows[1:]):
        dt = right["t"] - left["t"]
        if 0.002 <= dt <= 0.25:
            result.append((right[field] - left[field]) / dt)
    return result


def held_command_steps(rows: list[dict[str, float]], field: str) -> list[float]:
    result: list[float] = []
    previous = rows[0][field]
    for row in rows[1:]:
        value = row[field]
        if math.isclose(value, previous, rel_tol=0.0, abs_tol=1.0e-12):
            continue
        result.append(value - previous)
        previous = value
    return result


def distance(left: dict[str, float], right: dict[str, float]) -> float:
    return math.hypot(right["x"] - left["x"], right["y"] - left["y"])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--evaluation", type=Path)
    ap.add_argument("--summary", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    wanted = (
        "t", "x", "y", "v", "w", "raw_v", "raw_w", "nav_v", "nav_w",
        "safe_v", "safe_w", "steering_deg", "steering_target_deg",
        "action_result_observed", "remaining_path_m",
    )
    rows: list[dict[str, float]] = []
    with args.csv.open(newline="", encoding="utf-8") as stream:
        for source in csv.DictReader(stream):
            row: dict[str, float] = {}
            valid = True
            for key in wanted:
                value = finite(source.get(key))
                if value is None:
                    valid = False
                    break
                row[key] = value
            if valid:
                rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"insufficient finite rows in {args.csv}")

    motion_index = next((i for i, row in enumerate(rows) if abs(row["safe_v"]) >= 0.02), 0)
    motion_time = rows[motion_index]["t"]
    action_index = next((i for i, row in enumerate(rows)
                         if row["action_result_observed"] >= 0.5), len(rows) - 1)
    action_time = rows[action_index]["t"]
    moving = rows[motion_index:action_index + 1]

    evaluation = None
    path_length = None
    if args.evaluation and args.evaluation.exists():
        evaluation = json.loads(args.evaluation.read_text(encoding="utf-8"))
        path_length = finite(evaluation.get("path", {}).get("length_m"))

    command_actual_error = [row["safe_v"] - row["v"] for row in moving]
    accel_report = {}
    for field in ("raw_v", "nav_v", "safe_v", "v"):
        values = rates(moving, field, held_command=(field != "v"))
        accel_report[field] = {
            "positive_p95_mps2": percentile((v for v in values if v > 0.0), 0.95),
            "positive_max_mps2": max((v for v in values if v > 0.0), default=None),
            "negative_p05_mps2": percentile((v for v in values if v < 0.0), 0.05),
            "negative_min_mps2": min((v for v in values if v < 0.0), default=None),
        }
    angular_slew_report = {}
    for field in ("raw_w", "nav_w", "safe_w", "w"):
        values = rates(moving, field, held_command=(field != "w"))
        angular_slew_report[field] = {
            "abs_p95_radps2": percentile((abs(v) for v in values), 0.95),
            "abs_max_radps2": max((abs(v) for v in values), default=None),
            "positive_max_radps2": max((v for v in values if v > 0.0), default=None),
            "negative_min_radps2": min((v for v in values if v < 0.0), default=None),
        }

    thresholds = {
        str(threshold): {
            field: first_crossing(rows, field, threshold, motion_time)
            for field in ("nav_v", "safe_v", "v")
        }
        for threshold in (0.20, 0.30, 0.40, 0.50)
    }

    safe_stop_index = first_sustained_below(rows, "safe_v", 0.02, action_index)
    actual_stop_index = first_sustained_below(rows, "v", 0.02, action_index)
    braking = None
    if safe_stop_index is not None and actual_stop_index is not None:
        braking = {
            "safe_stop_t": rows[safe_stop_index]["t"],
            "actual_stop_t": rows[actual_stop_index]["t"],
            "safe_to_actual_stop_sec": rows[actual_stop_index]["t"] - rows[safe_stop_index]["t"],
            "safe_to_actual_stop_distance_m": distance(rows[safe_stop_index], rows[actual_stop_index]),
            "action_to_actual_stop_sec": rows[actual_stop_index]["t"] - action_time,
            "action_to_actual_stop_distance_m": distance(rows[action_index], rows[actual_stop_index]),
        }

    def terminal_brake_onset(field: str) -> int | None:
        """Locate the terminal braking event, not an earlier MPPI fluctuation.

        The search is restricted to the final 0.75 m when remaining-path data
        exists (otherwise the last four seconds).  It finds the earliest point
        near the terminal-region speed maximum which precedes a meaningful
        drop.  This makes v0 explicit and avoids comparing runs which entered
        the terminal event at different speeds.
        """
        search_start = motion_index
        remaining_candidates = [
            i for i in range(motion_index, action_index + 1)
            if math.isfinite(rows[i]["remaining_path_m"]) and
            rows[i]["remaining_path_m"] <= 0.75
        ]
        if remaining_candidates:
            search_start = remaining_candidates[0]
        else:
            threshold_time = action_time - 4.0
            search_start = next((i for i in range(motion_index, action_index + 1)
                                 if rows[i]["t"] >= threshold_time), motion_index)
        region = rows[search_start:action_index + 1]
        if not region:
            return None
        peak = max(row[field] for row in region)
        if peak < 0.12:
            return None
        for i in range(search_start, action_index + 1):
            value = rows[i][field]
            later_min = min(row[field] for row in rows[i:action_index + 1])
            if value >= 0.95 * peak and value - later_min >= 0.08:
                return i
        return None

    def event(index: int | None) -> dict[str, float] | None:
        if index is None:
            return None
        row = rows[index]
        result = {key: row[key] for key in (
            "t", "x", "y", "raw_v", "nav_v", "safe_v", "v",
            "remaining_path_m")}
        if path_length is not None and math.isfinite(row["remaining_path_m"]):
            result["path_s_m"] = max(0.0, path_length - row["remaining_path_m"])
        return result

    raw_onset = terminal_brake_onset("raw_v")

    def first_command_drop_after(field: str, start: int | None) -> int | None:
        if start is None:
            return None
        previous = rows[start][field]
        for i in range(start + 1, action_index + 1):
            value = rows[i][field]
            if value < previous - 1.0e-4:
                return i
            if not math.isclose(value, previous, rel_tol=0.0, abs_tol=1.0e-12):
                previous = value
        return None

    def first_actual_sustained_drop_after(start: int | None) -> int | None:
        if start is None:
            return None
        for i in range(start, action_index + 1):
            limit = rows[i]["t"] + 0.15
            j = i + 1
            while j <= action_index and rows[j]["t"] < limit:
                j += 1
            if j <= action_index and rows[i]["v"] - rows[j]["v"] >= 0.01:
                return i
        return None

    onset_indices = {
        "raw_v": raw_onset,
        "nav_v": first_command_drop_after("nav_v", raw_onset),
        "safe_v": first_command_drop_after("safe_v", raw_onset),
        "v": first_actual_sustained_drop_after(raw_onset),
    }
    aligned_braking: dict[str, object] = {
        "definition": (
            "terminal-region near-peak preceding >=0.08 m/s drop; region is "
            "remaining_path<=0.75 m when available"),
        "onset": {field: event(index) for field, index in onset_indices.items()},
        "actual_stop": event(actual_stop_index),
        "safe_zero": event(safe_stop_index),
    }
    if actual_stop_index is not None:
        comparisons: dict[str, object] = {}
        for field, index in onset_indices.items():
            if index is None or index >= actual_stop_index:
                comparisons[field] = None
                continue
            elapsed = rows[actual_stop_index]["t"] - rows[index]["t"]
            travelled = distance(rows[index], rows[actual_stop_index])
            v0 = rows[index]["v"]
            comparisons[field] = {
                "onset_actual_v0_mps": v0,
                "to_actual_stop_sec": elapsed,
                "to_actual_stop_distance_m": travelled,
                "effective_decel_from_v0_distance_mps2": (
                    max(0.0, v0 * v0 - 0.02 * 0.02) / (2.0 * travelled)
                    if travelled > 1.0e-6 else None),
                "effective_decel_from_v0_time_mps2": (
                    max(0.0, v0 - 0.02) / elapsed if elapsed > 1.0e-6 else None),
            }
        aligned_braking["onset_to_actual_stop"] = comparisons

    terminal_start_candidates = [index for index in onset_indices.values()
                                 if index is not None]
    if terminal_start_candidates:
        terminal_start = min(terminal_start_candidates)
        terminal_end = safe_stop_index if safe_stop_index is not None else action_index
        terminal_rows = rows[terminal_start:max(terminal_start + 2, terminal_end + 1)]
        observed_terminal_decel = {}
        for field in ("raw_v", "nav_v", "safe_v", "v"):
            values = [value for value in rates(
                terminal_rows, field, held_command=(field != "v")) if value < 0.0]
            item = {
                "negative_p10_mps2": percentile(values, 0.10),
                "negative_min_mps2": min(values, default=None),
                "sample_count": len(values),
            }
            if field != "v":
                steps = [value for value in held_command_steps(
                    terminal_rows, field) if value < 0.0]
                item.update({
                    "negative_step_p10_mps": percentile(steps, 0.10),
                    "negative_step_min_mps": min(steps, default=None),
                    "update_step_count": len(steps),
                })
            observed_terminal_decel[field] = item
        aligned_braking["observed_terminal_deceleration"] = observed_terminal_decel
        nav_safe = [row["nav_v"] - row["safe_v"] for row in terminal_rows]
        aligned_braking["nav_to_safe"] = {
            "rmse_mps": math.sqrt(fmean(value * value for value in nav_safe)),
            "max_abs_mps": max(abs(value) for value in nav_safe),
        }

    steering_target_rates = rates(moving, "steering_target_deg", held_command=True)
    steering_actual_rates = rates(moving, "steering_deg")
    report: dict[str, object] = {
        "csv": str(args.csv),
        "sample_count": len(rows),
        "motion_start_t": motion_time,
        "action_result_t": action_time,
        "commanded_motion_duration_sec": action_time - motion_time,
        "speed": {
            "raw_peak_mps": max(row["raw_v"] for row in moving),
            "nav_peak_mps": max(row["nav_v"] for row in moving),
            "safe_peak_mps": max(row["safe_v"] for row in moving),
            "actual_peak_mps": max(row["v"] for row in moving),
            "safe_minus_actual_rmse_mps": math.sqrt(
                fmean(value * value for value in command_actual_error)),
            "safe_minus_actual_abs_p95_mps": percentile(
                (abs(value) for value in command_actual_error), 0.95),
            "fixed_threshold_rise": thresholds,
        },
        "observed_slew": accel_report,
        "observed_angular_slew": angular_slew_report,
        "braking": braking,
        "aligned_terminal_braking": aligned_braking,
        "steering": {
            "target_rate_abs_p95_degps": percentile(
                (abs(v) for v in steering_target_rates), 0.95),
            "target_rate_abs_max_degps": max(
                (abs(v) for v in steering_target_rates), default=None),
            "actual_rate_abs_p95_degps": percentile(
                (abs(v) for v in steering_actual_rates), 0.95),
            "actual_rate_abs_max_degps": max(
                (abs(v) for v in steering_actual_rates), default=None),
        },
    }
    if evaluation is not None:
        report["tracking"] = evaluation.get("mppi_actual")
        report["segmented_tracking"] = evaluation.get("segmented_metrics")
        report["velocity_chain"] = evaluation.get("velocity_chain")
    if args.summary and args.summary.exists():
        summary = json.loads(args.summary.read_text(encoding="utf-8"))
        report["result"] = summary.get("result")
        report["settle_reason"] = summary.get("settle_reason")
        report["action_result_pose"] = summary.get("action_result_pose")
        report["settled_pose"] = summary.get("settled_pose")

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
