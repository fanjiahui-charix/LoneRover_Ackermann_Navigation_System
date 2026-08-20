#!/usr/bin/env python3
"""Generate conservative full-loop Tube candidates.

Inner and Outer are displaced through
every straight and corner.  The offset is position-dependent: a map/footprint
search finds the available lateral room at each sample, the hard curvature
contract caps tight inner bends, and circular smoothing turns those caps into a
continuous offset profile.  Outputs are candidate-only unless a later explicit
promotion command copies one admitted set into the runtime asset directory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
from pathlib import Path

import numpy as np
from scipy.optimize import minimize

from channel_asset_common import (
    HARD_MINIMUM_RADIUS_M,
    audit_path,
    cyclic_geometry,
    load_map,
    manifest_file_rows,
    normalize,
    pose_clearance,
    read_path_csv,
    sha256_file,
    write_tube,
)


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src/hobot_navigation/hobot_nav"
DEFAULT_CENTER_DIR = PACKAGE / "config/channel_tubes"
DEFAULT_MAP = PACKAGE / "maps/rdk_2026_hospital_static_1cm.pgm"
DEFAULT_OUTPUT = WORKSPACE / "logs/channel_tube_candidates"
MAX_OFFSET_M = 0.12
SMOOTHING_LENGTH_M = 0.12
SEARCH_ITERATIONS = 14
PREFERRED_MINIMUM_RADIUS_M = 0.40
QUANTIZATION_RESERVE_M = 0.005
MAP_CAP_AUDIT_RESERVE_M = 0.005
REPAIR_WINDOW_M = 0.18
MAX_REPAIR_ITERATIONS = 40
MIN_USEFUL_INNER_CURVE_OFFSET_M = 0.02
MIN_GLOBAL_HARD_CAP_MEDIAN_UTILIZATION = 0.40
MIN_DESIGN_CAP_MEDIAN_UTILIZATION = 0.80
MIN_LONG_STRAIGHT_MEDIAN_UTILIZATION = 0.75


def lateral_pose(row: dict[str, float | str], signed_left_offset_m: float) -> tuple[float, float, float]:
    yaw = float(row["yaw"])
    return (
        float(row["x"]) - signed_left_offset_m * math.sin(yaw),
        float(row["y"]) + signed_left_offset_m * math.cos(yaw),
        yaw,
    )


def map_offset_caps(center: list[dict[str, float | str]], grid,
                    left_sign: float, required_margin_m: float) -> np.ndarray:
    caps = np.zeros(len(center), dtype=float)
    for index, row in enumerate(center):
        low = 0.0
        high = MAX_OFFSET_M
        for _ in range(SEARCH_ITERATIONS):
            middle = 0.5 * (low + high)
            x_m, y_m, yaw_rad = lateral_pose(row, left_sign * middle)
            free, clearance = pose_clearance(grid, x_m, y_m, yaw_rad)
            if free and clearance + 1.0e-9 >= required_margin_m:
                low = middle
            else:
                high = middle
        caps[index] = low
    return caps


def curvature_offset_caps(center: list[dict[str, float | str]],
                          left_sign: float, hard_rmin_m: float,
                          quantization_reserve_m: float) -> np.ndarray:
    caps = np.full(len(center), MAX_OFFSET_M, dtype=float)
    for index, row in enumerate(center):
        curvature = float(row["kappa"])
        # For the left-normal offset convention, k_offset=k/(1-d*k).
        # Only offsets toward the instantaneous curvature center tighten R.
        if left_sign * curvature <= 1.0e-12:
            continue
        magnitude = abs(curvature)
        guarded_radius = hard_rmin_m + quantization_reserve_m
        available = (1.0 - magnitude * guarded_radius) / magnitude
        caps[index] = max(0.0, min(MAX_OFFSET_M, available))
    return caps


def bounded_periodic_smooth(target: np.ndarray, upper: np.ndarray,
                            sample_step_m: float, initial: np.ndarray | None = None,
                            lower: np.ndarray | None = None,
                            smoothing_length_m: float = SMOOTHING_LENGTH_M) -> np.ndarray:
    """Return a smooth bounded candidate; exact geometry remains the gate.

    The quadratic derivative terms are deliberately only a candidate generator.
    They are not interpreted as vehicle constraints and never replace the dense
    footprint/curvature audit performed after regenerating the SE(2) path.
    """
    ds = max(sample_step_m, 1.0e-4)
    first_weight = smoothing_length_m ** 2
    second_weight = smoothing_length_m ** 4
    lower_bound = np.zeros_like(upper) if lower is None else lower
    start = np.minimum(
        np.maximum(target if initial is None else initial, lower_bound), upper,
    )

    def objective(value: np.ndarray) -> tuple[float, np.ndarray]:
        error = value - target
        previous = np.roll(value, 1)
        following = np.roll(value, -1)
        d1 = (following - value) / ds
        d2 = (following - 2.0 * value + previous) / (ds * ds)
        loss = ds * (
            float(np.dot(error, error)) +
            first_weight * float(np.dot(d1, d1)) +
            second_weight * float(np.dot(d2, d2))
        )
        first_gradient = (
            2.0 * value - previous - following
        ) / (ds * ds)
        second_gradient = (
            np.roll(value, 2) - 4.0 * previous + 6.0 * value -
            4.0 * following + np.roll(value, -2)
        ) / (ds ** 4)
        gradient = 2.0 * ds * (
            error + first_weight * first_gradient + second_weight * second_gradient
        )
        return loss, gradient

    result = minimize(
        objective, start, method="L-BFGS-B", jac=True,
        bounds=[(float(floor), float(limit))
                for floor, limit in zip(lower_bound, upper)],
        options={"maxiter": 600, "ftol": 1.0e-13, "gtol": 1.0e-9,
                 "maxls": 40},
    )
    if not result.success and not np.all(np.isfinite(result.x)):
        raise RuntimeError(f"bounded smoothing failed: {result.message}")
    return np.minimum(
        np.maximum(np.asarray(result.x, dtype=float), lower_bound), upper,
    )


def offset_geometry(center: list[dict[str, float | str]], left_sign: float,
                    offset_m: np.ndarray) -> list[dict[str, float]]:
    x: list[float] = []
    y: list[float] = []
    for row, magnitude in zip(center, offset_m):
        px, py, _ = lateral_pose(row, left_sign * float(magnitude))
        # Audit the exact coordinates that will be serialized into the runtime
        # CSV; six-decimal quantization must not create a hidden R<0.35 defect.
        x.append(round(px, 6))
        y.append(round(py, 6))
    return cyclic_geometry(x, y)


def admitted_with_margin(report: dict[str, object], margin_m: float,
                         hard_rmin_m: float) -> bool:
    return (
        bool(report["admitted"]) and
        float(report["minimum_radius_m"]) + 1.0e-6 >= hard_rmin_m and
        float(report["minimum_static_footprint_clearance_m"]) + 1.0e-6 >= margin_m
    )


def exact_violation_indices(center: list[dict[str, float | str]], grid,
                            rows: list[dict[str, float]], required_margin_m: float,
                            hard_rmin_m: float,
                            dense_step_m: float = 0.003) -> tuple[set[int], dict[str, int]]:
    """Locate exact regenerated-path violations without a global correction."""
    bad: set[int] = set()
    curvature_count = 0
    clearance_count = 0
    hard_curvature = 1.0 / hard_rmin_m
    for index, row in enumerate(rows):
        if abs(float(row["kappa"])) > hard_curvature + 1.0e-6:
            bad.add(index)
            curvature_count += 1

    count = len(rows)
    for index in range(count):
        following = (index + 1) % count
        start = rows[index]
        finish = rows[following]
        length = math.hypot(
            float(finish["x"]) - float(start["x"]),
            float(finish["y"]) - float(start["y"]),
        )
        samples = max(1, int(math.ceil(length / dense_step_m)))
        yaw_delta = normalize(float(finish["yaw"]) - float(start["yaw"]))
        for sample in range(samples):
            ratio = sample / samples
            free, clearance = pose_clearance(
                grid,
                float(start["x"]) + ratio * (float(finish["x"]) - float(start["x"])),
                float(start["y"]) + ratio * (float(finish["y"]) - float(start["y"])),
                normalize(float(start["yaw"]) + ratio * yaw_delta),
            )
            if not free or clearance + 1.0e-9 < required_margin_m:
                bad.update((index, following))
                clearance_count += 1
    return bad, {
        "curvature_violation_points": curvature_count,
        "clearance_violation_samples": clearance_count,
    }


def apply_local_contraction(target: np.ndarray, offsets: np.ndarray,
                            bad_indices: set[int], sample_step_m: float,
                            iteration: int) -> np.ndarray:
    repaired = target.copy()
    radius = max(3, int(math.ceil(REPAIR_WINDOW_M / sample_step_m)))
    # Escalate locally if the same area survives another exact audit.  This is
    # intentionally not a whole-loop multiplier.
    contraction = min(0.45, 0.12 + 0.025 * iteration)
    count = len(repaired)
    influence = np.zeros(count, dtype=float)
    for bad_index in bad_indices:
        for delta in range(-radius, radius + 1):
            taper = 0.5 * (1.0 + math.cos(math.pi * abs(delta) / radius))
            index = (bad_index + delta) % count
            influence[index] = max(influence[index], taper)
    local_limit = offsets * (1.0 - contraction * influence)
    repaired = np.minimum(repaired, np.maximum(0.0, local_limit))
    return repaired


def locally_expand(center: list[dict[str, float | str]], grid,
                   left_sign: float, offsets: np.ndarray, original_cap: np.ndarray,
                   required_margin_m: float, hard_rmin_m: float,
                   sample_step_m: float) -> tuple[np.ndarray, int]:
    """Recover room one low-curvature region at a time with exact acceptance."""
    accepted = 0
    current = offsets.copy()
    count = len(current)
    straight = [abs(float(row["kappa"])) < 0.05 for row in center]
    runs: list[list[int]] = []
    start = next((index for index, value in enumerate(straight) if not value), 0)
    active: list[int] = []
    for offset in range(1, count + 1):
        index = (start + offset) % count
        if straight[index]:
            active.append(index)
        elif active:
            runs.append(active)
            active = []
    if active:
        runs.append(active)

    no_expansion_rounds = 0
    for _round in range(6):
        accepted_this_round = 0
        for run in runs:
            if len(run) < max(8, int(math.ceil(1.20 / sample_step_m))):
                continue
            proposal = current.copy()
            ramp_count = min(
                max(4, int(math.ceil(0.16 / sample_step_m))),
                max(4, len(run) // 3),
            )
            interior = run[ramp_count:-ramp_count] or run
            plateau_limit = float(np.percentile(original_cap[interior], 10))
            plateau_target = min(
                plateau_limit,
                float(np.median(current[interior])) + 0.006,
            )
            for position, index in enumerate(run):
                edge = min(position + 1, len(run) - position)
                ratio = min(1.0, edge / max(ramp_count, 1))
                taper = ratio ** 3 * (10.0 - 15.0 * ratio + 6.0 * ratio ** 2)
                desired = min(original_cap[index], plateau_target)
                proposal[index] = current[index] + taper * (desired - current[index])
            if float(np.max(proposal - current)) < 0.001:
                continue
            rows = offset_geometry(center, left_sign, proposal)
            bad, _ = exact_violation_indices(
                center, grid, rows, required_margin_m, hard_rmin_m,
            )
            if bad:
                continue
            report = audit_path(grid, rows, closed=True, dense_step_m=0.003)
            if admitted_with_margin(report, required_margin_m, hard_rmin_m):
                current = proposal
                accepted += 1
                accepted_this_round += 1
        if accepted_this_round == 0:
            no_expansion_rounds += 1
        else:
            no_expansion_rounds = 0
        if no_expansion_rounds >= 2:
            break
    return current, accepted


def fit_admitted_profile(center: list[dict[str, float | str]], grid,
                         left_sign: float, local_cap: np.ndarray,
                         preferred_target: np.ndarray,
                         locked_profile: np.ndarray,
                         required_margin_m: float, hard_rmin_m: float,
                         sample_step_m: float,
                         max_repair_iterations: int) -> tuple[
                             np.ndarray, list[dict[str, float]], dict, dict]:
    # Preserve the pointwise cap exactly.  A blanket 0.998 multiplier is still
    # a global scaling operation and, at cap transitions, can inject curvature
    # into an otherwise constant turn profile.  Dense exact audit and local
    # repair provide the required reserve instead.
    original_cap = np.maximum(local_cap, 0.0)
    repair_target = np.minimum(preferred_target, original_cap)
    repair_upper = original_cap.copy()
    repair_lower = np.zeros_like(original_cap)
    # The complete turn influence region (including short tangents between
    # adjacent corners) already has a curvature-safe structured profile.  Map
    # repairs on the long straights must not bleed back into those samples and
    # silently spend the 0.35 m hard-radius reserve.
    repair_upper[locked_profile] = repair_target[locked_profile]
    repair_lower[locked_profile] = repair_target[locked_profile]
    offsets = repair_target.copy()
    repair_history: list[dict[str, int]] = []
    for iteration in range(max_repair_iterations):
        rows = offset_geometry(center, left_sign, offsets)
        bad, counts = exact_violation_indices(
            center, grid, rows, required_margin_m, hard_rmin_m,
        )
        repair_history.append({
            "iteration": iteration,
            "violating_profile_points": len(bad),
            **counts,
        })
        if not bad:
            report = audit_path(grid, rows, closed=True, dense_step_m=0.003)
            if admitted_with_margin(report, required_margin_m, hard_rmin_m):
                break
        if not bad:
            raise RuntimeError("exact audit failed without a localized violation")
        repair_target = apply_local_contraction(
            repair_target, offsets, bad, sample_step_m, iteration,
        )
        repair_target[locked_profile] = preferred_target[locked_profile]
        offsets = bounded_periodic_smooth(
            repair_target, repair_upper, sample_step_m, initial=offsets,
            lower=repair_lower,
        )
    else:
        raise RuntimeError(
            f"local repair did not converge after {max_repair_iterations} iterations"
        )

    offsets, expansion_count = locally_expand(
        center, grid, left_sign, offsets, original_cap,
        required_margin_m, hard_rmin_m, sample_step_m,
    )
    rows = offset_geometry(center, left_sign, offsets)
    bad, counts = exact_violation_indices(
        center, grid, rows, required_margin_m, hard_rmin_m,
    )
    report = audit_path(grid, rows, closed=True, dense_step_m=0.003)
    if bad or not admitted_with_margin(report, required_margin_m, hard_rmin_m):
        raise RuntimeError(f"post-expansion exact audit failed: {counts}")
    solver = {
        "method": "local_cap_iterative_local_repair_and_reexpansion",
        "global_scale_used": False,
        "repair_iterations": len(repair_history) - 1,
        "accepted_local_expansions": expansion_count,
        "repair_history": repair_history,
    }
    return offsets, rows, report, solver


def distribution(values: np.ndarray) -> dict[str, float]:
    if len(values) == 0:
        return {"p10": 0.0, "median": 0.0, "p90": 0.0}
    return {
        "p10": round(float(np.percentile(values, 10)), 6),
        "median": round(float(np.median(values)), 6),
        "p90": round(float(np.percentile(values, 90)), 6),
    }


def region_name(row: dict[str, float | str], center_x: float,
                center_y: float) -> str:
    x_m = float(row["x"])
    y_m = float(row["y"])
    yaw = float(row["yaw"])
    if abs(float(row["kappa"])) < 0.05:
        if abs(math.cos(yaw)) >= abs(math.sin(yaw)):
            return "top_straight" if y_m >= center_y else "bottom_straight"
        return "right_straight" if x_m >= center_x else "left_straight"
    vertical = "top" if y_m >= center_y else "bottom"
    horizontal = "right" if x_m >= center_x else "left"
    return f"{vertical}_{horizontal}_corner"


def region_statistics(center: list[dict[str, float | str]], offsets: np.ndarray,
                      utilization: np.ndarray) -> dict[str, dict[str, object]]:
    center_x = float(np.mean([float(row["x"]) for row in center]))
    center_y = float(np.mean([float(row["y"]) for row in center]))
    buckets: dict[str, list[int]] = {}
    for index, row in enumerate(center):
        buckets.setdefault(region_name(row, center_x, center_y), []).append(index)
    output: dict[str, dict[str, object]] = {}
    for name, indices in sorted(buckets.items()):
        local_offsets = offsets[indices]
        local_utilization = utilization[indices]
        output[name] = {
            "sample_count": len(indices),
            "offset_m": {
                "minimum": round(float(np.min(local_offsets)), 6),
                "median": round(float(np.median(local_offsets)), 6),
                "maximum": round(float(np.max(local_offsets)), 6),
            },
            "offset_utilization": distribution(local_utilization),
        }
    return output


def straightened_design_caps(center: list[dict[str, float | str]],
                             design_caps: np.ndarray, hard_caps: np.ndarray,
                             sample_step_m: float) -> np.ndarray:
    """Build constant arcs/straights joined by long quintic straight ramps."""
    count = len(center)
    curved = [abs(float(row["kappa"])) >= 0.15 for row in center]
    start = next(
        (index for index in range(count) if curved[index] != curved[index - 1]), 0,
    )
    runs: list[tuple[bool, list[int]]] = []
    active: list[int] = []
    active_kind = curved[start]
    for offset in range(count):
        index = (start + offset) % count
        if curved[index] != active_kind:
            runs.append((active_kind, active))
            active = []
            active_kind = curved[index]
        active.append(index)
    if active:
        runs.append((active_kind, active))

    output = np.zeros_like(design_caps)
    for is_curved, run in runs:
        if is_curved:
            # Constant d on a constant-radius arc preserves the analytic offset
            # radius and removes d'/d'' curvature injection inside the corner.
            output[run] = float(np.min(design_caps[run]))
    requested_ramp = max(4, int(math.ceil(0.60 / sample_step_m)))
    for run_index, (is_curved, run) in enumerate(runs):
        if is_curved or len(run) >= 2 * requested_ramp:
            continue
        previous_kind, previous_run = runs[(run_index - 1) % len(runs)]
        following_kind, following_run = runs[(run_index + 1) % len(runs)]
        if previous_kind and following_kind:
            # Treat corner--short-tangent--corner as one turn complex.  A
            # constant value over only the short tangent would still leave an
            # offset discontinuity at each end when the two corner caps differ.
            cluster = [*previous_run, *run, *following_run]
            hold = float(np.min(design_caps[cluster]))
            output[cluster] = np.minimum(design_caps[cluster], hold)
    for is_curved, run in runs:
        if is_curved:
            continue
        left_value = float(output[(run[0] - 1) % count])
        right_value = float(output[(run[-1] + 1) % count])
        plateau = float(np.min(design_caps[run]))
        if len(run) < 2 * requested_ramp:
            # A short tangent between adjacent corners is not usable offset
            # expansion room.  Trying to reach its map cap and return within
            # a few decimetres injects a large d'' term into the regenerated
            # path even though every point remains below its scalar cap.  Hold
            # the smaller neighbouring corner offset through the whole run;
            # the long straights are where side separation is recovered.
            hold = min(left_value, right_value, plateau)
            output[run] = np.minimum(design_caps[run], hold)
            continue
        ramp = min(requested_ramp, max(1, len(run) // 2))
        for position, index in enumerate(run):
            if position < ramp:
                ratio = position / max(ramp - 1, 1)
                blend = ratio ** 3 * (10.0 - 15.0 * ratio + 6.0 * ratio ** 2)
                value = left_value + blend * (plateau - left_value)
            elif position >= len(run) - ramp:
                ratio = (len(run) - 1 - position) / max(ramp - 1, 1)
                blend = ratio ** 3 * (10.0 - 15.0 * ratio + 6.0 * ratio ** 2)
                value = right_value + blend * (plateau - right_value)
            else:
                value = plateau
            output[index] = min(value, float(design_caps[index]))
    return np.minimum(output, hard_caps)


def structured_profile_lock_mask(center: list[dict[str, float | str]],
                                 sample_step_m: float) -> np.ndarray:
    """Protect curvature-safe turns from unrelated straight map repairs."""
    count = len(center)
    turning = np.asarray([
        abs(float(row["kappa"])) >= 0.01 for row in center
    ], dtype=bool)
    locked = turning.copy()

    # There are short tangents between neighbouring corners on this channel.
    # They cannot accommodate a useful out-and-back lateral transition, so the
    # structured generator holds the corner value there and repair keeps it.
    start = next(
        (index for index in range(count) if turning[index] != turning[index - 1]), 0,
    )
    active: list[int] = []
    active_kind = bool(turning[start])
    runs: list[tuple[bool, list[int]]] = []
    for offset in range(count):
        index = (start + offset) % count
        if bool(turning[index]) != active_kind:
            runs.append((active_kind, active))
            active = []
            active_kind = bool(turning[index])
        active.append(index)
    if active:
        runs.append((active_kind, active))
    for is_turning, run in runs:
        if not is_turning and len(run) * sample_step_m < 1.20:
            locked[run] = True

    # Preserve the first/last part of each long-straight quintic transition.
    # This gives the optimizer a fixed C2-compatible boundary while leaving
    # the central straight available for local clearance repairs/expansion.
    guard = max(2, int(math.ceil(0.15 / sample_step_m)))
    turning_indices = np.flatnonzero(turning)
    for index in turning_indices:
        for delta in range(-guard, guard + 1):
            locked[(int(index) + delta) % count] = True
    return locked


def preferred_radius_statistics(rows: list[dict[str, float]],
                                preferred_rmin_m: float) -> dict[str, object]:
    segment_lengths = np.asarray([
        math.hypot(
            float(rows[(index + 1) % len(rows)]["x"]) - float(row["x"]),
            float(rows[(index + 1) % len(rows)]["y"]) - float(row["y"]),
        ) for index, row in enumerate(rows)
    ])
    curvature = np.asarray([abs(float(row["kappa"])) for row in rows])
    radii = np.divide(
        1.0, curvature, out=np.full_like(curvature, np.inf), where=curvature > 1.0e-12,
    )
    minimum_index = int(np.argmin(radii))
    below = radii + 1.0e-6 < preferred_rmin_m
    total_length = float(np.sum(segment_lengths))
    below_length = float(np.sum(segment_lengths[below]))
    return {
        "preferred_minimum_radius_m": preferred_rmin_m,
        "minimum_radius_m": round(float(radii[minimum_index]), 6),
        "minimum_radius_index": minimum_index,
        "minimum_radius_s_m": round(float(rows[minimum_index]["s"]), 6),
        "length_with_radius_below_preferred_m": round(below_length, 6),
        "percentage_with_radius_below_preferred": round(
            100.0 * below_length / max(total_length, 1.0e-9), 4,
        ),
    }


def active_constraints(offsets: np.ndarray, raw_map_caps: np.ndarray,
                       dynamic_map_caps: np.ndarray,
                       design_local_caps: np.ndarray,
                       unsmoothed_design_caps: np.ndarray) -> list[str]:
    labels: list[str] = []
    for value, raw_map, dynamic_map, design_cap, unsmoothed_cap in zip(
            offsets, raw_map_caps, dynamic_map_caps, design_local_caps,
            unsmoothed_design_caps):
        tolerance = max(0.001, 0.04 * max(design_cap, 0.001))
        if design_cap + 0.0005 < unsmoothed_cap:
            labels.append("SMOOTHNESS")
        elif dynamic_map + 0.0005 < raw_map and value + tolerance >= dynamic_map:
            labels.append("DYNAMIC_MARGIN")
        elif design_cap + 0.0005 < dynamic_map and value + tolerance >= design_cap:
            labels.append("RMIN")
        elif raw_map <= design_cap + 0.0005 and value + tolerance >= raw_map:
            labels.append("MAP")
        else:
            labels.append("SMOOTHNESS")
    return labels


def write_offset_profile(path: Path, center: list[dict[str, float | str]],
                         offsets: np.ndarray, raw_map_caps: np.ndarray,
                         dynamic_map_caps: np.ndarray,
                         hard_curvature_caps: np.ndarray,
                         preferred_curvature_caps: np.ndarray,
                         design_local_caps: np.ndarray,
                         labels: list[str]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow((
            "index", "s_m", "map_cap_m", "dynamic_margin_cap_m",
            "hard_curvature_cap_m", "preferred_curvature_cap_m",
            "hard_local_cap_m", "design_local_cap_m", "final_offset_m",
            "offset_utilization", "design_cap_utilization", "active_constraint",
        ))
        for index, (row, value, raw_map, dynamic_map, hard_curvature,
                    preferred_curvature, local_cap, label) in enumerate(zip(
                center, offsets, raw_map_caps, dynamic_map_caps,
                hard_curvature_caps, preferred_curvature_caps,
                design_local_caps, labels)):
            hard_cap = min(dynamic_map, hard_curvature)
            utilization = value / local_cap if local_cap > 1.0e-9 else 1.0
            hard_utilization = value / hard_cap if hard_cap > 1.0e-9 else 1.0
            writer.writerow((
                index, f"{float(row['s']):.6f}", f"{float(raw_map):.6f}",
                f"{float(dynamic_map):.6f}", f"{float(hard_curvature):.6f}",
                f"{float(preferred_curvature):.6f}", f"{float(hard_cap):.6f}",
                f"{float(local_cap):.6f}", f"{float(value):.6f}",
                f"{float(hard_utilization):.6f}", f"{float(utilization):.6f}", label,
            ))


def generate_candidate(center_dir: Path, map_path: Path, output: Path,
                       required_margin_m: float, hard_rmin_m: float,
                       preferred_rmin_m: float, quantization_reserve_m: float,
                       max_repair_iterations: int) -> dict:
    grid = load_map(map_path)
    tube_dir = output / "tubes"
    profile_dir = output / "offset_profiles"
    tube_dir.mkdir(parents=True, exist_ok=True)
    profile_dir.mkdir(parents=True, exist_ok=True)
    report: dict[str, object] = {
        "schema_version": 2,
        "candidate_only": True,
        "required_dynamic_margin_m": required_margin_m,
        "hard_minimum_radius_m": hard_rmin_m,
        "preferred_minimum_radius_m": preferred_rmin_m,
        "curvature_quantization_reserve_m": quantization_reserve_m,
        "map_cap_audit_reserve_m": MAP_CAP_AUDIT_RESERVE_M,
        "minimum_useful_inner_curve_offset_m": MIN_USEFUL_INNER_CURVE_OFFSET_M,
        "maximum_search_offset_m": MAX_OFFSET_M,
        "smoothing_length_m": SMOOTHING_LENGTH_M,
        "map": {
            "path": str(map_path), "sha256": sha256_file(map_path),
            "width": grid.width, "height": grid.height,
            "resolution_m": grid.resolution_m,
            "origin_m": [grid.origin_x_m, grid.origin_y_m],
        },
        "tubes": {},
    }
    for direction in ("cw", "ccw"):
        source_center = center_dir / f"tube_center_{direction}.csv"
        destination_center = tube_dir / source_center.name
        shutil.copyfile(source_center, destination_center)
        center = read_path_csv(source_center)
        center_report = audit_path(grid, center, closed=True, dense_step_m=0.003)
        report["tubes"][f"tube_center_{direction}"] = {
            **center_report, "offset_min_m": 0.0, "offset_max_m": 0.0,
            "offset_mean_m": 0.0,
            "radius_preference": preferred_radius_statistics(center, preferred_rmin_m),
        }
        lengths = [
            math.hypot(
                float(center[index]["x"]) - float(center[index - 1]["x"]),
                float(center[index]["y"]) - float(center[index - 1]["y"]),
            ) for index in range(1, len(center))
        ]
        sample_step = float(np.median(lengths))
        for lane in ("inner", "outer"):
            # Left normal points into the loop for CCW and out for CW.
            inner_left_sign = 1.0 if direction == "ccw" else -1.0
            left_sign = inner_left_sign if lane == "inner" else -inner_left_sign
            raw_map_caps = map_offset_caps(center, grid, left_sign, 0.0)
            dynamic_map_caps = map_offset_caps(
                center, grid, left_sign,
                required_margin_m + MAP_CAP_AUDIT_RESERVE_M,
            )
            curvature_caps = curvature_offset_caps(
                center, left_sign, hard_rmin_m, quantization_reserve_m,
            )
            preferred_curvature_caps = curvature_offset_caps(
                center, left_sign, preferred_rmin_m, 0.0,
            )
            local_caps = np.minimum(dynamic_map_caps, curvature_caps)
            unsmoothed_preferred_target = np.minimum(
                local_caps,
                np.maximum(
                    preferred_curvature_caps,
                    np.minimum(curvature_caps, MIN_USEFUL_INNER_CURVE_OFFSET_M),
                ),
            )
            preferred_target = straightened_design_caps(
                center, unsmoothed_preferred_target, local_caps, sample_step,
            )
            locked_profile = structured_profile_lock_mask(center, sample_step)
            offsets, rows, tube_report, solver_report = fit_admitted_profile(
                center, grid, left_sign, local_caps, preferred_target, locked_profile,
                required_margin_m,
                hard_rmin_m, sample_step, max_repair_iterations,
            )
            name = f"tube_{lane}_{direction}"
            destination = tube_dir / f"{name}.csv"
            write_tube(destination, rows, lane, direction)
            labels = active_constraints(
                offsets, raw_map_caps, dynamic_map_caps, preferred_target,
                unsmoothed_preferred_target,
            )
            write_offset_profile(
                profile_dir / f"{name}_offset.csv", center, offsets,
                raw_map_caps, dynamic_map_caps, curvature_caps,
                preferred_curvature_caps, preferred_target, labels,
            )
            separation = [float(value) for value in offsets]
            design_utilization = np.divide(
                offsets, preferred_target, out=np.ones_like(offsets),
                where=preferred_target > 1.0e-9,
            )
            utilization = np.divide(
                offsets, local_caps, out=np.ones_like(offsets),
                where=local_caps > 1.0e-9,
            )
            constraint_counts = {
                label: labels.count(label)
                for label in ("MAP", "RMIN", "SMOOTHNESS", "DYNAMIC_MARGIN")
            }
            regions = region_statistics(center, offsets, utilization)
            utilization_quality = {
                "hard_cap_global_median_minimum":
                    MIN_GLOBAL_HARD_CAP_MEDIAN_UTILIZATION,
                "design_cap_global_median_minimum":
                    MIN_DESIGN_CAP_MEDIAN_UTILIZATION,
                "long_straight_hard_cap_median_minimum":
                    MIN_LONG_STRAIGHT_MEDIAN_UTILIZATION,
                "hard_cap_global_median_pass": bool(
                    float(np.median(utilization)) >=
                    MIN_GLOBAL_HARD_CAP_MEDIAN_UTILIZATION
                ),
                "design_cap_global_median_pass": bool(
                    float(np.median(design_utilization)) >=
                    MIN_DESIGN_CAP_MEDIAN_UTILIZATION
                ),
                "long_straights_pass": all(
                    float(regions[name]["offset_utilization"]["median"]) >=
                    MIN_LONG_STRAIGHT_MEDIAN_UTILIZATION
                    for name in ("top_straight", "bottom_straight")
                    if name in regions
                ),
            }
            utilization_quality["admitted"] = all(
                bool(value) for key, value in utilization_quality.items()
                if key.endswith("_pass")
            )
            report["tubes"][name] = {
                **tube_report,
                "left_normal_sign": left_sign,
                "solver": solver_report,
                "locked_structured_profile_samples": int(np.sum(locked_profile)),
                "offset_min_m": round(min(separation), 6),
                "offset_max_m": round(max(separation), 6),
                "offset_mean_m": round(float(np.mean(offsets)), 6),
                "offset_median_m": round(float(np.median(offsets)), 6),
                "offset_p05_m": round(float(np.percentile(offsets, 5)), 6),
                "offset_p95_m": round(float(np.percentile(offsets, 95)), 6),
                "offset_utilization": distribution(utilization),
                "design_cap_offset_utilization": distribution(design_utilization),
                "offset_utilization_quality": utilization_quality,
                "active_constraint_counts": constraint_counts,
                "radius_preference": preferred_radius_statistics(rows, preferred_rmin_m),
                "regions": regions,
            }

    tube_paths = sorted(tube_dir.glob("tube_*.csv"))
    profile_paths = sorted(profile_dir.glob("tube_*_offset.csv"))
    all_admitted = all(bool(value["admitted"]) for value in report["tubes"].values())
    margin_admitted = all(
        float(value["minimum_static_footprint_clearance_m"]) + 1.0e-6 >= required_margin_m
        for value in report["tubes"].values()
    )
    distinct_sides = all(
        float(value["offset_mean_m"]) > 0.005
        for key, value in report["tubes"].items() if "center" not in key
    )
    utilization_sufficient = all(
        bool(value["offset_utilization_quality"]["admitted"])
        for key, value in report["tubes"].items() if "center" not in key
    )
    report["candidate_admitted"] = (
        all_admitted and margin_admitted and distinct_sides and utilization_sufficient
    )
    report["admission"] = {
        "zero_static_lethal_collision": all_admitted,
        "required_margin_met": margin_admitted,
        "six_paths_present": len(tube_paths) == 6,
        "side_paths_distinct_full_loop": distinct_sides,
        "side_offset_utilization_quality": utilization_sufficient,
        "runtime_promotion_allowed": False,
        "reason": "T2 Center and T3 six-Tube native RPP gates are not yet attached",
    }
    geometry_path = output / "geometry_report.json"
    geometry_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    manifest = {
        "schema_version": 2,
        "asset_kind": "channel_tube_candidate",
        "candidate_only": True,
        "required_dynamic_margin_m": required_margin_m,
        "hard_minimum_radius_m": hard_rmin_m,
        "preferred_minimum_radius_m": preferred_rmin_m,
        "map_sha256": sha256_file(map_path),
        "generator": {
            "path": str(Path(__file__).resolve().relative_to(WORKSPACE)),
            "sha256": sha256_file(Path(__file__).resolve()),
            "common_sha256": sha256_file(Path(__file__).with_name("channel_asset_common.py")),
        },
        "files": manifest_file_rows(output, [*tube_paths, *profile_paths, geometry_path]),
        "promotion_gates": {
            "geometry": bool(report["candidate_admitted"]),
            "center_rpp_shadow": False,
            "six_tube_rpp_shadow": False,
        },
    }
    manifest_path = output / "tube_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--center-dir", type=Path, default=DEFAULT_CENTER_DIR)
    parser.add_argument("--map", type=Path, default=DEFAULT_MAP)
    parser.add_argument("--out-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--margins", type=float, nargs="+", default=(0.02, 0.03, 0.04),
        help="required footprint-to-lethal-map margins in meters",
    )
    parser.add_argument("--hard-rmin", type=float, default=HARD_MINIMUM_RADIUS_M)
    parser.add_argument(
        "--preferred-rmin", type=float, default=PREFERRED_MINIMUM_RADIUS_M,
    )
    parser.add_argument(
        "--curvature-quantization-reserve", type=float,
        default=QUANTIZATION_RESERVE_M,
    )
    parser.add_argument(
        "--max-repair-iterations", type=int, default=MAX_REPAIR_ITERATIONS,
    )
    parser.add_argument(
        "--replace", action="store_true",
        help="explicitly replace an existing candidate root (never the default)",
    )
    args = parser.parse_args()
    formal_roots = {
        (PACKAGE / "config").resolve(), (PACKAGE / "runtime").resolve(),
    }
    resolved_output = args.out_root.resolve()
    if any(root == resolved_output or root in resolved_output.parents for root in formal_roots):
        raise SystemExit("candidate output must not be inside formal config/runtime directories")
    if abs(args.hard_rmin - HARD_MINIMUM_RADIUS_M) > 1.0e-9:
        raise SystemExit(
            f"hard_rmin is the physical contract and must remain {HARD_MINIMUM_RADIUS_M:.2f} m"
        )
    if args.preferred_rmin + 1.0e-9 < args.hard_rmin:
        raise SystemExit("preferred_rmin must be no smaller than hard_rmin")
    if not 0.0 <= args.curvature_quantization_reserve <= 0.02:
        raise SystemExit("curvature quantization reserve must be in [0, 0.02] m")
    if args.max_repair_iterations < 1:
        raise SystemExit("max repair iterations must be positive")
    summary: dict[str, object] = {"candidates": {}}
    for margin in args.margins:
        if margin < 0.0 or margin > 0.08:
            raise SystemExit(f"invalid dynamic margin {margin}; expected 0..0.08 m")
        name = f"margin_{int(round(margin * 1000)):03d}mm"
        output = args.out_root / name
        if output.exists():
            if not args.replace:
                raise SystemExit(f"refusing to overwrite existing candidate: {output}")
            shutil.rmtree(output)
        candidate = generate_candidate(
            args.center_dir, args.map, output, margin,
            args.hard_rmin, args.preferred_rmin,
            args.curvature_quantization_reserve,
            args.max_repair_iterations,
        )
        summary["candidates"][name] = {
            "path": str(output),
            "candidate_admitted": candidate["candidate_admitted"],
            "tube_metrics": candidate["tubes"],
        }
    args.out_root.mkdir(parents=True, exist_ok=True)
    (args.out_root / "candidate_index.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if all(
        bool(value["candidate_admitted"])
        for value in summary["candidates"].values()
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
