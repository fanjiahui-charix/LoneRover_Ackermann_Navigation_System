#!/usr/bin/env python3
"""Evaluate one native Nav2 shadow run against its own Smac path.

This evaluator deliberately does not read the historical ``/plan`` messages
from the calibration bag when computing CTE.  The path supplied with
``--path`` is the path produced by the same native run as ``--csv``.
It also performs a dense (approximately 1 cm) static-map sweep with the
frozen vehicle footprint before judging the MPPI trajectory.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np

VEHICLE_MODEL = Path(__file__).resolve().parents[1] / "vehicle_model"
if str(VEHICLE_MODEL) not in sys.path:
    sys.path.insert(0, str(VEHICLE_MODEL))

from offline_vehicle_response_sim import MapData, deserialize_bag, footprint_samples  # noqa: E402


PI = math.pi
FOOTPRINT_FRONT_M = 0.28
FOOTPRINT_REAR_M = 0.11
FOOTPRINT_HALF_WIDTH_M = 0.13


def wrap(a: float) -> float:
    return (a + PI) % (2.0 * PI) - PI


def load_path(path_file: Path) -> dict[str, Any]:
    obj = json.loads(path_file.read_text(encoding="utf-8"))
    rows = obj.get("path", obj if isinstance(obj, list) else [])
    if len(rows) < 2:
        raise ValueError(f"path has fewer than two poses: {path_file}")
    xy = np.asarray([[float(p["x"]), float(p["y"])] for p in rows], dtype=float)
    yaw = np.unwrap(np.asarray([math.radians(float(p["yaw_deg"])) for p in rows]))
    seg = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    arc = np.concatenate(([0.0], np.cumsum(seg)))
    return {"obj": obj, "xy": xy, "yaw": yaw, "arc": arc}


def make_clearance_grid(map_data: MapData) -> np.ndarray:
    from scipy.ndimage import distance_transform_edt
    free = map_data.occupancy < 50
    distance = distance_transform_edt(free) * map_data.resolution
    return np.maximum(
        0.0, distance - math.sqrt(2.0) * map_data.resolution * 0.5)


def footprint_collision(map_data: MapData, x: float, y: float, yaw: float) -> bool:
    samples = np.asarray(footprint_samples(x, y, yaw), dtype=float)
    ix = np.floor((samples[:, 0] - map_data.origin_x) / map_data.resolution).astype(int)
    iy = np.floor((samples[:, 1] - map_data.origin_y) / map_data.resolution).astype(int)
    if np.any(ix < 0) or np.any(iy < 0) or np.any(ix >= map_data.width) or np.any(iy >= map_data.height):
        return True
    return bool(np.any(map_data.occupancy[iy, ix] >= 50))


def min_clearance(samples: np.ndarray, map_data: MapData,
                  clearance_grid: np.ndarray) -> float:
    if len(samples) == 0:
        return float("inf")
    ix = np.floor((samples[:, 0] - map_data.origin_x) / map_data.resolution).astype(int)
    iy = np.floor((samples[:, 1] - map_data.origin_y) / map_data.resolution).astype(int)
    inside = (
        (ix >= 0) & (iy >= 0) &
        (ix < map_data.width) & (iy < map_data.height))
    if not np.all(inside):
        return 0.0
    return float(np.min(clearance_grid[iy, ix]))


def interpolate_path(p: dict[str, Any], ds: float = 0.01) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xy, yaw, arc = p["xy"], p["yaw"], p["arc"]
    out_s = np.unique(np.concatenate((np.arange(0.0, arc[-1], ds), [arc[-1]])))
    out_x = np.interp(out_s, arc, xy[:, 0])
    out_y = np.interp(out_s, arc, xy[:, 1])
    out_yaw = np.interp(out_s, arc, yaw)
    return out_s, np.column_stack((out_x, out_y)), out_yaw


def sweep_path(p: dict[str, Any], map_data: MapData,
               clearance_grid: np.ndarray, ds: float) -> dict[str, Any]:
    s, xy, yaw = interpolate_path(p, ds)
    collision = np.zeros(len(s), dtype=bool)
    clearance = np.zeros(len(s), dtype=float)
    all_samples: list[tuple[float, float]] = []
    for i, ((x, y), a) in enumerate(zip(xy, yaw)):
        samples = np.asarray(footprint_samples(float(x), float(y), float(a)), dtype=float)
        all_samples.extend(samples.tolist())
        collision[i] = footprint_collision(map_data, float(x), float(y), float(a))
        clearance[i] = min_clearance(samples, map_data, clearance_grid)
    overall_clearance = min_clearance(
        np.asarray(all_samples), map_data, clearance_grid)
    hits = np.flatnonzero(collision)
    return {
        "samples": int(len(s)),
        "sample_spacing_m": float(ds),
        "length_m": float(p["arc"][-1]),
        "collision_count": int(len(hits)),
        "min_clearance_m": float(overall_clearance),
        "first_collision_s_m": float(s[hits[0]]) if len(hits) else None,
        "collision_free": bool(len(hits) == 0),
        "start_valid": bool(not collision[0]),
        "s": s,
        "xy": xy,
        "yaw": yaw,
        "clearance": clearance,
    }


def project_to_path(x: float, y: float, p: dict[str, Any]) -> tuple[float, float, int]:
    xy, arc = p["xy"], p["arc"]
    q = np.asarray([x, y], dtype=float)
    a, b = xy[:-1], xy[1:]
    d = b - a
    den = np.sum(d * d, axis=1)
    u = np.zeros(len(d), dtype=float)
    valid = den > 1.0e-12
    u[valid] = np.clip(np.sum((q - a[valid]) * d[valid], axis=1) / den[valid], 0.0, 1.0)
    proj = a + u[:, None] * d
    dist = np.linalg.norm(proj - q[None, :], axis=1)
    j = int(np.argmin(dist))
    signed = float(-d[j, 1] * (x - proj[j, 0]) + d[j, 0] * (y - proj[j, 1])) / max(float(np.linalg.norm(d[j])), 1.0e-9)
    return float(arc[j] + u[j] * np.linalg.norm(d[j])), signed, j


def yaw_at_s(p: dict[str, Any], s: float) -> float:
    return float(np.interp(s, p["arc"], p["yaw"]))


def path_curvature(p: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    s, xy, yaw = interpolate_path(p, 0.01)
    k = np.zeros(len(s), dtype=float)
    if len(s) > 2:
        k[1:-1] = np.unwrap(yaw)[2:] - np.unwrap(yaw)[:-2]
        k[1:-1] /= np.maximum(s[2:] - s[:-2], 1.0e-6)
    if len(s) > 1:
        k[0] = k[1] if len(s) > 1 else 0.0
        k[-1] = k[-2]
    dk = np.gradient(k, s, edge_order=1) if len(s) > 1 else np.zeros_like(k)
    return s, k, dk


def point_to_footprint_distance(x: float, y: float, yaw: float,
                                point_x: float, point_y: float) -> float:
    """Exact distance from a map point to the frozen rectangular footprint."""
    dx = point_x - x
    dy = point_y - y
    c = math.cos(yaw)
    s = math.sin(yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    excess_x = max(
        -FOOTPRINT_REAR_M - local_x,
        0.0,
        local_x - FOOTPRINT_FRONT_M)
    excess_y = max(
        -FOOTPRINT_HALF_WIDTH_M - local_y,
        0.0,
        local_y - FOOTPRINT_HALF_WIDTH_M)
    return math.hypot(excess_x, excess_y)


def evaluate_fixed_cones(p: dict[str, Any], rows: list[dict[str, float]],
                         hard_radius_m: float,
                         path_step_m: float = 0.002) -> dict[str, Any]:
    """Audit the same injected cone set against nominal and actual footprints."""
    raw_cones = p["obj"].get("fixed_cones_map", [])
    if not raw_cones:
        return {"available": False, "reason": "no_fixed_cones_map"}
    if hard_radius_m <= 0.0:
        raise ValueError("cone hard radius must be positive")
    cones = [(float(item[0]), float(item[1])) for item in raw_cones]
    path_s, path_xy, path_yaw = interpolate_path(p, path_step_m)
    moving = [row for row in rows if abs(row.get("v", 0.0)) > 0.02]
    actual_rows = moving if moving else rows
    per_cone = []
    for cone_index, (cone_x, cone_y) in enumerate(cones):
        path_clearance = np.asarray([
            point_to_footprint_distance(float(x), float(y), float(yaw),
                                        cone_x, cone_y) - hard_radius_m
            for (x, y), yaw in zip(path_xy, path_yaw)
        ])
        path_index = int(np.argmin(path_clearance))
        actual_clearance = np.asarray([
            point_to_footprint_distance(
                row["x"], row["y"], math.radians(row["yaw_deg"]),
                cone_x, cone_y) - hard_radius_m
            for row in actual_rows
        ])
        actual_index = int(np.argmin(actual_clearance))
        actual_row = actual_rows[actual_index]
        per_cone.append({
            "index": cone_index,
            "x": cone_x,
            "y": cone_y,
            "path_min_hard_clearance_m": float(path_clearance[path_index]),
            "path_min_station_m": float(path_s[path_index]),
            "actual_min_hard_clearance_m": float(
                actual_clearance[actual_index]),
            "actual_min_t": float(actual_row["t"]),
            "actual_min_station_m": float(actual_row.get("s_m", float("nan"))),
            "actual_min_cte_m": float(actual_row.get("cte_m", float("nan"))),
            "actual_clearance_loss_from_path_m": float(
                path_clearance[path_index] - actual_clearance[actual_index]),
        })
    path_worst = min(per_cone, key=lambda item: item["path_min_hard_clearance_m"])
    actual_worst = min(per_cone, key=lambda item: item["actual_min_hard_clearance_m"])
    return {
        "available": True,
        "hard_radius_m": hard_radius_m,
        "footprint": {
            "front_m": FOOTPRINT_FRONT_M,
            "rear_m": FOOTPRINT_REAR_M,
            "half_width_m": FOOTPRINT_HALF_WIDTH_M,
        },
        "path_sample_spacing_m": path_step_m,
        "path_min_hard_clearance_m": path_worst["path_min_hard_clearance_m"],
        "path_worst_cone_index": path_worst["index"],
        "actual_min_hard_clearance_m": actual_worst[
            "actual_min_hard_clearance_m"],
        "actual_worst_cone_index": actual_worst["index"],
        "path_collision_free": path_worst["path_min_hard_clearance_m"] >= 0.0,
        "actual_collision_free": actual_worst[
            "actual_min_hard_clearance_m"] >= 0.0,
        "per_cone": per_cone,
    }


def read_actual(csv_file: Path) -> list[dict[str, float]]:
    with csv_file.open(newline="", encoding="utf-8") as f:
        return [{k: float(v) for k, v in row.items() if v != ""} for row in csv.DictReader(f)]


def json_safe(value: Any) -> Any:
    """Replace NaN/Inf command-curvature gaps with JSON null."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    return value


def evaluate_actual(rows: list[dict[str, float]], p: dict[str, Any],
                    map_data: MapData,
                    clearance_grid: np.ndarray,
                    smac_s: np.ndarray,
                    smac_clearance: np.ndarray) -> tuple[dict[str, Any], list[dict[str, float]]]:
    errors: list[float] = []
    signed: list[float] = []
    stations: list[float] = []
    out: list[dict[str, float]] = []
    collisions = 0
    min_clear = float("inf")
    first_collision = None
    first_cte = None
    first_clearance_zero = None
    max_clearance_loss = -float("inf")
    path_s, path_kappa, _ = path_curvature(p)
    for row in rows:
        s, cte, _ = project_to_path(row["x"], row["y"], p)
        a = yaw_at_s(p, s)
        coll = footprint_collision(map_data, row["x"], row["y"], math.radians(row["yaw_deg"]))
        if coll:
            collisions += 1
            if first_collision is None:
                first_collision = {"t": row["t"], "s_m": s, "x": row["x"], "y": row["y"]}
        samples = np.asarray(footprint_samples(row["x"], row["y"], math.radians(row["yaw_deg"])), dtype=float)
        sample_clearance = min_clearance(samples, map_data, clearance_grid)
        path_clearance = float(np.interp(s, smac_s, smac_clearance))
        clearance_loss = path_clearance - sample_clearance
        max_clearance_loss = max(max_clearance_loss, clearance_loss)
        if first_clearance_zero is None and sample_clearance <= 0.0:
            first_clearance_zero = {
                "t": row["t"], "s_m": s,
                "actual_clearance_m": sample_clearance,
                "smac_clearance_m": path_clearance,
            }
        min_clear = min(min_clear, sample_clearance)
        e = abs(cte)
        if first_cte is None and e > 0.05:
            first_cte = {"t": row["t"], "s_m": s, "cte_m": e}
        raw_v, raw_w = row.get("raw_v", float("nan")), row.get("raw_w", float("nan"))
        nav_v, nav_w = row.get("nav_v", float("nan")), row.get("nav_w", float("nan"))
        safe_v, safe_w = row.get("safe_v", float("nan")), row.get("safe_w", float("nan"))
        out.append({
            **row, "s_m": s, "cte_signed_m": cte, "cte_m": e,
            "path_yaw_deg": math.degrees(a),
            "yaw_error_deg": math.degrees(wrap(math.radians(row["yaw_deg"]) - a)),
            "kappa_path": float(np.interp(s, path_s, path_kappa)),
            "kappa_raw": raw_w / raw_v if math.isfinite(raw_v) and abs(raw_v) > 0.01 else float("nan"),
            "kappa_nav": nav_w / nav_v if math.isfinite(nav_v) and abs(nav_v) > 0.01 else float("nan"),
            "kappa_safe": safe_w / safe_v if math.isfinite(safe_v) and abs(safe_v) > 0.01 else float("nan"),
            "kappa_actual": row["w"] / row["v"] if math.isfinite(row.get("v", float("nan"))) and abs(row.get("v", 0.0)) > 0.01 else float("nan"),
            "smac_footprint_clearance_m": path_clearance,
            "footprint_clearance_m": sample_clearance,
            "clearance_loss_m": clearance_loss,
            "footprint_collision": 1.0 if coll else 0.0,
        })
        errors.append(e); signed.append(cte); stations.append(s)
    if not rows:
        raise ValueError(f"empty trajectory: {rows}")
    last = rows[-1]
    final_s, final_cte, _ = project_to_path(last["x"], last["y"], p)
    final_yaw_error = math.degrees(wrap(math.radians(last["yaw_deg"]) - yaw_at_s(p, final_s)))
    return {
        "samples": len(rows), "duration_s": float(last["t"] - rows[0]["t"]),
        "max_cte_m": float(max(errors)), "rms_cte_m": float(math.sqrt(np.mean(np.square(errors)))),
        "signed_cte_final_m": float(final_cte), "terminal_s_m": final_s,
        "terminal_xy_m": float(math.hypot(last["x"] - p["xy"][-1, 0], last["y"] - p["xy"][-1, 1])),
        "terminal_yaw_error_deg": float(final_yaw_error),
        "collision_count": int(collisions), "min_clearance_m": float(min_clear),
        "max_clearance_loss_m": float(max_clearance_loss),
        "first_clearance_le_zero": first_clearance_zero,
        "first_collision": first_collision, "first_cte_over_5cm": first_cte,
    }, out


def steering_path_difficulty(p: dict[str, Any], speed: float,
                             wheelbase: float = 0.144,
                             sustain_distance: float = 0.04) -> dict[str, float]:
    s, kappa, _ = path_curvature(p)
    delta = np.arctan(wheelbase * kappa)
    ddelta_ds = np.gradient(delta, s, edge_order=1)
    required_rate = speed * ddelta_ds
    abs_rate = np.abs(required_rate)
    count = max(1, int(round(sustain_distance / max(np.median(np.diff(s)), 1.0e-6))))
    kernel = np.ones(count, dtype=float) / count
    sustained = np.convolve(abs_rate, kernel, mode="valid")
    return {
        "wheelbase_m": wheelbase,
        "evaluation_speed_mps": speed,
        "sustain_distance_m": sustain_distance,
        "initial_delta_deg": float(delta[0] * 180.0 / math.pi),
        "max_abs_delta_deg": float(np.max(np.abs(delta)) * 180.0 / math.pi),
        "required_delta_dot_p95_radps": float(np.percentile(abs_rate, 95.0)),
        "required_delta_dot_sustained_mean_max_radps": float(np.max(sustained)),
        "required_delta_dot_point_max_radps": float(np.max(abs_rate)),
    }


def historical_effective_steering_rate(
        plans: list[Any], odom: list[tuple], wheelbase: float = 0.144) -> dict[str, Any]:
    """Estimate realized steering-rate capability from the Phase210 Gate run.

    This is deliberately named *effective*: it derives bicycle steering angle
    from measured odometry ``atan(L*w/v)`` and is not a steering-sensor trace.
    A 0.20 s Savitzky-Golay window suppresses encoder/yaw-rate quantization
    before differentiating.  The result is still much stronger evidence than
    comparing against a configured steering-rate constant.
    """
    if len(plans) < 2 or not odom:
        return {"available": False, "reason": "bag_missing_gate_plan_or_odom"}
    from scipy.signal import savgol_filter
    t0 = float(plans[1].stamp)
    rows = [r for r in odom if t0 <= r[0] <= t0 + 12.0 and abs(r[4]) > 0.05]
    if len(rows) < 21:
        return {"available": False, "reason": "insufficient_moving_odom"}
    t = np.asarray([r[0] for r in rows], dtype=float)
    delta = np.arctan(wheelbase * np.asarray([r[5] for r in rows]) /
                      np.asarray([r[4] for r in rows]))
    dt = max(float(np.median(np.diff(t))), 1.0e-3)
    grid = np.arange(t[0], t[-1], dt)
    interp_delta = np.interp(grid, t, delta)
    window = max(5, int(round(0.20 / dt)) | 1)
    if window >= len(interp_delta):
        window = len(interp_delta) - 1 + len(interp_delta) % 2
    smooth = savgol_filter(interp_delta, window, 2)
    rate = np.abs(np.gradient(smooth, grid, edge_order=1))
    sustained_count = max(1, int(round(0.20 / dt)))
    sustained = np.convolve(
        rate, np.ones(sustained_count) / sustained_count, mode="valid")
    return {
        "available": True,
        "source": "Phase210 /odom during second recorded /plan (Gate)",
        "measurement_semantics": "effective_delta=atan(wheelbase*w_odom/v_odom), not steering sensor",
        "wheelbase_m": wheelbase,
        "moving_speed_threshold_mps": 0.05,
        "samples": len(rows),
        "median_dt_sec": dt,
        "smoothing_window_sec": window * dt,
        "max_abs_effective_delta_deg": float(
            np.max(np.abs(smooth)) * 180.0 / math.pi),
        "effective_delta_dot_p95_radps": float(np.percentile(rate, 95.0)),
        "effective_delta_dot_sustained_0p20s_mean_max_radps": float(
            np.max(sustained)),
        "effective_delta_dot_point_max_radps": float(np.max(rate)),
    }


def achieved_steering_rate(trace: list[dict[str, float]],
                           wheelbase: float = 0.144) -> dict[str, float | None]:
    moving = [
        row for row in trace
        if abs(row.get("v", 0.0)) > 0.02 and
        math.isfinite(row.get("kappa_actual", float("nan")))
    ]
    if len(moving) < 5:
        return {"achieved_delta_dot_p95_radps": None,
                "achieved_delta_dot_sustained_0p20s_mean_max_radps": None}
    t = np.asarray([row["t"] for row in moving], dtype=float)
    delta = np.asarray([
        math.atan(wheelbase * row["kappa_actual"]) for row in moving], dtype=float)
    rate = np.abs(np.gradient(delta, t, edge_order=1))
    # Exclude isolated numerical spikes from asynchronous command sampling in
    # the P95 and add a 0.20 s mean as the sustained plant capability metric.
    dt = max(float(np.median(np.diff(t))), 1.0e-3)
    count = max(1, int(round(0.20 / dt)))
    sustained = np.convolve(rate, np.ones(count) / count, mode="valid")
    return {
        "achieved_delta_dot_p95_radps": float(np.percentile(rate, 95.0)),
        "achieved_delta_dot_sustained_0p20s_mean_max_radps": float(np.max(sustained)),
    }


def first_sustained_rise(trace: list[dict[str, float]], key: str,
                         threshold: float = 0.5, count: int = 3) -> dict[str, float] | None:
    for i in range(0, max(0, len(trace) - count + 1)):
        window = trace[i:i + count]
        values = [row.get(key, float("nan")) for row in window]
        if all(math.isfinite(v) and abs(v) >= threshold for v in values):
            row = window[0]
            return {"t": row["t"], "s_m": row["s_m"], "kappa": row[key]}
    return None


def first_bend_interval(p: dict[str, Any]) -> dict[str, float | str | None]:
    s, kappa, _ = path_curvature(p)
    active = np.flatnonzero(np.abs(kappa) >= 0.25)
    if not len(active):
        return {"start_s_m": None, "end_s_m": min(float(s[-1]), 0.60),
                "sign": "none"}
    i0 = int(active[0])
    sign = 1.0 if kappa[i0] >= 0.0 else -1.0
    end = min(float(s[-1]), float(s[i0] + 0.60))
    # A compound connector can reverse curvature.  Treat that reversal as the
    # end of the first bend; otherwise use a bounded 0.60 m arc window.
    for i in range(i0 + 3, len(s)):
        if s[i] - s[i0] < 0.08:
            continue
        if sign * kappa[i] <= -0.15:
            end = float(s[i])
            break
    return {"start_s_m": float(s[i0]), "end_s_m": end,
            "sign": "left" if sign > 0.0 else "right"}


def segmented_metrics(trace: list[dict[str, float]], p: dict[str, Any]) -> dict[str, Any]:
    bend = first_bend_interval(p)
    end_s = float(bend["end_s_m"])
    first_rows = [row for row in trace if row["s_m"] <= end_s + 1.0e-9]
    if not first_rows:
        first_rows = trace[:1]
    rise = {
        key: first_sustained_rise(trace, key)
        for key in ("kappa_path", "kappa_raw", "kappa_nav", "kappa_safe", "kappa_actual")
    }
    high_threshold = min(
        2.4,
        0.8 * max(abs(row["kappa_path"]) for row in first_rows))
    high = {
        key: first_sustained_rise(trace, key, threshold=high_threshold)
        for key in ("kappa_path", "kappa_raw", "kappa_nav", "kappa_safe", "kappa_actual")
    }
    path_rise_s = rise["kappa_path"]["s_m"] if rise["kappa_path"] else None
    rise_offset = {}
    for key, item in rise.items():
        rise_offset[key] = (
            item["s_m"] - path_rise_s
            if item is not None and path_rise_s is not None else None)
    raw_onset_t = rise["kappa_raw"]["t"] if rise["kappa_raw"] else None
    onset_delay_from_raw = {
        key: (None if key == "kappa_path" else
            item["t"] - raw_onset_t
            if item is not None and raw_onset_t is not None else None)
        for key, item in rise.items()
    }
    raw_high_t = high["kappa_raw"]["t"] if high["kappa_raw"] else None
    high_delay_from_raw = {
        key: (None if key == "kappa_path" else
            item["t"] - raw_high_t
            if item is not None and raw_high_t is not None else None)
        for key, item in high.items()
    }
    high_path_s = high["kappa_path"]["s_m"] if high["kappa_path"] else None
    high_offset_from_path = {
        key: (
            item["s_m"] - high_path_s
            if item is not None and high_path_s is not None else None)
        for key, item in high.items()
    }
    moving_rows = [
        row for row in trace
        if abs(row.get("raw_v", 0.0)) > 0.01 or abs(row.get("v", 0.0)) > 0.01
    ]
    motion_start_t = moving_rows[0]["t"] if moving_rows else None
    path_to_raw_high_delay = (
        raw_high_t - motion_start_t
        if raw_high_t is not None and motion_start_t is not None and
        high_path_s is not None and high_path_s <= 0.01 else None)
    last = trace[-1]
    goal_x, goal_y = float(p["xy"][-1, 0]), float(p["xy"][-1, 1])
    xy_error = math.hypot(last["x"] - goal_x, last["y"] - goal_y)
    formal = xy_error <= 0.30 and 70.0 <= last["yaw_deg"] <= 110.0
    actual_motion = [row for row in trace if abs(row.get("v", 0.0)) > 0.02]
    command_motion = [
        row for row in trace
        if max(abs(row.get(key, 0.0)) for key in
               ("raw_v", "nav_v", "safe_v")) > 0.02]
    timing_origin = (
        actual_motion[0]["t"] if actual_motion else
        command_motion[0]["t"] if command_motion else trace[0]["t"])
    first_xy = next((
        row for row in trace
        if row["t"] >= timing_origin and
        math.hypot(row["x"] - goal_x, row["y"] - goal_y) <= 0.30), None)
    first_handoff = next((
        row for row in trace
        if row["t"] >= timing_origin and
        math.hypot(row["x"] - goal_x, row["y"] - goal_y) <= 0.30 and
        70.0 <= row["yaw_deg"] <= 110.0), None)
    end_time = last["t"]
    final_window = [row for row in trace if row["t"] >= end_time - 2.0]
    stalled = bool(
        not formal and final_window and
        max(abs(row.get("v", 0.0)) for row in final_window) < 0.02)
    action_result_row = next((
        row for row in trace
        if row.get("action_result_observed", 0.0) >= 0.5), None)
    terminal_settle = {
        "available": action_result_row is not None,
        "action_result_pose": None,
        "settled_pose": {
            "t": last["t"], "x": last["x"], "y": last["y"],
            "yaw_deg": last["yaw_deg"], "v": last.get("v", 0.0),
            "w": last.get("w", 0.0),
        },
        "settling_time_sec": None,
        "additional_distance_m": None,
        "delta_yaw_deg": None,
    }
    if action_result_row is not None:
        terminal_settle.update({
            "action_result_pose": {
                "t": action_result_row["t"],
                "x": action_result_row["x"],
                "y": action_result_row["y"],
                "yaw_deg": action_result_row["yaw_deg"],
                "v": action_result_row.get("v", 0.0),
                "w": action_result_row.get("w", 0.0),
            },
            "settling_time_sec": last["t"] - action_result_row["t"],
            "additional_distance_m": math.hypot(
                last["x"] - action_result_row["x"],
                last["y"] - action_result_row["y"]),
            "delta_yaw_deg": math.degrees(wrap(
                math.radians(last["yaw_deg"] -
                             action_result_row["yaw_deg"]))),
        })
    return {
        "first_bend": {
            **bend,
            "trajectory_samples": len(first_rows),
            "max_cte_m": max(row["cte_m"] for row in first_rows),
            "min_footprint_clearance_m": min(
                row["footprint_clearance_m"] for row in first_rows),
            "max_clearance_loss_m": max(
                row["clearance_loss_m"] for row in first_rows),
            "first_clearance_le_zero_s_m": next((
                row["s_m"] for row in first_rows
                if row["footprint_clearance_m"] <= 0.0), None),
            "collision_samples": int(sum(
                row["footprint_collision"] for row in first_rows)),
            "curvature_rise": rise,
            "curvature_rise_offset_from_path_m": rise_offset,
            "curvature_onset_delay_from_raw_sec": onset_delay_from_raw,
            "high_curvature_threshold": high_threshold,
            "high_curvature_rise": high,
            "high_curvature_offset_from_path_m": high_offset_from_path,
            "path_to_raw_high_curvature_delay_from_motion_sec": path_to_raw_high_delay,
            "high_curvature_delay_from_raw_sec": high_delay_from_raw,
        },
        "full_gate": {
            "duration_s": last["t"] - trace[0]["t"],
            "timing_origin": (
                "actual_motion_v_gt_0p02" if actual_motion else
                "command_motion_v_gt_0p02" if command_motion else
                "trace_start"),
            "time_to_first_xy_0p30_sec": (
                first_xy["t"] - timing_origin if first_xy else None),
            "time_to_first_formal_handoff_sec": (
                first_handoff["t"] - timing_origin if first_handoff else None),
            "actual_motion_duration_sec": (
                actual_motion[-1]["t"] - actual_motion[0]["t"]
                if len(actual_motion) >= 2 else 0.0),
            "command_motion_duration_sec": (
                command_motion[-1]["t"] - command_motion[0]["t"]
                if len(command_motion) >= 2 else 0.0),
            "final_xy_error_m": xy_error,
            "final_yaw_deg": last["yaw_deg"],
            "final_yaw_error_to_90_deg": wrap(
                math.radians(last["yaw_deg"] - 90.0)) * 180.0 / math.pi,
            "formal_handoff_xy_030_yaw_70_110": formal,
            "stalled_last_2s": stalled,
            "action_result_to_settled": terminal_settle,
        },
    }


def velocity_chain_metrics(trace: list[dict[str, float]],
                           p: dict[str, Any]) -> dict[str, Any]:
    """Score speed selection and curvature preservation by path regime.

    The competition limiter scales linear and angular velocity together when
    enforcing lateral acceleration, so a useful audit must distinguish a
    deliberate speed reduction from curvature distortion.  Regimes are based
    on the native Smac path, not on the realized trajectory, and the terminal
    window is reported separately because it overlaps the curvature buckets.
    """
    path_length = float(p["arc"][-1])

    def percentile(values: list[float], q: float) -> float | None:
        finite = [v for v in values if math.isfinite(v)]
        return float(np.percentile(finite, q)) if finite else None

    def summarize(rows: list[dict[str, float]]) -> dict[str, Any]:
        if not rows:
            return {"samples": 0}
        active = [
            r for r in rows
            if max(abs(r.get(key, 0.0)) for key in
                   ("raw_v", "nav_v", "safe_v", "v")) > 0.01]
        result: dict[str, Any] = {
            "samples": len(rows),
            "moving_command_samples": len(active),
        }
        for prefix, key in (
                ("raw", "raw_v"), ("nav", "nav_v"),
                ("safe", "safe_v"), ("actual", "v")):
            finite_speed = [
                abs(r.get(key, float("nan"))) for r in rows
                if math.isfinite(r.get(key, float("nan"))) and
                abs(r.get(key, 0.0)) > 0.01]
            result[f"{prefix}_moving_samples"] = len(finite_speed)
            result[f"{prefix}_speed_mean_mps"] = (
                float(np.mean(finite_speed)) if finite_speed else None)
            result[f"{prefix}_speed_median_mps"] = percentile(finite_speed, 50.0)
            result[f"{prefix}_speed_p05_mps"] = percentile(finite_speed, 5.0)
            result[f"{prefix}_speed_p95_mps"] = percentile(finite_speed, 95.0)
            result[f"{prefix}_speed_max_mps"] = percentile(finite_speed, 100.0)
        lateral = [
            abs(r.get("safe_v", float("nan")) *
                r.get("safe_w", float("nan"))) for r in active]
        result["safe_lateral_accel_p95_mps2"] = percentile(lateral, 95.0)
        result["safe_lateral_accel_max_mps2"] = percentile(lateral, 100.0)
        for label, left, right in (
                ("raw_to_nav", "kappa_raw", "kappa_nav"),
                ("nav_to_safe", "kappa_nav", "kappa_safe"),
                ("safe_to_actual", "kappa_safe", "kappa_actual")):
            error = [
                abs(r.get(right, float("nan")) -
                    r.get(left, float("nan")))
                for r in rows]
            result[f"{label}_abs_kappa_error_p95_1pm"] = percentile(error, 95.0)
            result[f"{label}_abs_kappa_error_max_1pm"] = percentile(error, 100.0)
        return result

    regimes = {
        "straight_abs_kappa_lt_0p5": [
            r for r in trace if abs(r["kappa_path"]) < 0.5],
        "moderate_abs_kappa_0p5_to_1p5": [
            r for r in trace if 0.5 <= abs(r["kappa_path"]) < 1.5],
        "sharp_abs_kappa_ge_1p5": [
            r for r in trace if abs(r["kappa_path"]) >= 1.5],
        "terminal_last_0p30m": [
            r for r in trace if path_length - r["s_m"] <= 0.30],
    }
    return {
        "regime_definition": {
            "straight_abs_kappa_1pm": "<0.5",
            "moderate_abs_kappa_1pm": "[0.5,1.5)",
            "sharp_abs_kappa_1pm": ">=1.5",
            "terminal_remaining_path_m": "<=0.30",
        },
        "regimes": {name: summarize(rows) for name, rows in regimes.items()},
    }


def limiter_timing_metrics(trace: list[dict[str, float]],
                           p: dict[str, Any],
                           sustain_count: int = 3) -> dict[str, Any]:
    """Locate where the reactive lateral limiter first changes the command.

    The limiter has no Smac-path preview: it can only scale a velocity command
    after upstream ``v,w`` imply excessive lateral acceleration.  Reporting
    all events in the same path-station coordinate makes that distinction
    explicit and prevents calling a reactive safety cap an anticipatory speed
    planner.
    """

    def first_window(predicate: Any) -> dict[str, float] | None:
        for i in range(0, max(0, len(trace) - sustain_count + 1)):
            window = trace[i:i + sustain_count]
            if all(predicate(row) for row in window):
                return window[0]
        return None

    moving = first_window(lambda r: (
        abs(r.get("raw_v", 0.0)) > 0.01 or
        abs(r.get("nav_v", 0.0)) > 0.01 or
        abs(r.get("v", 0.0)) > 0.01))
    raw_turn = first_window(lambda r: (
        math.isfinite(r.get("kappa_raw", float("nan"))) and
        abs(r["kappa_raw"]) >= 0.5))
    raw_high = first_window(lambda r: (
        math.isfinite(r.get("kappa_raw", float("nan"))) and
        abs(r["kappa_raw"]) >= 1.5))

    def limiter_changed(row: dict[str, float]) -> bool:
        nav = abs(row.get("nav_v", 0.0))
        safe = abs(row.get("safe_v", 0.0))
        if nav <= 0.02:
            return False
        return nav - safe >= max(0.005, 0.02 * nav)

    limited = first_window(limiter_changed)
    path_s, path_kappa, _ = path_curvature(p)
    path_turn_indices = np.flatnonzero(np.abs(path_kappa) >= 0.5)
    path_high_indices = np.flatnonzero(np.abs(path_kappa) >= 1.5)
    path_turn_s = (
        float(path_s[path_turn_indices[0]]) if len(path_turn_indices) else None)
    path_high_s = (
        float(path_s[path_high_indices[0]]) if len(path_high_indices) else None)

    actual_decel = None
    if limited is not None and len(trace) >= sustain_count + 1:
        limited_index = trace.index(limited)
        for i in range(max(1, limited_index), len(trace) - sustain_count + 1):
            window = trace[i:i + sustain_count]
            rates = []
            for left, right in zip(trace[i - 1:i + sustain_count - 1], window):
                dt = right["t"] - left["t"]
                if dt <= 1.0e-6:
                    rates.append(0.0)
                else:
                    rates.append((abs(right.get("v", 0.0)) -
                                  abs(left.get("v", 0.0))) / dt)
            if (all(rate <= -0.05 for rate in rates) and
                    all(abs(row.get("v", 0.0)) >= 0.05 for row in window)):
                actual_decel = window[0]
                break

    def event(row: dict[str, float] | None) -> dict[str, float] | None:
        if row is None:
            return None
        nav_v = row.get("nav_v", 0.0)
        safe_v = row.get("safe_v", 0.0)
        return {
            "t": row["t"],
            "s_m": row["s_m"],
            "kappa_path_1pm": row.get("kappa_path", float("nan")),
            "kappa_raw_1pm": row.get("kappa_raw", float("nan")),
            "nav_v_mps": nav_v,
            "nav_w_radps": row.get("nav_w", 0.0),
            "safe_v_mps": safe_v,
            "safe_w_radps": row.get("safe_w", 0.0),
            "actual_v_mps": row.get("v", 0.0),
            "actual_w_radps": row.get("w", 0.0),
            "linear_scale_safe_over_nav": (
                abs(safe_v / nav_v) if abs(nav_v) > 1.0e-6 else float("nan")),
        }

    def delay(later: dict[str, float] | None,
              earlier: dict[str, float] | None,
              key: str) -> float | None:
        if later is None or earlier is None:
            return None
        return float(later[key] - earlier[key])

    return {
        "semantics": (
            "competition limiter is reactive to upstream v,w; it has no "
            "Smac curvature preview"),
        "sustain_samples": sustain_count,
        "path_curvature_onset_s_m": path_turn_s,
        "path_high_curvature_onset_s_m": path_high_s,
        "motion_start": event(moving),
        "raw_curvature_onset": event(raw_turn),
        "raw_high_curvature_onset": event(raw_high),
        "limiter_first_active": event(limited),
        "actual_first_sustained_deceleration": event(actual_decel),
        "limiter_delay_from_motion_start_sec": delay(limited, moving, "t"),
        "limiter_delay_from_raw_curvature_onset_sec": delay(limited, raw_turn, "t"),
        "limiter_delay_from_raw_high_curvature_sec": delay(limited, raw_high, "t"),
        "limiter_station_offset_from_path_curvature_onset_m": (
            limited["s_m"] - path_turn_s
            if limited is not None and path_turn_s is not None else None),
        "limiter_station_offset_from_path_high_curvature_m": (
            limited["s_m"] - path_high_s
            if limited is not None and path_high_s is not None else None),
        "actual_decel_delay_from_limiter_sec": delay(actual_decel, limited, "t"),
        "actual_decel_station_offset_from_limiter_m": delay(
            actual_decel, limited, "s_m"),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", type=Path, required=True)
    ap.add_argument("--path", type=Path, required=True)
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out-json", type=Path, required=True)
    ap.add_argument("--out-csv", type=Path)
    ap.add_argument("--dense-step", type=float, default=0.01)
    ap.add_argument("--cone-hard-radius", type=float, default=0.16)
    args = ap.parse_args()
    _, plans, _, _, map_data, odom, _ = deserialize_bag(args.bag)
    p = load_path(args.path)
    clearance_grid = make_clearance_grid(map_data)
    sweep = sweep_path(p, map_data, clearance_grid, args.dense_step)
    scenario_start = p["obj"].get("start")
    if not isinstance(scenario_start, list) or len(scenario_start) != 3:
        raise ValueError("native path artifact is missing exact scenario start")
    start_x, start_y, start_yaw_deg = map(float, scenario_start)
    start_yaw = math.radians(start_yaw_deg)
    start_samples = np.asarray(
        footprint_samples(start_x, start_y, start_yaw), dtype=float)
    exact_start_valid = not footprint_collision(
        map_data, start_x, start_y, start_yaw)
    exact_start_clearance = min_clearance(
        start_samples, map_data, clearance_grid)
    rows = read_actual(args.csv)
    actual, detailed = evaluate_actual(
        rows, p, map_data, clearance_grid,
        sweep["s"], sweep["clearance"])
    cone_clearance = evaluate_fixed_cones(
        p, detailed, args.cone_hard_radius)
    segments = segmented_metrics(detailed, p)
    ps, pk, pdk = path_curvature(p)
    finite_k = np.abs(pk[np.isfinite(pk)])
    report = {
        "csv": str(args.csv), "native_smacc_path": str(args.path),
        "path_frame": p["obj"].get("frame"), "start": p["obj"].get("start"), "goal": p["obj"].get("goal"),
        "path": {"point_count": int(len(p["xy"])), "length_m": float(p["arc"][-1]),
                 "mean_spacing_m": float(np.mean(np.diff(p["arc"]))),
                 "median_spacing_m": float(np.median(np.diff(p["arc"]))),
                 "max_abs_kappa": float(np.max(finite_k)) if len(finite_k) else 0.0,
                 "max_abs_dkappa_ds": float(np.max(np.abs(pdk))) if len(pdk) else 0.0,
                 "steering_difficulty": steering_path_difficulty(p, 0.50)},
        "start_full_footprint": {
            "valid": exact_start_valid,
            "x": start_x,
            "y": start_y,
            "yaw_deg": start_yaw_deg,
            "min_clearance_m": exact_start_clearance,
            "smac_first_pose": {
                "x": float(p["xy"][0, 0]),
                "y": float(p["xy"][0, 1]),
                "yaw_deg": math.degrees(float(p["yaw"][0])),
            },
        },
        "smac_path_static": {k: v for k, v in sweep.items() if k not in ("s", "xy", "yaw", "clearance")},
        "mppi_actual": actual,
        "fixed_cone_clearance": cone_clearance,
        "plant_steering_response": achieved_steering_rate(detailed),
        "velocity_chain": velocity_chain_metrics(detailed, p),
        "limiter_timing": limiter_timing_metrics(detailed, p),
        "historical_real_vehicle_effective_steering_response":
            historical_effective_steering_rate(plans, odom),
        "segmented_metrics": segments,
        "diagnostic_trace": json_safe(detailed),
    }
    args.out_json.parent.mkdir(parents=True, exist_ok=True)
    args.out_json.write_text(json.dumps(json_safe(report), indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if args.out_csv:
        args.out_csv.parent.mkdir(parents=True, exist_ok=True)
        fields = sorted({k for r in detailed for k in r})
        with args.out_csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(detailed)
    print(json.dumps({k: report[k] for k in ("native_smacc_path", "start_full_footprint", "smac_path_static", "mppi_actual", "path")}, indent=2, allow_nan=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
