#!/usr/bin/env python3
"""Shared geometry and validation helpers for channel assets.

The functions in this module are deliberately ROS-independent.  Generators,
visualizers, evaluators and promotion checks all use the same map indexing,
footprint sampling, path interpolation and SHA-256 rules.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Iterator, Sequence

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt


FOOTPRINT_FRONT_M = 0.28
FOOTPRINT_REAR_M = 0.11
FOOTPRINT_HALF_WIDTH_M = 0.13
FOOTPRINT_PADDING_M = 0.0
HARD_MINIMUM_RADIUS_M = 0.35
DEFAULT_RESOLUTION_M = 0.01
LETHAL_PIXEL_MAX = 89

TUBE_HEADER = (
    "s_m", "x_m", "y_m", "yaw_rad", "kappa_1pm", "radius_m",
    "tube", "direction",
)
CONNECTOR_HEADER = (
    "s_m", "x_m", "y_m", "yaw_rad", "kappa_1pm", "radius_m",
    "direction", "source_lane", "target_lane", "anchor_id",
)


def normalize(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_json(value: object) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


@dataclass(frozen=True)
class MapGrid:
    path: Path
    pixels: np.ndarray
    hard: np.ndarray
    clearance_m: np.ndarray
    resolution_m: float = DEFAULT_RESOLUTION_M
    origin_x_m: float = 0.0
    origin_y_m: float = 0.0

    @property
    def height(self) -> int:
        return int(self.pixels.shape[0])

    @property
    def width(self) -> int:
        return int(self.pixels.shape[1])

    def world_to_image(self, x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        col = np.floor((x - self.origin_x_m) / self.resolution_m).astype(int)
        map_row = np.floor((y - self.origin_y_m) / self.resolution_m).astype(int)
        row = self.height - 1 - map_row
        return row, col


def load_map(path: Path, *, resolution_m: float = DEFAULT_RESOLUTION_M,
             origin_x_m: float = 0.0, origin_y_m: float = 0.0) -> MapGrid:
    pixels = np.asarray(Image.open(path))
    if pixels.ndim != 2:
        raise ValueError(f"expected grayscale map, got shape={pixels.shape}")
    hard = pixels <= LETHAL_PIXEL_MAX
    clearance = distance_transform_edt(~hard) * resolution_m
    return MapGrid(
        path=path, pixels=pixels, hard=hard, clearance_m=clearance,
        resolution_m=resolution_m, origin_x_m=origin_x_m,
        origin_y_m=origin_y_m,
    )


def footprint_points(step_m: float = 0.01, *, interior: bool = True) -> np.ndarray:
    front = FOOTPRINT_FRONT_M + FOOTPRINT_PADDING_M
    rear = FOOTPRINT_REAR_M + FOOTPRINT_PADDING_M
    half = FOOTPRINT_HALF_WIDTH_M + FOOTPRINT_PADDING_M
    if interior:
        xs = np.arange(-rear, front + step_m * 0.5, step_m)
        ys = np.arange(-half, half + step_m * 0.5, step_m)
        return np.asarray([(x, y) for x in xs for y in ys], dtype=float)
    corners = [(front, half), (front, -half), (-rear, -half), (-rear, half)]
    points: list[tuple[float, float]] = []
    for start, finish in zip(corners, corners[1:] + corners[:1]):
        length = math.hypot(finish[0] - start[0], finish[1] - start[1])
        count = max(1, int(math.ceil(length / step_m)))
        for index in range(count):
            ratio = index / count
            points.append((
                start[0] + ratio * (finish[0] - start[0]),
                start[1] + ratio * (finish[1] - start[1]),
            ))
    return np.asarray(points, dtype=float)


FOOTPRINT_INTERIOR = footprint_points(0.01, interior=True)
FOOTPRINT_BOUNDARY = footprint_points(0.005, interior=False)


def world_footprint(x_m: float, y_m: float, yaw_rad: float,
                    samples: np.ndarray = FOOTPRINT_INTERIOR) -> tuple[np.ndarray, np.ndarray]:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        x_m + cosine * samples[:, 0] - sine * samples[:, 1],
        y_m + sine * samples[:, 0] + cosine * samples[:, 1],
    )


def pose_clearance(grid: MapGrid, x_m: float, y_m: float,
                   yaw_rad: float) -> tuple[bool, float]:
    x, y = world_footprint(x_m, y_m, yaw_rad)
    row, col = grid.world_to_image(x, y)
    valid = (row >= 0) & (row < grid.height) & (col >= 0) & (col < grid.width)
    if not bool(np.all(valid)):
        return False, 0.0
    collision = bool(np.any(grid.hard[row, col]))
    clearance = float(np.min(grid.clearance_m[row, col]))
    return not collision, clearance


def read_path_csv(path: Path) -> list[dict[str, float | str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"x_m", "y_m", "yaw_rad"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"{path}: missing required columns {sorted(required)}")
    output: list[dict[str, float | str]] = []
    for row in rows:
        output.append({
            "s": float(row.get("s_m") or 0.0),
            "x": float(row["x_m"]),
            "y": float(row["y_m"]),
            "yaw": float(row["yaw_rad"]),
            "kappa": float(row.get("kappa_1pm") or 0.0),
            "lane": row.get("tube", row.get("target_lane", "")),
            "direction": row.get("direction", ""),
        })
    return output


def cyclic_geometry(x: Sequence[float], y: Sequence[float]) -> list[dict[str, float]]:
    if len(x) != len(y) or len(x) < 4:
        raise ValueError("closed path needs at least four x/y samples")
    count = len(x)
    yaw = [0.0] * count
    for index in range(count):
        previous = (index - 1) % count
        following = (index + 1) % count
        yaw[index] = math.atan2(y[following] - y[previous], x[following] - x[previous])
    kappa = [0.0] * count
    for index in range(count):
        previous = (index - 1) % count
        following = (index + 1) % count
        ab_x = x[index] - x[previous]
        ab_y = y[index] - y[previous]
        bc_x = x[following] - x[index]
        bc_y = y[following] - y[index]
        ac_x = x[following] - x[previous]
        ac_y = y[following] - y[previous]
        denominator = (
            math.hypot(ab_x, ab_y) * math.hypot(bc_x, bc_y) *
            math.hypot(ac_x, ac_y)
        )
        # Signed circumcircle curvature is substantially less sensitive to
        # the analytic center path's rounded CSV coordinates than a second
        # finite difference of two chord headings.  The latter falsely
        # reported R=0.35 m even for sub-millimeter offsets at arc joins.
        kappa[index] = (
            2.0 * (ab_x * bc_y - ab_y * bc_x) / denominator
            if denominator > 1.0e-12 else 0.0
        )
    cumulative = [0.0] * count
    for index in range(1, count):
        cumulative[index] = cumulative[index - 1] + math.hypot(
            x[index] - x[index - 1], y[index] - y[index - 1],
        )
    return [{
        "s": cumulative[index], "x": float(x[index]), "y": float(y[index]),
        "yaw": yaw[index], "kappa": kappa[index],
    } for index in range(count)]


def open_geometry(x: Sequence[float], y: Sequence[float]) -> list[dict[str, float]]:
    if len(x) != len(y) or len(x) < 2:
        raise ValueError("open path needs at least two x/y samples")
    count = len(x)
    yaw: list[float] = []
    for index in range(count):
        previous = max(0, index - 1)
        following = min(count - 1, index + 1)
        yaw.append(math.atan2(y[following] - y[previous], x[following] - x[previous]))
    kappa = [0.0] * count
    for index in range(1, count - 1):
        chord = math.hypot(x[index + 1] - x[index - 1], y[index + 1] - y[index - 1])
        kappa[index] = normalize(yaw[index + 1] - yaw[index - 1]) / max(chord, 1.0e-9)
    if count > 2:
        kappa[0] = kappa[1]
        kappa[-1] = kappa[-2]
    cumulative = [0.0] * count
    for index in range(1, count):
        cumulative[index] = cumulative[index - 1] + math.hypot(
            x[index] - x[index - 1], y[index] - y[index - 1],
        )
    return [{
        "s": cumulative[index], "x": float(x[index]), "y": float(y[index]),
        "yaw": yaw[index], "kappa": kappa[index],
    } for index in range(count)]


def write_tube(path: Path, rows: Sequence[dict[str, float]], lane: str,
               direction: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(TUBE_HEADER)
        for row in rows:
            kappa = float(row["kappa"])
            radius = "" if abs(kappa) < 1.0e-12 else f"{1.0 / abs(kappa):.6f}"
            writer.writerow((
                f"{float(row['s']):.6f}", f"{float(row['x']):.6f}",
                f"{float(row['y']):.6f}", f"{float(row['yaw']):.9f}",
                f"{kappa:.9f}", radius, lane, direction,
            ))


def write_connector(path: Path, rows: Sequence[dict[str, float]], *,
                    direction: str, source_lane: str, target_lane: str,
                    anchor_id: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(CONNECTOR_HEADER)
        for row in rows:
            kappa = float(row["kappa"])
            radius = "" if abs(kappa) < 1.0e-12 else f"{1.0 / abs(kappa):.6f}"
            writer.writerow((
                f"{float(row['s']):.6f}", f"{float(row['x']):.6f}",
                f"{float(row['y']):.6f}", f"{float(row['yaw']):.9f}",
                f"{kappa:.9f}", radius, direction, source_lane, target_lane,
                anchor_id,
            ))


def dense_poses(rows: Sequence[dict[str, float | str]], *, closed: bool,
                step_m: float = 0.005) -> Iterator[tuple[float, float, float]]:
    pairs = list(zip(rows, rows[1:]))
    if closed:
        pairs.append((rows[-1], rows[0]))
    for start, finish in pairs:
        length = math.hypot(
            float(finish["x"]) - float(start["x"]),
            float(finish["y"]) - float(start["y"]),
        )
        count = max(1, int(math.ceil(length / step_m)))
        yaw_delta = normalize(float(finish["yaw"]) - float(start["yaw"]))
        for index in range(count):
            ratio = index / count
            yield (
                float(start["x"]) + ratio * (float(finish["x"]) - float(start["x"])),
                float(start["y"]) + ratio * (float(finish["y"]) - float(start["y"])),
                normalize(float(start["yaw"]) + ratio * yaw_delta),
            )
    if rows:
        yield float(rows[-1]["x"]), float(rows[-1]["y"]), float(rows[-1]["yaw"])


def audit_path(grid: MapGrid, rows: Sequence[dict[str, float | str]], *,
               closed: bool, dense_step_m: float = 0.005) -> dict[str, float | int | bool]:
    collisions = 0
    minimum_clearance = math.inf
    samples = 0
    for x_m, y_m, yaw_rad in dense_poses(rows, closed=closed, step_m=dense_step_m):
        samples += 1
        free, clearance = pose_clearance(grid, x_m, y_m, yaw_rad)
        collisions += int(not free)
        minimum_clearance = min(minimum_clearance, clearance)
    maximum_curvature = max(abs(float(row["kappa"])) for row in rows)
    minimum_radius = 1.0 / maximum_curvature if maximum_curvature > 1.0e-12 else math.inf
    closure_xy = 0.0
    closure_yaw = 0.0
    if closed:
        closure_xy = math.hypot(
            float(rows[-1]["x"]) - float(rows[0]["x"]),
            float(rows[-1]["y"]) - float(rows[0]["y"]),
        )
        closure_yaw = abs(normalize(float(rows[-1]["yaw"]) - float(rows[0]["yaw"])))
    return {
        "dense_pose_samples": samples,
        "hard_violation_count": collisions,
        "minimum_static_footprint_clearance_m": round(minimum_clearance, 6),
        "maximum_abs_curvature_1pm": round(maximum_curvature, 9),
        "minimum_radius_m": round(minimum_radius, 6),
        "closure_xy_m": round(closure_xy, 6),
        "closure_yaw_rad": round(closure_yaw, 9),
        "admitted": collisions == 0 and minimum_radius + 1.0e-6 >= HARD_MINIMUM_RADIUS_M,
    }


def nearest_index(rows: Sequence[dict[str, float | str]], x_m: float,
                  y_m: float) -> int:
    return min(range(len(rows)), key=lambda index: (
        float(rows[index]["x"]) - x_m) ** 2 +
        (float(rows[index]["y"]) - y_m) ** 2)


def forward_distance(rows: Sequence[dict[str, float | str]], start_index: int,
                     finish_index: int) -> float:
    total = 0.0
    index = start_index % len(rows)
    finish = finish_index % len(rows)
    for _ in range(len(rows) + 1):
        if index == finish:
            return total
        following = (index + 1) % len(rows)
        total += math.hypot(
            float(rows[following]["x"]) - float(rows[index]["x"]),
            float(rows[following]["y"]) - float(rows[index]["y"]),
        )
        index = following
    raise RuntimeError("failed to traverse closed path")


def path_point_distance(rows: Sequence[dict[str, float | str]], x_m: float,
                        y_m: float) -> float:
    return math.sqrt(min(
        (float(row["x"]) - x_m) ** 2 + (float(row["y"]) - y_m) ** 2
        for row in rows
    ))


def manifest_file_rows(root: Path, paths: Iterable[Path]) -> list[dict[str, object]]:
    return [{
        "path": str(path.relative_to(root)),
        "bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    } for path in sorted(paths)]
