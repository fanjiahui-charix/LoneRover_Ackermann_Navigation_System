#!/usr/bin/env python3
"""Build direction-bound 1 cm cone-side classifier lookup tables.

Runtime codes describe where the cone is, not whether a candidate lane is
guaranteed feasible: a cone on the geometric Inner side requests Outer and a
cone on the Outer side requests Inner.  Swept-footprint feasibility remains a
separate offline diagnostic and never turns normal channel cells into HOLD.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt
from scipy.spatial import cKDTree

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import (  # noqa: E402
    FOOTPRINT_FRONT_M, FOOTPRINT_HALF_WIDTH_M, FOOTPRINT_INTERIOR,
    FOOTPRINT_REAR_M, load_map, read_path_csv, sha256_file, world_footprint,
)


ENCODING = {"OUTSIDE_OR_INVALID": 0, "CONE_INNER_SIDE": 1, "CONE_OUTER_SIDE": 2}


def swept_mask(grid, rows) -> np.ndarray:
    """Rasterize the exact rectangular footprint at every Tube sample."""
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    for row in rows:
        x, y = world_footprint(float(row["x"]), float(row["y"]), float(row["yaw"]),
                               FOOTPRINT_INTERIOR)
        image_row, col = grid.world_to_image(x, y)
        valid = ((image_row >= 0) & (image_row < grid.height) &
                 (col >= 0) & (col < grid.width))
        mask[image_row[valid], col[valid]] = True
    return mask


def centerline_mask(grid, rows) -> np.ndarray:
    mask = np.zeros((grid.height, grid.width), dtype=bool)
    x = np.asarray([float(row["x"]) for row in rows])
    y = np.asarray([float(row["y"]) for row in rows])
    image_row, col = grid.world_to_image(x, y)
    valid = ((image_row >= 0) & (image_row < grid.height) &
             (col >= 0) & (col < grid.width))
    mask[image_row[valid], col[valid]] = True
    return mask


def classify_geometric_side(grid, center_rows, legal: np.ndarray,
                            direction: str) -> np.ndarray:
    codes = np.zeros((grid.height, grid.width), dtype=np.uint8)
    image_rows, cols = np.nonzero(legal)
    if len(image_rows) == 0:
        return codes
    world_x = grid.origin_x_m + (cols + 0.5) * grid.resolution_m
    map_rows = grid.height - 1 - image_rows
    world_y = grid.origin_y_m + (map_rows + 0.5) * grid.resolution_m
    center_xy = np.asarray([
        (float(row["x"]), float(row["y"])) for row in center_rows
    ])
    _, nearest = cKDTree(center_xy).query(np.column_stack((world_x, world_y)), k=1)
    yaw = np.asarray([float(center_rows[int(index)]["yaw"]) for index in nearest])
    dx = world_x - center_xy[nearest, 0]
    dy = world_y - center_xy[nearest, 1]
    signed_left = -np.sin(yaw) * dx + np.cos(yaw) * dy
    inner_left_sign = 1.0 if direction == "ccw" else -1.0
    cone_inner = inner_left_sign * signed_left >= 0.0
    codes[image_rows[cone_inner], cols[cone_inner]] = ENCODING["CONE_INNER_SIDE"]
    codes[image_rows[~cone_inner], cols[~cone_inner]] = ENCODING["CONE_OUTER_SIDE"]
    return codes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--connector-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--cone-effective-radius", type=float, default=0.13)
    parser.add_argument("--channel-half-width", type=float, default=0.27,
                        help="legal cone-center band around the Center reference")
    parser.add_argument("--out-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.out_dir.exists():
        raise SystemExit(f"refusing to overwrite avoid-map candidate: {args.out_dir}")
    if not 0.01 <= args.cone_effective_radius <= 0.30:
        raise SystemExit("cone effective radius must be within 0.01..0.30 m")
    connector_files = sorted(args.connector_dir.glob("connectors/connector_*.csv"))
    if not connector_files:
        raise SystemExit("connector candidate/library is empty")
    grid = load_map(args.map)
    args.out_dir.mkdir(parents=True)
    direction_reports = {}
    output_files = []
    for direction in ("cw", "ccw"):
        rows = {
            lane: read_path_csv(args.tube_dir / f"tube_{lane}_{direction}.csv")
            for lane in ("center", "inner", "outer")
        }
        masks = {lane: swept_mask(grid, value) for lane, value in rows.items()}
        clearance = {
            lane: distance_transform_edt(~mask) * grid.resolution_m
            for lane, mask in masks.items()
        }
        centerline_distance = distance_transform_edt(
            ~centerline_mask(grid, rows["center"])) * grid.resolution_m
        legal = (~grid.hard) & (centerline_distance <= args.channel_half_width)
        relevant = legal & (clearance["center"] <= args.cone_effective_radius)
        inner_ok = clearance["inner"] + 1e-9 >= args.cone_effective_radius
        outer_ok = clearance["outer"] + 1e-9 >= args.cone_effective_radius
        codes_image = classify_geometric_side(grid, rows["center"], legal, direction)
        cone_inner = codes_image == ENCODING["CONE_INNER_SIDE"]
        cone_outer = codes_image == ENCODING["CONE_OUTER_SIDE"]
        # Binary row 0 is map y=0, matching supervisor's O(1) iy*width+ix lookup.
        binary = np.flipud(codes_image).tobytes(order="C")
        output = args.out_dir / f"channel_cone_avoid_lane_1cm_{direction}.bin"
        output.write_bytes(binary)
        output_files.append(output)
        palette = np.zeros((grid.height, grid.width, 3), dtype=np.uint8)
        palette[:] = (35, 35, 35)
        palette[legal] = (225, 225, 225)
        palette[codes_image == 1] = (40, 110, 235)
        palette[codes_image == 2] = (245, 145, 35)
        visualization = args.out_dir / f"visualization_{direction}.png"
        Image.fromarray(palette).save(visualization)
        output_files.append(visualization)
        feasibility_palette = np.zeros((grid.height, grid.width, 3), dtype=np.uint8)
        feasibility_palette[:] = (35, 35, 35)
        feasibility_palette[legal & ~inner_ok & ~outer_ok] = (215, 45, 45)
        feasibility_palette[legal & inner_ok & ~outer_ok] = (40, 110, 235)
        feasibility_palette[legal & ~inner_ok & outer_ok] = (245, 145, 35)
        feasibility_palette[legal & inner_ok & outer_ok] = (55, 180, 95)
        feasibility_visualization = args.out_dir / f"feasibility_diagnostic_{direction}.png"
        Image.fromarray(feasibility_palette).save(feasibility_visualization)
        output_files.append(feasibility_visualization)
        direction_reports[direction] = {
            "file": output.name,
            "sha256": sha256_file(output),
            "legal_cells": int(np.count_nonzero(legal)),
            "center_threat_cells": int(np.count_nonzero(relevant)),
            "cone_inner_side_cells": int(np.count_nonzero(cone_inner)),
            "cone_outer_side_cells": int(np.count_nonzero(cone_outer)),
            "classified_legal_fraction": round(
                float(np.count_nonzero(codes_image)) /
                max(float(np.count_nonzero(legal)), 1.0), 6,
            ),
            "feasibility_diagnostic": {
                "neither_side_feasible_cells": int(np.count_nonzero(
                    legal & ~inner_ok & ~outer_ok)),
                "inner_only_feasible_cells": int(np.count_nonzero(
                    legal & inner_ok & ~outer_ok)),
                "outer_only_feasible_cells": int(np.count_nonzero(
                    legal & ~inner_ok & outer_ok)),
                "both_sides_feasible_cells": int(np.count_nonzero(
                    legal & inner_ok & outer_ok)),
                "center_threat_neither_feasible_cells": int(np.count_nonzero(
                    relevant & ~inner_ok & ~outer_ok)),
                "runtime_classifier_dependency": False,
            },
        }
    manifest = {
        "schema_version": 2,
        "asset_kind": "channel_cone_avoid_lane_candidate",
        "candidate_only": True,
        "runtime_promotion_allowed": False,
        "encoding": ENCODING,
        "grid": {"width": grid.width, "height": grid.height,
                 "resolution_m": grid.resolution_m,
                 "origin_x_m": grid.origin_x_m, "origin_y_m": grid.origin_y_m,
                 "binary_row_order": "map_y_ascending"},
        "footprint_m": {"front": FOOTPRINT_FRONT_M, "rear": FOOTPRINT_REAR_M,
                        "half_width": FOOTPRINT_HALF_WIDTH_M, "padding": 0.0},
        "cone_effective_radius_m": args.cone_effective_radius,
        "cone_effective_radius_semantics": (
            "ConeLayer lethal disk around cone center; vehicle footprint is applied "
            "by Nav2 collision checking and is not added again"
        ),
        "channel_half_width_m": args.channel_half_width,
        "static_map": {"path": str(args.map), "sha256": sha256_file(args.map)},
        "tube_hashes": {p.name: sha256_file(p) for p in sorted(args.tube_dir.glob("tube_*.csv"))},
        "connector_hashes": {p.name: sha256_file(p) for p in connector_files},
        "generator": {"path": str(Path(__file__).resolve()),
                      "sha256": sha256_file(Path(__file__).resolve())},
        "directions": direction_reports,
        "remaining_gate": "native RPP obstacle shadow and field validation",
    }
    manifest_path = args.out_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n")
    print(json.dumps(direction_reports, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
