#!/usr/bin/env python3
"""Render real-map Tube overlays, corner crops, curvature and clearance plots."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import (  # noqa: E402
    FOOTPRINT_BOUNDARY,
    load_map,
    pose_clearance,
    read_path_csv,
    world_footprint,
)


COLORS = {"inner": "#00b894", "center": "#f9ca24", "outer": "#0984e3"}
STYLES = {"cw": "-", "ccw": "--"}
CONSTRAINT_COLORS = {
    "MAP": "#6c5ce7", "RMIN": "#d63031", "SMOOTHNESS": "#636e72",
    "DYNAMIC_MARGIN": "#e17055",
}


def draw_map(ax, grid):
    extent = (
        grid.origin_x_m, grid.origin_x_m + grid.width * grid.resolution_m,
        grid.origin_y_m, grid.origin_y_m + grid.height * grid.resolution_m,
    )
    ax.imshow(grid.pixels, cmap="gray", origin="upper", extent=extent, vmin=0, vmax=255)
    ax.set_aspect("equal")
    ax.set_xlabel("map x [m]")
    ax.set_ylabel("map y [m]")


def plot_paths(ax, paths, *, footprint=False):
    for name, rows in sorted(paths.items()):
        _, lane, direction = name.split("_")
        ax.plot([float(row["x"]) for row in rows],
                [float(row["y"]) for row in rows],
                STYLES[direction], color=COLORS[lane], linewidth=1.6,
                label=f"{lane}-{direction}")
        if footprint:
            for row in rows[::40]:
                x, y = world_footprint(
                    float(row["x"]), float(row["y"]), float(row["yaw"]),
                    FOOTPRINT_BOUNDARY,
                )
                ax.add_patch(Polygon(
                    np.column_stack((x, y)), closed=True,
                    facecolor=COLORS[lane], edgecolor="none", alpha=0.04,
                ))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument(
        "--profile-dir", type=Path,
        help="offset_profiles directory; defaults beside the Tube directory",
    )
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    grid = load_map(args.map)
    paths = {path.stem: read_path_csv(path) for path in sorted(args.tube_dir.glob("tube_*.csv"))}
    if len(paths) != 6:
        raise SystemExit(f"expected six Tube CSVs, found {len(paths)}")

    figure, axis = plt.subplots(figsize=(9, 8), constrained_layout=True)
    draw_map(axis, grid)
    plot_paths(axis, paths, footprint=True)
    axis.legend(ncol=2, fontsize=8)
    axis.set_title("Tube candidates and sampled swept footprints")
    figure.savefig(args.out_dir / "overview.png", dpi=180)
    plt.close(figure)

    center = paths["tube_center_cw"]
    xs = np.asarray([float(row["x"]) for row in center])
    ys = np.asarray([float(row["y"]) for row in center])
    corner_centers = {
        "bottom_left": (float(xs.min() + 0.20), float(ys.min() + 0.20)),
        "top_left": (float(xs.min() + 0.20), float(ys.max() - 0.20)),
        "top_right": (float(xs.max() - 0.20), float(ys.max() - 0.20)),
        "bottom_right": (float(xs.max() - 0.20), float(ys.min() + 0.20)),
    }
    for name, (cx, cy) in corner_centers.items():
        figure, axis = plt.subplots(figsize=(6, 6), constrained_layout=True)
        draw_map(axis, grid)
        plot_paths(axis, paths, footprint=True)
        axis.set_xlim(cx - 0.65, cx + 0.65)
        axis.set_ylim(cy - 0.65, cy + 0.65)
        axis.set_title(name.replace("_", " ").title())
        figure.savefig(args.out_dir / f"corner_{name}.png", dpi=200)
        plt.close(figure)

    curvature_rows = []
    clearance_rows = []
    figure, (curvature_axis, clearance_axis) = plt.subplots(
        2, 1, figsize=(10, 8), constrained_layout=True,
    )
    for name, rows in sorted(paths.items()):
        _, lane, direction = name.split("_")
        s = [float(row["s"]) for row in rows]
        curvature = [float(row["kappa"]) for row in rows]
        clearance = []
        for index, row in enumerate(rows):
            _, value = pose_clearance(
                grid, float(row["x"]), float(row["y"]), float(row["yaw"]),
            )
            clearance.append(value)
            curvature_rows.append((name, index, s[index], curvature[index]))
            clearance_rows.append((name, index, s[index], value))
        curvature_axis.plot(s, curvature, STYLES[direction], color=COLORS[lane], label=name)
        clearance_axis.plot(s, clearance, STYLES[direction], color=COLORS[lane], label=name)
    curvature_axis.axhline(1.0 / 0.35, color="red", linewidth=0.8, linestyle=":")
    curvature_axis.axhline(-1.0 / 0.35, color="red", linewidth=0.8, linestyle=":")
    curvature_axis.axhline(1.0 / 0.40, color="orange", linewidth=0.8, linestyle="--")
    curvature_axis.axhline(-1.0 / 0.40, color="orange", linewidth=0.8, linestyle="--")
    curvature_axis.set_ylabel("curvature [1/m]")
    curvature_axis.set_title("Tube curvature")
    clearance_axis.set_xlabel("path s [m]")
    clearance_axis.set_ylabel("footprint clearance [m]")
    clearance_axis.set_title("Static-map footprint clearance")
    clearance_axis.legend(ncol=3, fontsize=7)
    figure.savefig(args.out_dir / "curvature_clearance.png", dpi=180)
    plt.close(figure)

    profile_dir = args.profile_dir or args.tube_dir.parent / "offset_profiles"
    profile_paths = sorted(profile_dir.glob("tube_*_offset.csv"))
    if profile_paths:
        figure, axes = plt.subplots(2, 2, figsize=(14, 9), constrained_layout=True)
        for axis, profile_path in zip(axes.flat, profile_paths):
            with profile_path.open(newline="", encoding="utf-8") as stream:
                rows = list(csv.DictReader(stream))
            s_m = np.asarray([float(row["s_m"]) for row in rows])
            final = np.asarray([float(row["final_offset_m"]) for row in rows])
            local_cap = np.asarray([float(row["hard_local_cap_m"]) for row in rows])
            map_cap = np.asarray([float(row["dynamic_margin_cap_m"]) for row in rows])
            hard_curvature = np.asarray([
                float(row["hard_curvature_cap_m"]) for row in rows
            ])
            axis.plot(s_m, map_cap * 100.0, color="#6c5ce7", alpha=0.45,
                      linewidth=0.9, label="map/dynamic cap")
            axis.plot(s_m, hard_curvature * 100.0, color="#d63031", alpha=0.45,
                      linewidth=0.9, label="hard Rmin cap")
            axis.plot(s_m, local_cap * 100.0, color="#2d3436", linestyle="--",
                      linewidth=1.0, label="design local cap")
            axis.plot(s_m, final * 100.0, color="#00b894", linewidth=1.5,
                      label="final offset")
            for label, color in CONSTRAINT_COLORS.items():
                indices = [
                    index for index, row in enumerate(rows)
                    if row["active_constraint"] == label
                ]
                if indices:
                    axis.scatter(s_m[indices], final[indices] * 100.0, s=5,
                                 color=color, label=label, zorder=4)
            axis.set_title(profile_path.stem.replace("_offset", ""))
            axis.set_xlabel("center s [m]")
            axis.set_ylabel("lateral offset [cm]")
            axis.grid(alpha=0.2)
        handles, labels = axes.flat[0].get_legend_handles_labels()
        figure.legend(handles, labels, loc="upper center", bbox_to_anchor=(0.5, 1.02),
                      ncol=4, fontsize=8)
        figure.suptitle("Local Tube caps, final offset, and active constraints")
        figure.savefig(args.out_dir / "offset_caps_constraints.png", dpi=190)
        plt.close(figure)

    for filename, header, rows in (
        ("curvature.csv", ("tube", "index", "s_m", "kappa_1pm"), curvature_rows),
        ("clearance.csv", ("tube", "index", "s_m", "clearance_m"), clearance_rows),
    ):
        with (args.out_dir / filename).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream, lineterminator="\n")
            writer.writerow(header)
            writer.writerows(rows)
    print(args.out_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
