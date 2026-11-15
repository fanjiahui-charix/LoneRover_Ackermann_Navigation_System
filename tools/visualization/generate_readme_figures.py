#!/usr/bin/env python3
"""Render README figures from Tube paths, lookup tables and servo assets.

Offline generation reads the published repository assets directly.
Dependencies: matplotlib, numpy, Pillow, PyYAML.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import re
import struct
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.colors import BoundaryNorm, ListedColormap
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from PIL import Image


INK = "#182d42"
MUTED = "#596d80"
LANES = {"inner": "#009b8d", "center": "#de8b1a", "outer": "#4977c7"}
SIDES = {"left": "#008c83", "right": "#bb5b92"}
CONE_CLASSES = {
    1: ("#eeb9c4", "Inner-side cone → Outer Tube"),
    2: ("#b1d1ec", "Outer-side cone → Inner Tube"),
}
CHANNEL_V2_ADMISSION_SHA256 = "f1b8d865214c67f7f4536806c66f5de47b5a055918a2a5f57fcfb9e251837870"
ENTRY_HEADER = struct.Struct("<4sIIIIdddii")
GOAL_HEADER = struct.Struct("<5sII")


def configure_style() -> None:
    plt.rcParams.update({
        "font.family": "DejaVu Sans", "font.size": 10,
        "text.color": INK, "axes.labelcolor": INK,
        "axes.edgecolor": "#c7d1db", "xtick.color": MUTED,
        "ytick.color": MUTED, "axes.titleweight": "bold",
        "axes.titlesize": 12, "axes.titlelocation": "left",
        "figure.facecolor": "#ffffff", "axes.facecolor": "#ffffff",
        "svg.fonttype": "none", "svg.hashsalt": "lonerover-readme-figures-v1",
        "savefig.facecolor": "#ffffff",
    })


def save_figure(figure, output: Path, name: str) -> None:
    figure.savefig(output / f"{name}.png", dpi=160)
    svg_path = output / f"{name}.svg"
    figure.savefig(svg_path, metadata={"Date": None})
    svg_path.write_text(
        "\n".join(line.rstrip() for line in svg_path.read_text(encoding="utf-8").splitlines()) + "\n",
        encoding="utf-8",
    )
    plt.close(figure)


def read_map(path: Path) -> tuple[np.ndarray, tuple[float, ...], dict, Path]:
    metadata = yaml.safe_load(path.read_text(encoding="utf-8"))
    if float(metadata["origin"][2]) != 0.0:
        raise ValueError("the figure currently requires an axis-aligned map")
    image_path = path.parent / metadata["image"]
    with Image.open(image_path) as image:
        pixels = np.asarray(image.convert("L"))
    occupancy = pixels / 255.0 if metadata.get("negate", 0) else 1 - pixels / 255.0
    classified = np.ones_like(pixels)
    classified[occupancy < metadata["free_thresh"]] = 0
    classified[occupancy > metadata["occupied_thresh"]] = 2
    x, y, _ = metadata["origin"]
    resolution = float(metadata["resolution"])
    extent = (x, x + pixels.shape[1] * resolution,
              y, y + pixels.shape[0] * resolution)
    return classified, extent, metadata, image_path


def read_admitted_tubes(directory: Path) -> tuple[dict[str, np.ndarray], dict]:
    admission_path = directory / "runtime_admission.csv"
    raw = admission_path.read_bytes()
    admission_sha256 = hashlib.sha256(raw).hexdigest()
    if admission_sha256 != CHANNEL_V2_ADMISSION_SHA256:
        raise ValueError("Channel v2 admission differs from the published manifest identity")
    with admission_path.open(encoding="utf-8", newline="") as stream:
        admission_rows = list(csv.DictReader(stream))
    paths = {}
    for entry in admission_rows:
        if entry["kind"] != "tube":
            continue
        lane, direction = entry["source_lane"], entry["direction"]
        if lane not in LANES or direction not in {"cw", "ccw"}:
            raise ValueError("unrecognized Tube identity in Channel v2 admission")
        path = directory / entry["file"]
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError(f"Tube SHA-256 mismatch: {entry['file']}")
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        xy = np.asarray([(float(row["x_m"]), float(row["y_m"])) for row in rows])
        if len(rows) < 2 or not np.all(np.isfinite(xy)):
            raise ValueError(f"invalid Tube coordinates: {path}")
        paths[f"tube_{lane}_{direction}"] = xy
    if set(paths) != {f"tube_{lane}_{direction}" for lane in LANES for direction in ("cw", "ccw")}:
        raise ValueError("Channel v2 admission must provide exactly six Tube paths")
    map_rows = [row for row in admission_rows if row["kind"] == "map"]
    if len(map_rows) != 1:
        raise ValueError("Channel v2 admission must identify one static map")
    return paths, {
        "static_map_sha256": map_rows[0]["sha256"],
        "avoid_entries": {row["direction"]: row for row in admission_rows if row["kind"] == "avoid"},
    }


def read_obstacle_luts(directory: Path, entries: dict, shape: tuple[int, int]) -> dict:
    if set(entries) != {"cw", "ccw"}:
        raise ValueError("the admitted obstacle lookup needs both directions")
    tables = {}
    for direction, entry in entries.items():
        path = directory / entry["file"]
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != entry["sha256"]:
            raise ValueError(f"obstacle LUT SHA-256 mismatch: {entry['file']}")
        if len(payload) != shape[0] * shape[1]:
            raise ValueError("obstacle LUT dimensions disagree with the map")
        codes = np.frombuffer(payload, dtype=np.uint8).reshape(shape)
        if np.any(codes > 2):
            raise ValueError("obstacle LUT contains an unknown cone-side code")
        tables[direction] = codes
    return tables


def read_reverse_entry_lut(entry_path: Path, goal_path: Path) -> dict:
    entry = entry_path.read_bytes()
    if len(entry) < ENTRY_HEADER.size:
        raise ValueError("RGE2 file is shorter than its header")
    magic, sx, sy, yaw_bins, mask_len, xmin, ymin, res, yaw_min, yaw_step = (
        ENTRY_HEADER.unpack_from(entry)
    )
    states = sx * sy * yaw_bins
    if magic != b"RGE2" or mask_len != (states + 7) // 8:
        raise ValueError("invalid RGE2 header")
    if len(entry) != ENTRY_HEADER.size + mask_len:
        raise ValueError("RGE2 payload length disagrees with its header")
    admitted = np.unpackbits(
        np.frombuffer(entry[ENTRY_HEADER.size:], dtype=np.uint8), bitorder="little",
    )[:states].astype(bool)
    goals = goal_path.read_bytes()
    if len(goals) < GOAL_HEADER.size:
        raise ValueError("RGEG2 file is shorter than its header")
    goal_magic, goal_states, goal_count = GOAL_HEADER.unpack_from(goals)
    offset = GOAL_HEADER.size + goal_count * 24
    if goal_magic != b"RGEG2" or goal_states != states or len(goals) != offset + states:
        raise ValueError("RGEG2 file disagrees with the RGE2 state grid")
    goal_ids = np.frombuffer(goals[offset:], dtype=np.uint8)
    if not np.all(goal_ids[admitted] < goal_count):
        raise ValueError("an admitted reverse-entry state lacks a valid goal ID")
    density = admitted.reshape(yaw_bins, sy, sx).sum(axis=0)
    return {
        "density": density, "x": xmin + np.arange(sx) * res,
        "y": ymin + np.arange(sy) * res, "resolution_m": res,
        "yaw_min_deg": yaw_min, "yaw_max_deg": yaw_min + (yaw_bins - 1) * yaw_step,
        "yaw_step_deg": yaw_step, "yaw_bins": yaw_bins,
        "states": int(admitted.sum()), "positions": int(np.count_nonzero(density)),
    }


def draw_map(axis, pixels: np.ndarray, extent: tuple[float, ...]) -> None:
    axis.imshow(pixels, origin="upper", extent=extent, interpolation="nearest",
                cmap=ListedColormap(["#f5f7f9", "#c2cbd3", "#273e50"]), vmin=0, vmax=2)
    axis.set_aspect("equal")
    axis.set_xlabel("Map x [m]")
    axis.set_ylabel("Map y [m]")
    axis.tick_params(length=0, pad=6)


def draw_paths(axis, paths: dict[str, np.ndarray]) -> None:
    for lane, color in LANES.items():
        for direction in ("cw", "ccw"):
            xy = paths[f"tube_{lane}_{direction}"]
            axis.plot(xy[:, 0], xy[:, 1], color=color, linewidth=1.65,
                      linestyle="-" if direction == "cw" else (0, (3.0, 3.0)),
                      alpha=0.96, zorder=3)


def make_navigation_figure(root: Path, output: Path, asset_directory: Path) -> None:
    nav = root / "src/hobot_navigation/hobot_nav"
    map_yaml = nav / "maps/rdk_2026_hospital_static_1cm.yaml"
    pixels, extent, metadata, map_image = read_map(map_yaml)
    paths, metrics = read_admitted_tubes(asset_directory)
    if hashlib.sha256(map_image.read_bytes()).hexdigest() != metrics["static_map_sha256"]:
        raise ValueError("the plotted map differs from the Channel v2 admission map")
    obstacle_luts = read_obstacle_luts(asset_directory, metrics["avoid_entries"], pixels.shape)
    reverse = read_reverse_entry_lut(
        nav / "runtime/reverse_gate_entry_clean.bin",
        nav / "runtime/reverse_gate_goal_id_clean.bin",
    )
    figure = plt.figure(figsize=(14, 8.3))
    figure.text(0.055, 0.948, "TUBE PATHS AND LUT", fontsize=21, weight="bold")
    figure.text(0.055, 0.908,
                 "Channel obstacle lookup  ·  reverse-entry lookup  ·  1 cm static map",
                 fontsize=11, color=MUTED)
    overview = figure.add_axes([0.065, 0.235, 0.445, 0.605])
    detail = figure.add_axes([0.615, 0.235, 0.29, 0.605])
    draw_map(overview, pixels, extent)
    overview.imshow(
        np.ma.masked_equal(obstacle_luts["cw"], 0), origin="lower", extent=extent,
        interpolation="nearest", vmin=0, vmax=2, zorder=2,
        cmap=ListedColormap(["#ffffff", *(color for color, _ in CONE_CLASSES.values())]),
    )
    draw_paths(overview, paths)
    overview.set_title("A  Tube paths and obstacle-side LUT (CW)", pad=14)
    overview.set_xlim(extent[:2])
    overview.set_ylim(extent[2:])
    draw_map(detail, pixels, extent)
    density = reverse["density"]
    res = reverse["resolution_m"]
    color_map = plt.get_cmap("PuRd", reverse["yaw_bins"])
    mesh = detail.pcolormesh(
        np.append(reverse["x"] - res/2, reverse["x"][-1] + res/2),
        np.append(reverse["y"] - res/2, reverse["y"][-1] + res/2),
        np.ma.masked_equal(density, 0), cmap=color_map,
        norm=BoundaryNorm(np.arange(0.5, reverse["yaw_bins"] + 1.5), color_map.N),
        shading="flat", edgecolors="white", linewidth=0.3, zorder=2,
    )
    active_y, active_x = np.nonzero(density)
    x0, x1 = reverse["x"][active_x.min()] - 2*res, reverse["x"][active_x.max()] + 2*res
    y0, y1 = reverse["y"][active_y.min()] - 2*res, reverse["y"][active_y.max()] + 2*res
    detail.set_xlim(x0, x1)
    detail.set_ylim(y0, y1)
    detail.set_xticks(np.arange(np.ceil(x0*5)/5, x1, 0.2))
    detail.set_yticks(np.arange(np.ceil(y0*5)/5, y1, 0.2))
    detail.set_title("B  Reverse-entry LUT", pad=14)
    color_axis = figure.add_axes([0.925, 0.326, 0.012, 0.37])
    colorbar = figure.colorbar(mesh, cax=color_axis, ticks=[1, 5, 10, 15, reverse["yaw_bins"]])
    colorbar.ax.tick_params(labelsize=8, length=0)
    colorbar.set_label("Admitted yaw bins per x/y cell", fontsize=9, labelpad=9)
    legend = [Line2D([], [], color=color, lw=2.5, label=lane.title())
              for lane, color in LANES.items()]
    legend += [Line2D([], [], color=MUTED, lw=1.7, label="CW"),
               Line2D([], [], color=MUTED, lw=1.7, ls=(0, (3, 3)), label="CCW")]
    figure.legend(handles=legend, loc="lower left", bbox_to_anchor=(0.065, 0.137),
                  ncol=5, frameon=False, fontsize=9, columnspacing=1.1, handlelength=2)
    cone_legend = [Patch(facecolor=color, edgecolor="none", label=label)
                   for color, label in CONE_CLASSES.values()]
    figure.legend(handles=cone_legend, loc="lower left", bbox_to_anchor=(0.065, 0.097),
                  ncol=2, frameon=False, fontsize=8.5, columnspacing=1.4, handlelength=1.4)
    figure.text(0.065, 0.075,
                 f"Obstacle LUT (CW): {np.count_nonzero(obstacle_luts['cw']):,} cone-position cells",
                 fontsize=9.2, color=MUTED)
    figure.text(0.615, 0.156,
                 f"{reverse['states']:,} states  |  {reverse['positions']} positions",
                 fontsize=11, weight="bold")
    figure.text(0.615, 0.126,
                 f"Position grid {res:g} m; yaw {reverse['yaw_min_deg']}–{reverse['yaw_max_deg']}° "
                 f"in {reverse['yaw_step_deg']}° steps", fontsize=9, color=MUTED)
    save_figure(figure, output, "navigation_map_and_lut")


def read_servo_segments(source: str, side: str) -> np.ndarray:
    match = re.search(rf"k_{side}_pwm_to_angle\[\]\s*=\s*\{{(.*?)\n\}};", source, re.S)
    if not match:
        raise ValueError(f"could not find frozen {side} PWM-to-angle segments")
    rows = [tuple(float(value.strip().removesuffix("f")) for value in row.split(","))
            for row in re.findall(r"\{([^{}]+)\}", match.group(1))]
    result = np.asarray(rows)
    if result.ndim != 2 or result.shape[1] != 6:
        raise ValueError(f"invalid frozen servo segments: {side}")
    return result


def evaluate_segments(segments: np.ndarray, pwm: np.ndarray) -> np.ndarray:
    command = np.clip(pwm, segments[0, 0], segments[-1, 1])
    indices = np.searchsorted(segments[:, 1], command, side="left")
    chosen = segments[indices]
    delta = command - chosen[:, 0]
    return np.rad2deg(((chosen[:, 2] * delta + chosen[:, 3]) * delta +
                      chosen[:, 4]) * delta + chosen[:, 5])


def make_servo_figure(root: Path, output: Path) -> None:
    calibration = root / "tools/calibration"
    c_source = calibration / "servo_fit.c"
    source = c_source.read_text(encoding="utf-8")
    figure = plt.figure(figsize=(14, 7.4))
    figure.text(0.065, 0.94, "MEASURED STEERING, ASYMMETRIC CALIBRATION",
                 fontsize=19, weight="bold")
    figure.text(0.065, 0.897,
                 "Published vehicle profile: measured angle means and frozen PCHIP lookup segments",
                 fontsize=11, color=MUTED)
    left = figure.add_axes([0.075, 0.27, 0.48, 0.54])
    right = figure.add_axes([0.66, 0.27, 0.28, 0.54])
    for side, color in SIDES.items():
        csv_path = calibration / f"data/servo_{side}_pwm_angle.csv"
        with csv_path.open(encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
        points = np.asarray([(float(r["pwm"]), float(r["angle_deg"]),
                              int(r["sample_count"])) for r in rows])
        measured = points[points[:, 2] > 0]
        neutral = points[points[:, 2] == 0]
        if len(neutral) != 1 or not np.array_equal(neutral[0, :2], [1500, 0]):
            raise ValueError("expected the documented neutral reference at PWM 1500")
        segments = read_servo_segments(source, side)
        pwm = np.linspace(segments[0, 0], segments[-1, 1], 700)
        angles = evaluate_segments(segments, pwm)
        residual = float(np.max(np.abs(evaluate_segments(segments, measured[:, 0]) - measured[:, 1])))
        if residual > 0.01:
            raise ValueError(f"frozen {side} curve and measured means disagree: {residual} deg")
        left.plot(pwm, angles, color=color, lw=2.3, label=f"{side.title()} PCHIP")
        left.scatter(measured[:, 0], measured[:, 1], s=40, color=color,
                     edgecolor="white", linewidth=1.2, zorder=4)
        right.plot(np.abs(pwm - 1500), np.abs(angles), color=color, lw=2.3,
                   label=f"{side.title()} turn")
        right.scatter(np.abs(measured[:, 0] - 1500), np.abs(measured[:, 1]), s=35,
                      color=color, edgecolor="white", linewidth=1, zorder=4)
        farthest = measured[np.argmax(np.abs(measured[:, 1]))]
        right.annotate(f"{abs(farthest[1]):g}°", (abs(farthest[0] - 1500), abs(farthest[1])),
                       xytext=(-4, 10), textcoords="offset points",
                       ha="right", color=color, fontsize=11, weight="bold")
    for axis in (left, right):
        axis.grid(color="#e4e9ee", linewidth=0.8, zorder=0)
        axis.spines[["top", "right"]].set_visible(False)
        axis.tick_params(length=0, pad=7)
        axis.set_axisbelow(True)
    left.scatter([1500], [0], marker="D", s=54, facecolor="white", edgecolor=MUTED,
                 lw=1.5, zorder=5)
    left.axhline(0, color="#bcc8d3", lw=0.8, zorder=0)
    left.axvline(1500, color="#bcc8d3", lw=0.8, ls=(0, (3, 3)), zorder=0)
    left.annotate("Neutral reference\n1500, 0°; n = 0", xy=(1500, 0),
                  xytext=(1570, 12), fontsize=9, color=MUTED,
                  arrowprops={"arrowstyle": "-", "color": "#9cabb9", "lw": 0.8})
    left.set_title("A  PWM command to signed steering angle", pad=14)
    left.set_xlabel("Servo PWM command")
    left.set_ylabel("Steering angle [deg]")
    left.set_xlim(760, 2240)
    left.set_ylim(-36, 49)
    left.set_xticks([800, 1000, 1200, 1500, 1800, 2000, 2200])
    right.set_title("B  Left/right asymmetry", pad=14)
    right.set_xlabel("Absolute PWM offset from 1500")
    right.set_ylabel("Absolute steering angle [deg]")
    right.set_xlim(-20, 745)
    right.set_ylim(0, 49)
    right.set_xticks([0, 200, 400, 600])
    right.legend(frameon=False, loc="upper left", fontsize=9)
    legend = [Line2D([], [], color=SIDES[side], lw=2.3, label=f"{side.title()} frozen PCHIP")
              for side in SIDES]
    legend += [Line2D([], [], color=MUTED, marker="o", lw=0, markersize=6,
                      label="Measured CSV mean"),
               Line2D([], [], color=MUTED, marker="D", markerfacecolor="white", lw=0,
                      markersize=6, label="Neutral reference")]
    figure.legend(handles=legend, loc="lower left", bbox_to_anchor=(0.065, 0.16),
                  ncol=4, frameon=False, fontsize=9)
    figure.text(0.075, 0.117,
                 "Seven nonzero measured means per side; each mean aggregates 4–6 readings.",
                 fontsize=9.2, color=MUTED)
    figure.text(0.075, 0.078,
                 "Sources: tools/calibration/data/servo_{left,right}_pwm_angle.csv and "
                 "tools/calibration/servo_fit.c (PWM → angle tables).",
                 fontsize=8.5, color=MUTED)
    save_figure(figure, output, "servo_pchip_calibration")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument("--output-dir", type=Path, help="defaults to <repo-root>/docs/assets")
    parser.add_argument("--tube-asset-dir", type=Path, help="defaults to hobot_nav/runtime/channel_v2")
    args = parser.parse_args()
    root = args.repo_root.resolve()
    output = args.output_dir.resolve() if args.output_dir else root / "docs/assets"
    output.mkdir(parents=True, exist_ok=True)
    configure_style()
    asset_directory = (args.tube_asset_dir.resolve() if args.tube_asset_dir else
                       root / "src/hobot_navigation/hobot_nav/runtime/channel_v2")
    make_navigation_figure(root, output, asset_directory)
    make_servo_figure(root, output)
    for stem in ("navigation_map_and_lut", "servo_pchip_calibration"):
        for extension in ("png", "svg"):
            print(output / f"{stem}.{extension}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
