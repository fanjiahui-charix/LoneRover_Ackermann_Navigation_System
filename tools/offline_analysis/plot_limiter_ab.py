#!/usr/bin/env python3
"""Plot native virtual-vehicle lateral-limiter A/B evidence in path-station space."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt


BLUE = "#2563EB"
BLUE_DARK = "#1E3A8A"
ORANGE = "#D97706"
PURPLE = "#7C3AED"
INK = "#20242A"
GRID = "#D9DEE7"


def read_trace(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return [
            {key: float(value) for key, value in row.items() if value != ""}
            for row in csv.DictReader(stream)
        ]


def finite_xy(rows: list[dict[str, float]], key: str) -> tuple[list[float], list[float]]:
    x: list[float] = []
    y: list[float] = []
    for row in rows:
        value = row.get(key, float("nan"))
        if math.isfinite(value):
            x.append(row["s_m"])
            y.append(value)
    return x, y


def moving_rows(rows: list[dict[str, float]]) -> list[dict[str, float]]:
    first = next((
        index for index, row in enumerate(rows)
        if max(abs(row.get(key, 0.0)) for key in
               ("raw_v", "nav_v", "safe_v", "v")) > 0.01), 0)
    return rows[first:]


def style_axis(axis: plt.Axes) -> None:
    axis.set_facecolor("#FBFCFE")
    axis.grid(True, color=GRID, linewidth=0.7, alpha=0.75)
    axis.spines["top"].set_visible(False)
    axis.spines["right"].set_visible(False)
    axis.spines["left"].set_color("#9AA4B2")
    axis.spines["bottom"].set_color("#9AA4B2")
    axis.tick_params(colors="#505965", labelsize=8)


def plot_column(axes: list[plt.Axes], trace_path: Path,
                eval_path: Path, column_title: str) -> None:
    rows = moving_rows(read_trace(trace_path))
    report = json.loads(eval_path.read_text(encoding="utf-8"))
    timing = report["limiter_timing"]

    for axis in axes:
        style_axis(axis)
        active = timing.get("limiter_first_active")
        if active:
            axis.axvline(
                active["s_m"], color=ORANGE, linestyle="--", linewidth=1.3,
                label="limiter first active")
        preview_active = next((
            row for row in rows
            if row.get("speed_limit", 0.50) < 0.499), None)
        if preview_active:
            axis.axvline(
                preview_active["s_m"], color=PURPLE, linestyle=":",
                linewidth=1.3, label="preview speed cap active")

    axis = axes[0]
    for key, color, linestyle, width, label in (
            ("kappa_path", INK, ":", 2.0, "Smac path κ"),
            ("kappa_raw", BLUE, "-", 1.25, "MPPI raw κ"),
            ("kappa_nav", BLUE_DARK, "--", 1.0, "nav κ"),
            ("kappa_safe", ORANGE, "-", 1.3, "safe κ"),
            ("kappa_actual", INK, "-.", 1.0, "actual κ")):
        x, y = finite_xy(rows, key)
        axis.plot(x, y, color=color, linestyle=linestyle,
                  linewidth=width, alpha=0.88, label=label)
    axis.set_ylabel("Curvature κ (1/m)")
    axis.set_title(column_title, fontsize=10, color=INK, weight="semibold")
    axis.set_ylim(-3.5, 3.5)

    axis = axes[1]
    for key, color, linestyle, width, label in (
            ("raw_v", BLUE, "-", 1.2, "raw v"),
            ("nav_v", BLUE_DARK, "--", 1.0, "nav v"),
            ("safe_v", ORANGE, "-", 1.5, "safe v"),
            ("v", INK, "-.", 1.2, "actual v")):
        x, y = finite_xy(rows, key)
        axis.plot(x, [abs(value) for value in y], color=color,
                  linestyle=linestyle, linewidth=width, alpha=0.9,
                  label=label)
    x, y = finite_xy(rows, "speed_limit")
    if x:
        axis.plot(x, y, color=PURPLE, linestyle=":", linewidth=1.45,
                  alpha=0.9, label="Nav2 speed limit")
    axis.set_ylabel("Speed (m/s)")
    axis.set_ylim(-0.01, 0.52)

    axis = axes[2]
    x, cte = finite_xy(rows, "cte_m")
    axis.plot(x, cte, color=BLUE, linewidth=1.35, label="absolute CTE")
    x, clearance = finite_xy(rows, "footprint_clearance_m")
    axis.plot(x, clearance, color=ORANGE, linewidth=1.25,
              label="actual footprint clearance")
    axis.axhline(0.05, color="#707985", linestyle=":", linewidth=1.0,
                 label="CTE 5 cm")
    axis.axhline(0.0, color=INK, linewidth=0.8)
    axis.set_ylabel("Distance (m)")
    axis.set_xlabel("Projected Smac path station s (m)")
    axis.set_ylim(-0.01, max(0.40, max(clearance, default=0.0) * 1.05))

    for axis in axes:
        handles, labels = axis.get_legend_handles_labels()
        unique: dict[str, object] = {}
        for handle, label in zip(handles, labels):
            unique.setdefault(label, handle)
        axis.legend(unique.values(), unique.keys(), loc="best", fontsize=7,
                    frameon=True, facecolor="white", edgecolor=GRID,
                    framealpha=0.92)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-a", type=Path, required=True)
    parser.add_argument("--eval-a", type=Path, required=True)
    parser.add_argument("--label-a", required=True)
    parser.add_argument("--trace-b", type=Path, required=True)
    parser.add_argument("--eval-b", type=Path, required=True)
    parser.add_argument("--label-b", required=True)
    parser.add_argument("--title", required=True)
    parser.add_argument("--subtitle", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    figure, grid = plt.subplots(3, 2, figsize=(14, 10), sharex="col")
    figure.patch.set_facecolor("white")
    plot_column(list(grid[:, 0]), args.trace_a, args.eval_a, args.label_a)
    plot_column(list(grid[:, 1]), args.trace_b, args.eval_b, args.label_b)
    figure.suptitle(args.title, x=0.06, y=0.985, ha="left", fontsize=15,
                    color=INK, weight="bold")
    figure.text(0.06, 0.958, args.subtitle, ha="left", va="top",
                fontsize=9, color="#5A6472")
    figure.tight_layout(rect=(0.04, 0.035, 0.99, 0.94), h_pad=1.2, w_pad=1.4)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.out, dpi=180, bbox_inches="tight", facecolor="white")
    figure.savefig(args.out.with_suffix(".svg"), bbox_inches="tight",
                   facecolor="white")
    plt.close(figure)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
