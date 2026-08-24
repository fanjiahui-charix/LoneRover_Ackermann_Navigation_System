#!/usr/bin/env python3
"""Plot native command/plant curvature and CTE against Smac arc length."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


CURVES = (
    ("kappa_path", "Smac path", "black", 2.2),
    ("kappa_raw", "MPPI raw", "#1f77b4", 1.5),
    ("kappa_nav", "smoothed nav", "#ff7f0e", 1.3),
    ("kappa_safe", "limited safe", "#2ca02c", 1.3),
    ("kappa_actual", "virtual actual", "#d62728", 1.8),
)


def read(path: Path) -> list[dict[str, float]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = []
        for raw in csv.DictReader(stream):
            row = {}
            for key, value in raw.items():
                try:
                    row[key] = float(value) if value else np.nan
                except ValueError:
                    row[key] = np.nan
            rows.append(row)
    return rows


def binned(rows: list[dict[str, float]], key: str, ds: float = 0.01):
    points = np.asarray([
        (row.get("s_m", np.nan), row.get(key, np.nan))
        for row in rows
        if np.isfinite(row.get("s_m", np.nan)) and
        np.isfinite(row.get(key, np.nan)) and
        (key == "kappa_path" or abs(row.get("v", 0.0)) > 0.01 or
         abs(row.get("raw_v", 0.0)) > 0.01)
    ])
    if not len(points):
        return np.asarray([]), np.asarray([])
    bins = np.round(points[:, 0] / ds).astype(int)
    x, y = [], []
    for value in np.unique(bins):
        selection = points[bins == value]
        x.append(float(np.median(selection[:, 0])))
        y.append(float(np.median(selection[:, 1])))
    return np.asarray(x), np.asarray(y)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", action="append", nargs=2, metavar=("LABEL", "TRACE"), required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--s-max", type=float, default=0.70)
    args = ap.parse_args()

    fig, axes = plt.subplots(
        len(args.run), 2, figsize=(12, 4.2 * len(args.run)), squeeze=False)
    for i, (label, trace) in enumerate(args.run):
        rows = read(Path(trace))
        ax_k, ax_e = axes[i]
        for key, curve_label, color, width in CURVES:
            x, y = binned(rows, key)
            mask = x <= args.s_max
            ax_k.plot(x[mask], y[mask], label=curve_label, color=color,
                      linewidth=width, alpha=0.9)
        x, y = binned(rows, "cte_m")
        mask = x <= args.s_max
        ax_e.plot(x[mask], y[mask], color="#7b2cbf", linewidth=1.8,
                  label="|CTE|")
        ax_e.axhline(0.05, color="#d62728", linestyle="--", linewidth=1.1,
                     label="5 cm")
        for axis in (ax_k, ax_e):
            axis.grid(True, alpha=0.25)
            axis.set_xlim(0.0, args.s_max)
            axis.set_xlabel("Smac path arc length s (m)")
            axis.legend(loc="best", fontsize=8)
        ax_k.set_ylabel("curvature (1/m)")
        ax_k.set_title(f"{label}: curvature chain")
        ax_e.set_ylabel("absolute CTE (m)")
        ax_e.set_title(f"{label}: tracking error")

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=180)
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
