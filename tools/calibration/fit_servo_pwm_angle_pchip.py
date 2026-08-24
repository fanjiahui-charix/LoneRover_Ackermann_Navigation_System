#!/usr/bin/env python3
"""Fit an asymmetric servo-PWM <-> steering-angle lookup table.

The calibration used on the race vehicle is not a single symmetric linear
function.  The left and right sides are measured independently and fitted as
monotone piecewise cubic curves (PCHIP).  This script accepts either:

* a CSV with ``pwm`` and ``angle_deg`` columns; or
* an Excel sheet whose first row contains PWM values and whose following rows
  contain repeated angle measurements for each PWM column.

It exports both directions because the lower controller needs angle -> PWM,
while offline odometry and vehicle-model checks often need PWM -> angle.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.interpolate import PchipInterpolator


def read_measurements(path: Path, side: str) -> pd.DataFrame:
    if path.suffix.lower() in {".xls", ".xlsx"}:
        raw = pd.read_excel(path, header=None)
        records: list[dict[str, float | int | str]] = []
        for column in range(raw.shape[1]):
            pwm = pd.to_numeric(raw.iloc[0, column], errors="coerce")
            if pd.isna(pwm):
                continue
            angles = pd.to_numeric(raw.iloc[1:, column], errors="coerce").dropna()
            if angles.empty:
                continue
            records.append({
                "side": side,
                "pwm": int(round(float(pwm))),
                "angle_deg": float(angles.mean()),
                "sample_count": int(angles.size),
            })
        result = pd.DataFrame(records)
    else:
        result = pd.read_csv(path)
        required = {"pwm", "angle_deg"}
        missing = required.difference(result.columns)
        if missing:
            raise ValueError(f"{path} 缺少列: {', '.join(sorted(missing))}")
        result = result[["pwm", "angle_deg"]].copy()
        result["pwm"] = pd.to_numeric(result["pwm"], errors="coerce")
        result["angle_deg"] = pd.to_numeric(result["angle_deg"], errors="coerce")
        result = result.dropna(subset=["pwm", "angle_deg"])
        result = (
            result.groupby("pwm", as_index=False)
            .agg(angle_deg=("angle_deg", "mean"), sample_count=("angle_deg", "size"))
        )
        result["side"] = side

    if result.empty:
        raise ValueError(f"{path} 没有有效的 PWM/角度测量值")

    result["pwm"] = result["pwm"].astype(float)
    result["angle_deg"] = result["angle_deg"].astype(float)
    result = result.drop_duplicates(subset=["pwm"], keep="last")
    result = result.sort_values("angle_deg").reset_index(drop=True)
    result = pd.concat(
        [result, pd.DataFrame([{
            "side": side,
            "pwm": 1500.0,
            "angle_deg": 0.0,
            "sample_count": 0,
        }])],
        ignore_index=True,
    )
    result = result.drop_duplicates(subset=["pwm"], keep="last")
    result = result.sort_values("angle_deg").reset_index(drop=True)
    if result["angle_deg"].duplicated().any():
        raise ValueError(f"{side} 的角度不是严格单调的，无法建立可逆 PCHIP")
    return result


def export_segments(
    x: np.ndarray,
    y: np.ndarray,
    output: Path,
    x_name: str,
    y_name: str,
) -> None:
    fit = PchipInterpolator(x, y)
    rows = []
    for index in range(len(x) - 1):
        rows.append({
            "segment": index,
            f"x0_{x_name}": x[index],
            f"x1_{x_name}": x[index + 1],
            "c0": fit.c[0, index],
            "c1": fit.c[1, index],
            "c2": fit.c[2, index],
            "c3": fit.c[3, index],
        })
    pd.DataFrame(rows).to_csv(output, index=False, encoding="utf-8-sig")


def fit_side(points: pd.DataFrame, output: Path, side: str) -> None:
    angle_rad = np.deg2rad(points["angle_deg"].to_numpy(dtype=float))
    pwm = points["pwm"].to_numpy(dtype=float)
    points.to_csv(output / f"{side}_clean_points.csv", index=False, encoding="utf-8-sig")

    export_segments(
        angle_rad,
        pwm,
        output / f"{side}_angle_to_pwm_segments.csv",
        "rad",
        "pwm",
    )

    pwm_order = np.argsort(pwm)
    export_segments(
        pwm[pwm_order],
        angle_rad[pwm_order],
        output / f"{side}_pwm_to_angle_segments.csv",
        "pwm",
        "rad",
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", type=Path, required=True, help="左转 CSV/XLSX")
    parser.add_argument("--right", type=Path, required=True, help="右转 CSV/XLSX")
    parser.add_argument("--output", type=Path, required=True, help="输出目录")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    left = read_measurements(args.left, "left")
    right = read_measurements(args.right, "right")
    fit_side(left, args.output, "left")
    fit_side(right, args.output, "right")

    print(f"已完成左右转 PCHIP 拟合，输出目录：{args.output}")
    print("angle -> PWM 和 PWM -> angle 的分段系数已分别导出。")


if __name__ == "__main__":
    main()
