#!/usr/bin/env python3
"""Remove moving and wrongly oriented samples before six-face fitting."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np


def normalize_pose(value: str) -> str:
    return value.strip().upper().replace("+X", "+X").replace("-X", "-X")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--accel-lsb-per-g", type=float, default=16384.0)
    parser.add_argument("--norm-low", type=float, default=0.97)
    parser.add_argument("--norm-high", type=float, default=1.03)
    parser.add_argument("--axis-threshold", type=float, default=0.85)
    args = parser.parse_args()

    with args.input.open("r", encoding="utf-8-sig", newline="") as source:
        reader = csv.DictReader(source)
        if not reader.fieldnames:
            raise ValueError("输入 CSV 没有表头")
        pose_column = "pose" if "pose" in reader.fieldnames else "face"
        rows = list(reader)

    kept = []
    input_count: dict[str, int] = {}
    output_count: dict[str, int] = {}
    for row in rows:
        pose = row[pose_column].strip().upper()
        input_count[pose] = input_count.get(pose, 0) + 1
        acceleration = np.array([float(row[axis]) for axis in ("ax", "ay", "az")])
        acceleration /= args.accel_lsb_per_g
        norm = float(np.linalg.norm(acceleration))
        axis = pose[-1].lower() if pose[-1].lower() in "xyz" else ""
        sign = 1.0 if pose.startswith("+") else -1.0
        main_axis = {"x": acceleration[0], "y": acceleration[1], "z": acceleration[2]}.get(axis, 0.0)
        if not args.norm_low <= norm <= args.norm_high:
            continue
        if sign * main_axis < args.axis_threshold:
            continue
        kept.append(row)
        output_count[pose] = output_count.get(pose, 0) + 1

    if not kept:
        raise ValueError("筛选后没有样本，请放宽模长窗口或主轴阈值")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as target:
        writer = csv.DictWriter(target, fieldnames=kept[0].keys())
        writer.writeheader()
        writer.writerows(kept)

    print(f"已保存：{args.output}")
    for pose in sorted(input_count):
        print(f"{pose:>2}: {input_count[pose]} -> {output_count.get(pose, 0)}")
