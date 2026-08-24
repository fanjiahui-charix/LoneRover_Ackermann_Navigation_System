#!/usr/bin/env python3
"""Collect stationary gyro samples and save them in rad/s for analysis."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import numpy as np
import serial

from imu_protocol import FrameParser


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--seconds", type=float, default=120.0)
    parser.add_argument("--output", type=Path, default=Path("stationary_imu.csv"))
    parser.add_argument("--accel-lsb-per-g", type=float, default=16384.0)
    parser.add_argument("--gyro-lsb-per-dps", type=float, default=65.5)
    parser.add_argument("--g-window", type=float, default=0.03)
    parser.add_argument("--gyro-window-dps", type=float, default=3.0)
    args = parser.parse_args()

    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    frame_parser = FrameParser()
    rows: list[list[float | int]] = []
    start = time.time()
    end = time.monotonic() + args.seconds
    try:
        print("请保持 IMU 完全静止，开始采集。")
        while time.monotonic() < end:
            frame_parser.feed(ser.read(512))
            while True:
                sample = frame_parser.pop()
                if sample is None:
                    break
                acceleration = np.array([sample.ax, sample.ay, sample.az], dtype=float)
                acceleration /= args.accel_lsb_per_g
                gyro_dps = np.array([sample.gx, sample.gy, sample.gz], dtype=float)
                gyro_dps /= args.gyro_lsb_per_dps
                if abs(float(np.linalg.norm(acceleration)) - 1.0) > args.g_window:
                    continue
                if float(np.linalg.norm(gyro_dps)) > args.gyro_window_dps:
                    continue
                gyro_rad = gyro_dps * math.pi / 180.0
                rows.append([
                    time.time() - start, gyro_rad[0], gyro_rad[1], gyro_rad[2],
                    sample.gx, sample.gy, sample.gz,
                ])
    finally:
        ser.close()

    if not rows:
        raise RuntimeError("没有保留有效静止样本，请检查串口或放宽筛选阈值")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["t_sec", "gx", "gy", "gz", "gx_raw", "gy_raw", "gz_raw"])
        writer.writerows(rows)
    bias = np.mean(np.asarray(rows, dtype=float)[:, 1:4], axis=0)
    print(f"保留样本：{len(rows)}")
    print("gyro_bias(rad/s)=", bias.tolist())
    print(f"已保存：{args.output}")


if __name__ == "__main__":
    main()
