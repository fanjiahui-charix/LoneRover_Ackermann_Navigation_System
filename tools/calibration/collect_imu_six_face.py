#!/usr/bin/env python3
"""Collect six-face raw IMU samples from the STM32 serial interface."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import serial

from imu_protocol import FrameParser


POSES = ("+X", "-X", "+Y", "-Y", "+Z", "-Z")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--duration", type=float, default=60.0,
                        help="每个面采集秒数，建议 30~120")
    parser.add_argument("--output", type=Path, default=Path("accel_6pose.csv"))
    args = parser.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)
    ser = serial.Serial(args.port, baudrate=args.baud, timeout=0.1)
    frame_parser = FrameParser()
    start = time.time()

    try:
        with args.output.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow([
                "t_sec", "pose", "ax", "ay", "az", "gx", "gy", "gz",
                "mx", "my", "mz", "temp_raw",
            ])
            print("姿态顺序：+X -X +Y -Y +Z -Z；每个面保持静止后按回车开始。")
            for pose in POSES:
                input(f"请将 {pose} 轴朝上并固定，按回车采集 {args.duration:.1f}s：")
                end = time.monotonic() + args.duration
                count = 0
                while time.monotonic() < end:
                    frame_parser.feed(ser.read(512))
                    while True:
                        sample = frame_parser.pop()
                        if sample is None:
                            break
                        writer.writerow([
                            f"{time.time() - start:.6f}", pose, sample.ax, sample.ay,
                            sample.az, sample.gx, sample.gy, sample.gz, sample.mx,
                            sample.my, sample.mz, sample.temp_raw,
                        ])
                        count += 1
                stream.flush()
                print(f"{pose}: {count} 帧")
    finally:
        ser.close()

    print(f"已保存六面原始数据：{args.output}")


if __name__ == "__main__":
    main()
