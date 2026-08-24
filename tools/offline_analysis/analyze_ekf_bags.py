#!/usr/bin/env python3
import argparse
import csv
import math
import re
import statistics
from dataclasses import dataclass
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


TYPESTORE = get_typestore(Stores.ROS2_HUMBLE)


@dataclass
class Sample:
    t: float
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def unwrap(values):
    if not values:
        return values
    out = [values[0]]
    offset = 0.0
    prev = values[0]
    for value in values[1:]:
        delta = value - prev
        if delta > math.pi:
            offset -= 2.0 * math.pi
        elif delta < -math.pi:
            offset += 2.0 * math.pi
        out.append(value + offset)
        prev = value
    return out


def integrate(ts, values):
    if len(ts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(ts)):
        total += 0.5 * (values[i - 1] + values[i]) * (ts[i] - ts[i - 1])
    return total


def nearest(samples, t):
    if not samples:
        return None
    return min(samples, key=lambda sample: abs(sample.t - t))


def parse_scalar(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*([-+]?\d+(?:\.\d+)?)\s*$", text, re.M)
    return float(match.group(1)) if match else ""


def parse_string(text, key):
    match = re.search(rf"^\s*{re.escape(key)}:\s*(.+?)\s*$", text, re.M)
    return match.group(1).strip() if match else ""


def read_metadata(bag_root):
    readme = bag_root / "README.txt"
    text = readme.read_text(errors="replace") if readme.exists() else ""

    yaw = parse_scalar(text, "yaw_deg")
    direction = parse_string(text, "yaw_direction")
    if yaw != "" and direction.lower() in {"right", "cw", "clockwise"}:
        yaw = -abs(yaw)
    elif yaw != "" and direction.lower() in {"left", "ccw", "counterclockwise"}:
        yaw = abs(yaw)

    return {
        "scenario": parse_string(text, "scenario"),
        "linear_x": parse_scalar(text, "linear_x"),
        "angular_z": parse_scalar(text, "angular_z"),
        "motion_duration": parse_scalar(text, "motion_duration"),
        "measured_distance_m": parse_scalar(text, "distance_m"),
        "measured_yaw_deg": yaw,
        "measured_radius_m": parse_scalar(text, "radius_m"),
        "yaw_direction": direction,
        "reference_point": parse_string(text, "reference_point"),
        "note": parse_string(text, "note"),
    }


def summarize_bag(bagdir):
    data = {"raw": [], "ekf": [], "imu": [], "cmd": []}
    with AnyReader([bagdir], default_typestore=TYPESTORE) as reader:
        conns = [c for c in reader.connections if c.topic in {
            "/odom/data_raw", "/odom", "/imu/data_raw", "/cmd_vel"}]
        t0 = None
        for conn, stamp, raw in reader.messages(connections=conns):
            t = stamp * 1e-9
            if t0 is None:
                t0 = t
            tr = t - t0
            msg = reader.deserialize(raw, conn.msgtype)
            if conn.topic in ("/odom/data_raw", "/odom"):
                q = msg.pose.pose.orientation
                sample = Sample(
                    tr,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    yaw_from_quat(q),
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.angular.z,
                )
                data["raw" if conn.topic == "/odom/data_raw" else "ekf"].append(sample)
            elif conn.topic == "/imu/data_raw":
                data["imu"].append(Sample(tr, wz=msg.angular_velocity.z))
            elif conn.topic == "/cmd_vel":
                data["cmd"].append(Sample(tr, vx=msg.linear.x, wz=msg.angular.z))

    for key in ("raw", "ekf"):
        yaws = unwrap([sample.yaw for sample in data[key]])
        for sample, yaw in zip(data[key], yaws):
            sample.yaw = yaw
    return data


def motion_window(data):
    active = [s for s in data["cmd"] if abs(s.vx) > 0.01 or abs(s.wz) > 0.01]
    if active:
        return max(0.0, active[0].t - 0.15), active[-1].t + 0.15
    active = [s for s in data["raw"] if abs(s.vx) > 0.015 or abs(s.wz) > 0.015]
    if active:
        return max(0.0, active[0].t - 0.15), active[-1].t + 0.15
    if data["raw"]:
        return data["raw"][0].t, data["raw"][-1].t
    return 0.0, 0.0


def delta_pose(samples, start, end):
    a = nearest(samples, start)
    b = nearest(samples, end)
    if not a or not b:
        return None
    return {
        "dx": b.x - a.x,
        "dy": b.y - a.y,
        "distance": math.hypot(b.x - a.x, b.y - a.y),
        "dyaw": b.yaw - a.yaw,
    }


def values_in_window(samples, start, end):
    return [s for s in samples if start <= s.t <= end]


def classify(name, meta):
    scenario = str(meta.get("scenario") or name).lower()
    if "static" in scenario or "static" in name:
        return "static"
    if "straight" in scenario or "straight" in name:
        return "straight"
    if "circle_left" in scenario or "left" in scenario or "circle_left" in name:
        return "left_turn"
    if "circle_right" in scenario or "right" in scenario or "circle_right" in name:
        return "right_turn"
    return "other"


def safe_ratio(num, den):
    if num == "" or den in ("", 0.0) or abs(den) < 1e-9:
        return ""
    return num / den


def signed_error(estimate, truth):
    if estimate == "" or truth == "":
        return ""
    return estimate - truth


def summarize_all(root):
    rows = []
    for meta_file in sorted(root.glob("*/bag/metadata.yaml")):
        bag_root = meta_file.parents[1]
        name = bag_root.name
        if "invalid" in name:
            continue
        meta = read_metadata(bag_root)
        data = summarize_bag(meta_file.parent)
        start, end = motion_window(data)
        raw_delta = delta_pose(data["raw"], start, end) or {}
        ekf_delta = delta_pose(data["ekf"], start, end) or {}

        cmd_window = values_in_window(data["cmd"], start, end)
        raw_window = values_in_window(data["raw"], start, end)
        imu_window = values_in_window(data["imu"], start, end)

        cmd_dist = integrate([s.t for s in cmd_window], [s.vx for s in cmd_window])
        cmd_yaw = integrate([s.t for s in cmd_window], [s.wz for s in cmd_window])
        raw_v_dist = integrate([s.t for s in raw_window], [s.vx for s in raw_window])
        raw_w_yaw = integrate([s.t for s in raw_window], [s.wz for s in raw_window])
        imu_yaw = integrate([s.t for s in imu_window], [s.wz for s in imu_window])

        measured_dist = meta["measured_distance_m"]
        measured_yaw = meta["measured_yaw_deg"]
        raw_dist = raw_delta.get("distance", 0.0)
        raw_yaw = math.degrees(raw_delta.get("dyaw", 0.0))
        ekf_dist = ekf_delta.get("distance", 0.0)
        ekf_yaw = math.degrees(ekf_delta.get("dyaw", 0.0))
        imu_yaw_deg = math.degrees(imu_yaw)

        row = {
            "bag": name,
            "kind": classify(name, meta),
            "duration_s": end - start,
            "cmd_msgs": len(data["cmd"]),
            "cmd_dist_m": cmd_dist,
            "cmd_yaw_deg": math.degrees(cmd_yaw),
            "raw_pose_dist_m": raw_dist,
            "raw_vx_dist_m": raw_v_dist,
            "raw_pose_yaw_deg": raw_yaw,
            "raw_wz_yaw_deg": math.degrees(raw_w_yaw),
            "imu_yaw_deg": imu_yaw_deg,
            "ekf_dist_m": ekf_dist,
            "ekf_yaw_deg": ekf_yaw,
            "gt_dist_m": measured_dist,
            "gt_yaw_deg": measured_yaw,
            "raw_dist_scale_to_gt": safe_ratio(measured_dist, raw_dist),
            "raw_vx_scale_to_gt": safe_ratio(measured_dist, raw_v_dist),
            "cmd_dist_scale_to_gt": safe_ratio(measured_dist, cmd_dist),
            "raw_yaw_err_deg": signed_error(raw_yaw, measured_yaw),
            "raw_wz_yaw_err_deg": signed_error(math.degrees(raw_w_yaw), measured_yaw),
            "imu_yaw_err_deg": signed_error(imu_yaw_deg, measured_yaw),
            "ekf_yaw_err_deg": signed_error(ekf_yaw, measured_yaw),
            "ekf_dist_err_m": signed_error(ekf_dist, measured_dist),
            "reference_point": meta["reference_point"],
            "note": meta["note"],
        }
        rows.append(row)
    return rows


def mean(values):
    values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return statistics.fmean(values) if values else ""


def rms(values):
    values = [v for v in values if isinstance(v, (int, float)) and math.isfinite(v)]
    return math.sqrt(statistics.fmean([v * v for v in values])) if values else ""


def write_summary_md(rows, path):
    straight = [r for r in rows if r["kind"] == "straight" and r["gt_dist_m"] != ""]
    turns = [r for r in rows if r["kind"] in {"left_turn", "right_turn"} and r["gt_yaw_deg"] != ""]
    static = [r for r in rows if r["kind"] == "static"]

    lines = [
        "# EKF Bag Analysis Summary",
        "",
        "This file is generated by `tools/offline_analysis/analyze_ekf_bags.py` from valid bag data and README measurements.",
        "",
        "## Key Aggregates",
        "",
        f"- valid rows: {len(rows)}",
        f"- straight rows with measured distance: {len(straight)}",
        f"- turn rows with measured yaw: {len(turns)}",
    ]

    if straight:
        lines.extend([
            f"- mean raw pose distance scale: {mean([r['raw_dist_scale_to_gt'] for r in straight]):.4f}",
            f"- mean raw twist distance scale: {mean([r['raw_vx_scale_to_gt'] for r in straight]):.4f}",
            f"- straight EKF distance RMSE: {rms([r['ekf_dist_err_m'] for r in straight]):.4f} m",
        ])

    if turns:
        lines.extend([
            f"- turn raw yaw RMSE: {rms([r['raw_yaw_err_deg'] for r in turns]):.2f} deg",
            f"- turn IMU yaw RMSE: {rms([r['imu_yaw_err_deg'] for r in turns]):.2f} deg",
            f"- turn EKF yaw RMSE: {rms([r['ekf_yaw_err_deg'] for r in turns]):.2f} deg",
        ])

    if static:
        yaw_rates = []
        for r in static:
            duration = r["duration_s"]
            if duration:
                yaw_rates.append(math.radians(r["imu_yaw_deg"]) / duration)
        if yaw_rates:
            lines.append(f"- static IMU yaw-rate bias estimate: {mean(yaw_rates):.6f} rad/s")

    lines.extend(["", "## Valid Bags", ""])
    for r in rows:
        gt_dist = "" if r["gt_dist_m"] == "" else f", gt_dist={r['gt_dist_m']:.3f}m"
        gt_yaw = "" if r["gt_yaw_deg"] == "" else f", gt_yaw={r['gt_yaw_deg']:.1f}deg"
        lines.append(
            f"- `{r['bag']}` ({r['kind']}): raw_dist={r['raw_pose_dist_m']:.3f}m, "
            f"raw_yaw={r['raw_pose_yaw_deg']:.1f}deg, imu_yaw={r['imu_yaw_deg']:.1f}deg, "
            f"ekf_yaw={r['ekf_yaw_deg']:.1f}deg{gt_dist}{gt_yaw}"
        )

    path.write_text("\n".join(lines) + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", default="/root/ekf_bags_car")
    parser.add_argument("--out", required=True)
    parser.add_argument("--summary-md", required=True)
    args = parser.parse_args()

    rows = summarize_all(Path(args.root))
    fieldnames = list(rows[0].keys()) if rows else []
    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)

    write_summary_md(rows, Path(args.summary_md))
    print(f"wrote {args.out} rows={len(rows)}")
    print(f"wrote {args.summary_md}")


if __name__ == "__main__":
    main()
