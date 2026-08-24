#!/usr/bin/env python3
"""Filter vehicle-side Reverse->Tube witnesses by the runtime Gate audit.

Smac can return a path whose quaternion headings contain a coarse-grid jump
that the Supervisor's 0.35 m curvature/steering audit deliberately rejects.
The raw generator bit says a Smac request succeeded; this second pass makes
the installed bit mean "the saved witness also passes the runtime policy".
"""

from __future__ import annotations

import argparse
import json
import math
import struct
from collections import Counter
from pathlib import Path


def yaw_delta(a: float, b: float) -> float:
    return math.atan2(math.sin(b - a), math.cos(b - a))


def audit(item: dict) -> str | None:
    path = item.get("path", [])
    if len(path) < 2:
        return "path_too_short"
    if not all(len(p) == 3 and all(math.isfinite(float(v)) for v in p) for p in path):
        return "non_finite_pose"
    terminal = path[-1]
    if not (2.30 - 1.0e-9 <= float(terminal[1]) <= 2.70 + 1.0e-9 and
            2.30 - 1.0e-9 <= float(terminal[0]) <= 2.70 + 1.0e-9):
        return "terminal_outside_handoff"
    # Match auditPathAgainstGrid(): use the geometric circumcircle of
    # consecutive positions for Rmin.  Smac's coarse-grid quaternions can
    # contain heading quantisation jumps that are not the actual path curve.
    for a, b, c in zip(path, path[1:], path[2:]):
        ab = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        bc = math.hypot(float(c[0]) - float(b[0]), float(c[1]) - float(b[1]))
        ac = math.hypot(float(c[0]) - float(a[0]), float(c[1]) - float(a[1]))
        if min(ab, bc, ac) < 1.0e-4:
            continue
        cross = abs((float(b[0]) - float(a[0])) * (float(c[1]) - float(a[1])) -
                    (float(b[1]) - float(a[1])) * (float(c[0]) - float(a[0])))
        if 2.0 * cross / (ab * bc * ac) > 1.0 / 0.35 + 1.0e-3:
            return "geometric_rmin_audit"
    for a, b in zip(path, path[1:]):
        segment = math.hypot(float(b[0]) - float(a[0]), float(b[1]) - float(a[1]))
        if not math.isfinite(segment):
            return "non_finite_segment"
        if segment <= 1.0e-4:
            continue
        projection = ((float(b[0]) - float(a[0])) * math.cos(float(a[2])) +
                      (float(b[1]) - float(a[1])) * math.sin(float(a[2])))
        if projection < -0.01:
            return "forward_segment_audit"
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw-entry", type=Path, required=True)
    parser.add_argument("--raw-goal-ids", type=Path, required=True)
    parser.add_argument("--meta", type=Path, required=True)
    parser.add_argument("--paths", type=Path, required=True)
    parser.add_argument("--out-entry", type=Path, required=True)
    parser.add_argument("--out-goal-ids", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()

    meta = json.loads(args.meta.read_text())
    raw_entry = args.raw_entry.read_bytes()
    if raw_entry[:4] != b"RGE1":
        raise SystemExit("raw entry must be RGE1")
    size_x, size_y, yaw_bins, mask_len = struct.unpack_from("<IIII", raw_entry, 4)
    raw_mask = raw_entry[20:]
    if len(raw_mask) != mask_len:
        raise SystemExit("raw entry mask length mismatch")
    raw_goal = args.raw_goal_ids.read_bytes()
    if raw_goal[:5] != b"RGEG1":
        raise SystemExit("raw goal ids must be RGEG1")
    (state_count,) = struct.unpack_from("<I", raw_goal, 5)
    raw_ids = raw_goal[9:]
    if len(raw_ids) != state_count:
        raise SystemExit("raw goal id count mismatch")
    specs = meta.get("gate_goal_specs")
    if not specs:
        raise SystemExit("metadata has no goal table")

    state_total = size_x * size_y * yaw_bins
    if state_total != state_count:
        raise SystemExit("raw entry and goal-id state counts disagree")
    keep = bytearray((state_total + 7) // 8)
    keep_ids = bytearray([255] * state_total)
    reasons = Counter()
    witness_count = 0
    kept_count = 0
    for line in args.paths.open(encoding="utf-8"):
        if not line.strip():
            continue
        witness_count += 1
        item = json.loads(line)
        reason = audit(item)
        if reason is not None:
            reasons[reason] += 1
            continue
        ix, iy, iyaw = int(item["ix"]), int(item["iy"]), int(item["iyaw"])
        linear = (iyaw * size_y + iy) * size_x + ix
        if linear < 0 or linear >= state_total:
            reasons["index_out_of_range"] += 1
            continue
        if not (raw_mask[linear >> 3] & (1 << (linear & 7))):
            reasons["raw_mask_bit_clear"] += 1
            continue
        goal_id = int(item["goal_id"])
        if goal_id < 0 or goal_id >= len(specs):
            reasons["goal_id_out_of_range"] += 1
            continue
        keep[linear >> 3] |= 1 << (linear & 7)
        keep_ids[linear] = goal_id
        kept_count += 1

    goal_table = b"".join(
        struct.pack("<ddd", float(spec["x_m"]), float(spec["y_m"]),
                    float(spec["yaw_deg"])) for spec in specs)
    args.out_entry.parent.mkdir(parents=True, exist_ok=True)
    args.out_entry.write_bytes(
        b"RGE2" + struct.pack(
            "<IIIIdddii", size_x, size_y, yaw_bins, len(keep),
            float(meta["x_min_m"]), float(meta["y_min_m"]),
            float(meta["xy_resolution_m"]), int(meta["yaw_min_deg"]),
            int(meta["yaw_resolution_deg"]),
        ) + keep)
    args.out_goal_ids.write_bytes(
        b"RGEG2" + struct.pack("<II", state_total, len(specs)) +
        goal_table + keep_ids)
    report = {
        "raw_smac_feasible_witnesses": witness_count,
        "runtime_audit_feasible_witnesses": kept_count,
        "filtered": witness_count - kept_count,
        "reject_reasons": dict(reasons),
        "grid": {"size_x": size_x, "size_y": size_y, "yaw_bins": yaw_bins,
                 "x_min_m": meta["x_min_m"], "y_min_m": meta["y_min_m"],
                 "xy_resolution_m": meta["xy_resolution_m"],
                 "yaw_min_deg": meta["yaw_min_deg"],
                 "yaw_step_deg": meta["yaw_resolution_deg"]},
        "handoff_goal_count": len(specs),
        "minimum_turning_radius_m": 0.35,
    }
    args.report.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
