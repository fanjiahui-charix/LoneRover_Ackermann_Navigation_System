#!/usr/bin/env python3
"""Build the runtime clean Reverse->Gate LUT from a strict RGE2/RGEG2 pair."""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from collections import Counter
from pathlib import Path


ENTRY_HEADER = struct.Struct("<4sIIIIdddii")
GOAL_HEADER = struct.Struct("<5sII")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--strict-entry", type=Path, required=True)
    parser.add_argument("--strict-goal-ids", type=Path, required=True)
    parser.add_argument("--out-entry", type=Path, required=True)
    parser.add_argument("--out-goal-ids", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--x-max", type=float, default=2.60)
    parser.add_argument("--y-max", type=float, default=1.40)
    parser.add_argument("--expected-clean-states", type=int, default=890)
    args = parser.parse_args()

    entry = args.strict_entry.read_bytes()
    if len(entry) < ENTRY_HEADER.size:
        raise SystemExit("strict entry is shorter than the RGE2 header")
    magic, size_x, size_y, yaw_bins, mask_len, x_min, y_min, resolution, yaw_min, yaw_step = (
        ENTRY_HEADER.unpack_from(entry)
    )
    if magic != b"RGE2":
        raise SystemExit("strict entry is not RGE2")
    state_count = size_x * size_y * yaw_bins
    expected_mask_len = (state_count + 7) // 8
    strict_mask = entry[ENTRY_HEADER.size:]
    if mask_len != expected_mask_len or len(strict_mask) != mask_len:
        raise SystemExit("strict RGE2 dimensions/mask length disagree")

    goals = args.strict_goal_ids.read_bytes()
    if len(goals) < GOAL_HEADER.size:
        raise SystemExit("strict goal-id file is shorter than the RGEG2 header")
    goal_magic, goal_state_count, goal_count = GOAL_HEADER.unpack_from(goals)
    if goal_magic != b"RGEG2" or goal_state_count != state_count or not 0 < goal_count <= 255:
        raise SystemExit("strict RGEG2 header disagrees with RGE2")
    table_end = GOAL_HEADER.size + goal_count * 24
    goal_table = goals[GOAL_HEADER.size:table_end]
    strict_ids = goals[table_end:]
    if len(strict_ids) != state_count:
        raise SystemExit("strict RGEG2 state-id payload length mismatch")

    def feasible(ix: int, iy: int, iyaw: int) -> bool:
        if not (0 <= ix < size_x and 0 <= iy < size_y and 0 <= iyaw < yaw_bins):
            return False
        linear = (iyaw * size_y + iy) * size_x + ix
        return bool(strict_mask[linear >> 3] & (1 << (linear & 7))) and (
            strict_ids[linear] != 255 and strict_ids[linear] < goal_count
        )

    clean_mask = bytearray(mask_len)
    clean_ids = bytearray([255] * state_count)
    strict_count = 0
    interior_count = 0
    clean_count = 0
    clean_goal_ids: Counter[int] = Counter()
    for iyaw in range(yaw_bins):
        for iy in range(size_y):
            for ix in range(size_x):
                if not feasible(ix, iy, iyaw):
                    continue
                strict_count += 1
                interior = (
                    feasible(ix - 1, iy, iyaw)
                    and feasible(ix + 1, iy, iyaw)
                    and feasible(ix, iy - 1, iyaw)
                    and feasible(ix, iy + 1, iyaw)
                )
                if not interior:
                    continue
                interior_count += 1
                x = x_min + ix * resolution
                y = y_min + iy * resolution
                if x > args.x_max + 1.0e-9 or y > args.y_max + 1.0e-9:
                    continue
                linear = (iyaw * size_y + iy) * size_x + ix
                clean_mask[linear >> 3] |= 1 << (linear & 7)
                clean_ids[linear] = strict_ids[linear]
                clean_goal_ids[int(strict_ids[linear])] += 1
                clean_count += 1

    if clean_count != args.expected_clean_states:
        raise SystemExit(
            f"clean state count {clean_count} != expected {args.expected_clean_states}"
        )

    args.out_entry.parent.mkdir(parents=True, exist_ok=True)
    args.out_entry.write_bytes(entry[:ENTRY_HEADER.size] + clean_mask)
    args.out_goal_ids.write_bytes(goals[:GOAL_HEADER.size] + goal_table + clean_ids)
    report = {
        "version": "reverse-gate-clean-v1-20260808",
        "source": {
            "entry": str(args.strict_entry),
            "entry_sha256": sha256(args.strict_entry),
            "goal_ids": str(args.strict_goal_ids),
            "goal_ids_sha256": sha256(args.strict_goal_ids),
        },
        "predicate": f"strict && interior_xy && x<={args.x_max:.2f} && y<={args.y_max:.2f}",
        "nearest_cell_only": True,
        "grid": {
            "size_x": size_x,
            "size_y": size_y,
            "yaw_bins": yaw_bins,
            "x_min_m": x_min,
            "y_min_m": y_min,
            "xy_resolution_m": resolution,
            "yaw_min_deg": yaw_min,
            "yaw_step_deg": yaw_step,
        },
        "strict_states": strict_count,
        "interior_states": interior_count,
        "clean_states": clean_count,
        "clean_goal_id_counts": dict(sorted(clean_goal_ids.items())),
        "output": {
            "entry": str(args.out_entry),
            "entry_sha256": sha256(args.out_entry),
            "goal_ids": str(args.out_goal_ids),
            "goal_ids_sha256": sha256(args.out_goal_ids),
        },
    }
    args.report.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
