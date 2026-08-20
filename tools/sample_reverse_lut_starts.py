#!/usr/bin/env python3
"""Extract representative, formally admitted starts from the Clean RGE2 LUT."""

from __future__ import annotations

import argparse
import json
import math
import struct
from pathlib import Path

import numpy as np

from offline_mppi_shadow_sim import deserialize_bag, footprint_samples


HEADER = struct.Struct("<4sIIIIdddii")
GOAL_HEADER = struct.Struct("<5sII")
GOAL_RECORD = struct.Struct("<ddd")


def load_goal_ids(path: Path, state_count: int):
    data = path.read_bytes()
    if len(data) < GOAL_HEADER.size:
        raise ValueError(f"goal-id LUT is shorter than its header: {path}")
    magic, encoded_states, goal_count = GOAL_HEADER.unpack_from(data)
    if magic != b"RGEG2" or encoded_states != state_count or not 0 < goal_count <= 255:
        raise ValueError(
            f"goal-id LUT header disagrees with RGE2: "
            f"magic={magic!r} states={encoded_states} goals={goal_count}")
    table_end = GOAL_HEADER.size + goal_count * GOAL_RECORD.size
    if len(data) != table_end + state_count:
        raise ValueError(f"goal-id LUT payload length mismatch: {path}")
    witnesses = []
    for i in range(goal_count):
        x, y, yaw_deg = GOAL_RECORD.unpack_from(
            data, GOAL_HEADER.size + i * GOAL_RECORD.size)
        witnesses.append({
            "goal_id": i,
            "x": x,
            "y": y,
            "yaw_deg": yaw_deg,
        })
    return data[table_end:], witnesses


def occupied(map_data, x, y, yaw):
    pts = np.asarray(footprint_samples(x, y, yaw), dtype=float)
    ix = np.floor((pts[:, 0] - map_data.origin_x) / map_data.resolution).astype(int)
    iy = np.floor((pts[:, 1] - map_data.origin_y) / map_data.resolution).astype(int)
    if np.any(ix < 0) or np.any(iy < 0) or np.any(ix >= map_data.width) or np.any(iy >= map_data.height):
        return True
    return bool(np.any(map_data.occupancy[iy, ix] >= 50))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--entry", type=Path, default=Path("src/hobot_navigation/hobot_nav/runtime/reverse_gate_entry_clean.bin"))
    ap.add_argument("--goal-ids", type=Path, default=Path("src/hobot_navigation/hobot_nav/runtime/reverse_gate_goal_id_clean.bin"))
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--count", type=int, default=24)
    ap.add_argument("--goal", nargs=3, type=float, default=[2.50, 2.30, 90.0])
    args = ap.parse_args()
    _, _, _, _, map_data, _, _ = deserialize_bag(args.bag)
    b = args.entry.read_bytes()
    magic, sx, sy, yaw_bins, mask_len, xmin, ymin, res, yaw_min, yaw_step = HEADER.unpack_from(b)
    if magic != b"RGE2":
        raise SystemExit(f"unexpected LUT magic {magic!r}")
    state_count = sx * sy * yaw_bins
    goal_ids, goal_witnesses = load_goal_ids(args.goal_ids, state_count)
    mask = b[HEADER.size:]
    all_states = []
    for iyaw in range(yaw_bins):
        for iy in range(sy):
            for ix in range(sx):
                linear = (iyaw * sy + iy) * sx + ix
                if mask[linear >> 3] & (1 << (linear & 7)):
                    x, y = xmin + ix * res, ymin + iy * res
                    yaw = float(yaw_min + iyaw * yaw_step)
                    goal_id = int(goal_ids[linear])
                    all_states.append({
                        "x": x, "y": y, "yaw_deg": yaw,
                        "ix": ix, "iy": iy, "iyaw": iyaw,
                        "linear_index": linear,
                        "reverse_goal_id": goal_id,
                        "reverse_goal_witness": (
                            goal_witnesses[goal_id] if goal_id != 255 else None),
                        "start_valid": not occupied(
                            map_data, x, y, math.radians(yaw)),
                    })
    all_states = [s for s in all_states if s["start_valid"]]
    if not all_states:
        raise SystemExit("no full-footprint valid Clean LUT states")
    # Stratify by yaw first, then choose spatially spread states in each yaw
    # layer. This is deterministic and does not privilege one recorded run
    # start.  The all_states list is still retained in the manifest for audit.
    by_yaw = {}
    for s in all_states:
        by_yaw.setdefault(s["yaw_deg"], []).append(s)
    yaw_keys = sorted(by_yaw)
    chosen = []
    for i in range(min(args.count, len(all_states))):
        yaw = yaw_keys[round(i * (len(yaw_keys) - 1) / max(1, args.count - 1))]
        layer = by_yaw[yaw]
        # Farthest-point-like deterministic choice within the layer.
        if not chosen or all(c["yaw_deg"] != yaw for c in chosen):
            pick = layer[(i * 7) % len(layer)]
        else:
            used = {(c["x"], c["y"], c["yaw_deg"]) for c in chosen}
            candidates = [c for c in layer if (c["x"], c["y"], c["yaw_deg"]) not in used] or layer
            pick = max(candidates, key=lambda c: min((c["x"] - q["x"]) ** 2 + (c["y"] - q["y"]) ** 2 for q in chosen if q["yaw_deg"] == yaw))
        if pick not in chosen:
            chosen.append(pick)
    # Fill any duplicate/layer shortfall by evenly spaced global states.
    remaining = [s for s in all_states if s not in chosen]
    while len(chosen) < min(args.count, len(all_states)):
        j = round((len(chosen) - 1) * (len(remaining) - 1) / max(1, args.count - 1))
        chosen.append(remaining.pop(min(j, len(remaining) - 1)))
    chosen.sort(key=lambda s: (s["yaw_deg"], s["y"], s["x"]))
    args.out.parent.mkdir(parents=True, exist_ok=True)
    manifest = {
        "source": str(args.entry),
        "goal_id_source": str(args.goal_ids),
        "predicate": "Clean RGE2 mask + full footprint start admission",
        "total_clean_states": len(all_states), "selected_count": len(chosen),
        "goal": args.goal, "all_admitted_states": all_states,
        "selected": [{"name": f"lut_{i:02d}", "start": [s["x"], s["y"], s["yaw_deg"]], "goal": args.goal, "lut_state": s} for i, s in enumerate(chosen)],
    }
    args.out.write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps({"total_clean_states": len(all_states), "selected_count": len(chosen), "selected": manifest["selected"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
