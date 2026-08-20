#!/usr/bin/env python3
"""Convert the first vehicle-side LUT writer to the self-describing RGE2 format.

The long-running generator was intentionally not restarted when the runtime
loader was tightened.  Once its RGE1/RGEG1 files finish, this converter uses
the generator's JSON metadata (including the exact sorted handoff goal table)
to produce the self-describing RGE2/RGEG2 artifacts consumed by a navigation
runtime.
"""

from __future__ import annotations

import argparse
import json
import struct
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--entry", type=Path, required=True)
    parser.add_argument("--goal-ids", type=Path, required=True)
    parser.add_argument("--meta", type=Path, required=True)
    parser.add_argument("--out-entry", type=Path, required=True)
    parser.add_argument("--out-goal-ids", type=Path, required=True)
    args = parser.parse_args()

    meta = json.loads(args.meta.read_text())
    entry = args.entry.read_bytes()
    if entry[:4] != b"RGE1":
        raise SystemExit("entry input is not RGE1")
    size_x, size_y, yaw_bins, mask_len = struct.unpack_from("<IIII", entry, 4)
    mask = entry[20:]
    if len(mask) != mask_len:
        raise SystemExit("entry mask length mismatch")

    expected = (int(meta["size_x"]), int(meta["size_y"]), int(meta["yaw_bins"]),
                float(meta["xy_resolution_m"]))
    actual = (size_x, size_y, yaw_bins, float(meta["xy_resolution_m"]))
    if actual[:3] != expected[:3] or abs(actual[3] - expected[3]) > 1.0e-9:
        raise SystemExit(f"entry dimensions disagree with metadata: {actual} != {expected}")

    args.out_entry.parent.mkdir(parents=True, exist_ok=True)
    args.out_entry.write_bytes(
        b"RGE2" + struct.pack(
            "<IIIIdddii", size_x, size_y, yaw_bins, mask_len,
            float(meta["x_min_m"]), float(meta["y_min_m"]),
            float(meta["xy_resolution_m"]), int(meta["yaw_min_deg"]),
            int(meta["yaw_resolution_deg"]),
        ) + mask
    )

    old_goal_ids = args.goal_ids.read_bytes()
    if old_goal_ids[:5] != b"RGEG1":
        raise SystemExit("goal-id input is not RGEG1")
    (state_count,) = struct.unpack_from("<I", old_goal_ids, 5)
    ids = old_goal_ids[9:]
    if len(ids) != state_count:
        raise SystemExit("goal-id state count mismatch")
    specs = meta.get("gate_goal_specs")
    if not specs:
        raise SystemExit("metadata has no gate_goal_specs")
    goal_table = b"".join(
        struct.pack("<ddd", float(spec["x_m"]), float(spec["y_m"]),
                    float(spec["yaw_deg"]))
        for spec in specs
    )
    args.out_goal_ids.write_bytes(
        b"RGEG2" + struct.pack("<II", state_count, len(specs)) + goal_table + ids
    )
    print(f"wrote {args.out_entry} and {args.out_goal_ids}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
