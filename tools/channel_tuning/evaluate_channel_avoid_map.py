#!/usr/bin/env python3
"""Independent structural/admission audit for a Channel avoid-map candidate."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

CHANNEL_TUNING = Path(__file__).resolve().parent
if str(CHANNEL_TUNING) not in sys.path:
    sys.path.insert(0, str(CHANNEL_TUNING))

from channel_asset_common import sha256_file  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("candidate", type=Path)
    parser.add_argument("--tube-dir", type=Path, required=True)
    parser.add_argument("--connector-dir", type=Path, required=True)
    parser.add_argument("--map", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads((args.candidate / "manifest.json").read_text())
    failures = []
    if manifest.get("candidate_only") is not True:
        failures.append("candidate_only marker missing")
    if manifest["static_map"]["sha256"] != sha256_file(args.map):
        failures.append("static map hash mismatch")
    actual_tubes = {p.name: sha256_file(p) for p in sorted(args.tube_dir.glob("tube_*.csv"))}
    actual_connectors = {p.name: sha256_file(p) for p in sorted(args.connector_dir.glob("connectors/*.csv"))}
    if manifest.get("tube_hashes") != actual_tubes:
        failures.append("Tube hash set mismatch")
    if manifest.get("connector_hashes") != actual_connectors:
        failures.append("connector hash set mismatch")
    width = int(manifest["grid"]["width"])
    height = int(manifest["grid"]["height"])
    direction_results = {}
    for direction in ("cw", "ccw"):
        path = args.candidate / f"channel_cone_avoid_lane_1cm_{direction}.bin"
        data = path.read_bytes()
        values = set(data)
        report = manifest["directions"][direction]
        counts = {value: data.count(value) for value in (0, 1, 2)}
        ok = (len(data) == width * height and values <= {0, 1, 2} and
              sha256_file(path) == report["sha256"] and
              counts[1] == report["cone_inner_side_cells"] and
              counts[2] == report["cone_outer_side_cells"] and
              report["classified_legal_fraction"] >= 0.999999 and
              counts[1] > 0 and counts[2] > 0)
        if not ok:
            failures.append(f"{direction} binary/hash/count contract failed")
        direction_results[direction] = {"bytes": len(data), "counts": counts, "pass": ok}
    result = {"schema_version": 2, "pass": not failures,
              "failures": failures, "directions": direction_results}
    output = args.candidate / "independent_evaluation.json"
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
