#!/usr/bin/env python3
"""Materialize native-shadow scenarios from an immutable LUT manifest."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, required=True)
    ap.add_argument("--remote-dir", type=str, required=True)
    ap.add_argument("--selection", choices=("selected", "all"), default="selected")
    ap.add_argument("--spec", type=Path, required=True)
    args = ap.parse_args()

    source = json.loads(args.manifest.read_text(encoding="utf-8"))
    goal = source["goal"]
    if args.selection == "selected":
        entries = source["selected"]
    else:
        entries = [
            {
                "name": f"lut_{state['linear_index']:04d}",
                "start": [state["x"], state["y"], state["yaw_deg"]],
                "goal": goal,
                "lut_state": state,
            }
            for state in source["all_admitted_states"]
        ]

    args.out_dir.mkdir(parents=True, exist_ok=True)
    materialized = []
    spec_lines = []
    for entry in entries:
        name = str(entry["name"]).replace("lut_", "lut")
        scenario_name = f"scenario_{name}.json"
        scenario = {"start": entry["start"], "goal": entry.get("goal", goal)}
        (args.out_dir / scenario_name).write_text(
            json.dumps(scenario, separators=(",", ":")) + "\n", encoding="utf-8")
        remote_path = f"{args.remote_dir.rstrip('/')}/{scenario_name}"
        spec_lines.append(f"{name}\t{remote_path}\n")
        materialized.append({
            "name": name,
            "scenario_file": scenario_name,
            "remote_scenario": remote_path,
            "start": scenario["start"],
            "goal": scenario["goal"],
            "lut_state": entry["lut_state"],
        })

    args.spec.parent.mkdir(parents=True, exist_ok=True)
    args.spec.write_text("".join(spec_lines), encoding="utf-8")
    (args.out_dir / "materialized_manifest.json").write_text(
        json.dumps({
            "source_manifest": str(args.manifest),
            "selection": args.selection,
            "count": len(materialized),
            "states": materialized,
        }, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"count": len(materialized), "spec": str(args.spec)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
