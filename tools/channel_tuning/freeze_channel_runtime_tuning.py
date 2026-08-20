#!/usr/bin/env python3
"""Freeze one reviewed RPP/preview/avoidance selection into runtime YAML."""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BASE = (
    ROOT / "src/hobot_navigation/hobot_nav/config/channel_tuning/"
    "channel_runtime_tuning.yaml"
)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def selected_parameters(path: Path) -> dict:
    value = json.loads(path.read_text())
    if "ranked" in value:
        if not value["ranked"]:
            raise SystemExit("sweep result has no ranked candidate")
        return dict(value["ranked"][0].get("parameters", {}))
    if "parameters" in value:
        return dict(value["parameters"])
    manifest = path.parent / "RUN_MANIFEST.json"
    if manifest.is_file():
        return dict(json.loads(manifest.read_text()).get("rpp", {}))
    return dict(value.get("rpp", {}))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--selected-summary", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--corner-speed", type=float)
    parser.add_argument("--lane-change-speed", type=float)
    parser.add_argument("--corner-preview", type=float)
    parser.add_argument("--trigger-lookahead", type=float)
    parser.add_argument("--return-margin", type=float)
    args = parser.parse_args()
    if args.out.exists():
        raise SystemExit(f"refusing to overwrite frozen tuning: {args.out}")
    value = yaml.safe_load(args.base.read_text())
    parameters = selected_parameters(args.selected_summary) if args.selected_summary else {}
    mapping = {
        "lookahead_time": "lookahead_time_sec",
        "lookahead_dist": "lookahead_dist_m",
        "min_lookahead": "min_lookahead_dist_m",
        "max_lookahead": "max_lookahead_dist_m",
    }
    for source, target in mapping.items():
        if source in parameters:
            value["rpp"][target] = float(parameters[source])
    if "corner_speed" in parameters:
        value["speed"]["corner_ceiling_mps"] = float(parameters["corner_speed"])
    if "corner_preview_distance" in parameters:
        value["corner_preview"]["distance_m"] = float(
            parameters["corner_preview_distance"])
    overrides = (
        (args.corner_speed, value["speed"], "corner_ceiling_mps"),
        (args.lane_change_speed, value["speed"], "lane_change_ceiling_mps"),
        (args.corner_preview, value["corner_preview"], "distance_m"),
        (args.trigger_lookahead, value["avoidance"], "cone_trigger_lookahead_m"),
        (args.return_margin, value["avoidance"], "cone_passed_return_margin_m"),
    )
    for override, section, key in overrides:
        if override is not None:
            section[key] = float(override)
    if value.get("schema_version") != 2 or value["contracts"]["hard_rmin_m"] != 0.35:
        raise SystemExit("invalid Channel runtime tuning contract")
    if value["speed"]["straight_mode"] != "selected_profile":
        raise SystemExit("straight_mode must remain selected_profile")
    value["freeze_provenance"] = {
        "frozen_utc": dt.datetime.now(dt.timezone.utc).isoformat(),
        "base_sha256": sha256(args.base),
        "selected_summary": None if args.selected_summary is None else {
            "path": str(args.selected_summary),
            "sha256": sha256(args.selected_summary),
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(yaml.safe_dump(value, sort_keys=False))
    print(json.dumps({"out": str(args.out), "sha256": sha256(args.out)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
