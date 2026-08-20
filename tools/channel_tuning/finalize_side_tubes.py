#!/usr/bin/env python3
"""Turn measured RPP/localization envelopes into one Tube safety budget."""

from __future__ import annotations

import argparse
import datetime as dt
import json
import subprocess
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[2]
PACKAGE = WORKSPACE / "src/hobot_navigation/hobot_nav"
GENERATOR = WORKSPACE / "tools/generate_channel_tubes_v2.py"
EVALUATOR = Path(__file__).with_name("evaluate_channel_tubes.py")
VISUALIZER = Path(__file__).with_name("visualize_channel_assets.py")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--tracking-p95", type=float, required=True)
    parser.add_argument("--tracking-max", type=float, required=True)
    parser.add_argument("--localization-margin", type=float, required=True)
    parser.add_argument("--residual-margin", type=float, required=True)
    parser.add_argument("--center-dir", type=Path, default=PACKAGE / "config/channel_tubes")
    parser.add_argument("--map", type=Path, default=PACKAGE / "maps/rdk_2026_hospital_static_1cm.pgm")
    parser.add_argument("--out-root", type=Path)
    args = parser.parse_args()
    for name in ("tracking_p95", "tracking_max", "localization_margin", "residual_margin"):
        if getattr(args, name) < 0.0:
            raise SystemExit(f"{name} must be nonnegative")
    tracking_envelope = max(args.tracking_max, 1.25 * args.tracking_p95)
    required_margin = tracking_envelope + args.localization_margin + args.residual_margin
    if required_margin > 0.08:
        raise SystemExit(
            f"unique safety budget {required_margin:.3f} m exceeds available candidate search; "
            "do not silently duplicate/shrink margins")
    output = (args.out_root or WORKSPACE / "logs/channel_tuning" /
              f"{dt.datetime.now():%Y%m%d_%H%M%S}_finalize_side").resolve()
    if output.exists():
        raise SystemExit(f"refusing to overwrite finalizer output: {output}")
    candidate_root = output / "candidate"
    command = [
        sys.executable, str(GENERATOR), "--center-dir", str(args.center_dir),
        "--map", str(args.map), "--out-root", str(candidate_root),
        "--margins", str(required_margin),
    ]
    completed = subprocess.run(command)
    if completed.returncode != 0:
        return completed.returncode
    candidate_dir = next(candidate_root.glob("margin_*mm"))
    evaluation = candidate_dir / "independent_evaluation.json"
    completed = subprocess.run([
        sys.executable, str(EVALUATOR), "--tube-dir", str(candidate_dir / "tubes"),
        "--map", str(args.map), "--required-margin", str(required_margin),
        "--out", str(evaluation),
    ])
    subprocess.run([
        sys.executable, str(VISUALIZER), "--tube-dir", str(candidate_dir / "tubes"),
        "--map", str(args.map), "--out-dir", str(candidate_dir / "visualization"),
    ], check=True)
    budget = {
        "schema_version": 2,
        "measurement_status": "measured_from_center_native_rpp",
        "measured": True,
        "formula": "max(tracking_max,1.25*tracking_p95)+localization_margin+residual_margin",
        "tracking_p95_m": args.tracking_p95,
        "tracking_max_m": args.tracking_max,
        "tracking_envelope_m": tracking_envelope,
        "localization_margin_m": args.localization_margin,
        "residual_margin_m": args.residual_margin,
        "required_dynamic_margin_m": required_margin,
        "double_counted_margin": False,
        "side_rpp_acceptance": {
            "fixed_cte_gate_used": False,
            "fixed_minimum_clearance_gate_used": False,
            "rule": (
                "max(measured_cte_max,1.25*measured_cte_p95)+"
                "localization_margin+residual_margin <= nominal_tube_static_clearance"
            ),
            "zero_collision_always_required": True,
        },
        "candidate_dir": str(candidate_dir),
        "independent_evaluation_pass": completed.returncode == 0,
        "runtime_promotion_allowed": False,
        "remaining_gate": "six native RPP Tube stress runs",
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "SAFETY_BUDGET.json").write_text(json.dumps(budget, indent=2) + "\n")
    print(json.dumps(budget, indent=2))
    return completed.returncode


if __name__ == "__main__":
    raise SystemExit(main())
