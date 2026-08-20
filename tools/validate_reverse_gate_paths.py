#!/usr/bin/env python3
"""Validate saved Smac Reverse->Tube handoff witness paths without motion."""

import argparse
import json
import math
from pathlib import Path


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('paths', type=Path)
    args = parser.parse_args()
    total = valid = 0
    reasons = {}
    max_curvature = 0.0
    lengths = []
    for line in args.paths.open(encoding='utf-8'):
        if not line.strip():
            continue
        total += 1
        item = json.loads(line)
        path = item.get('path', [])
        reason = None
        if len(path) < 2:
            reason = 'path_too_short'
        elif not all(
                len(p) == 3 and all(math.isfinite(float(v)) for v in p)
                for p in path):
            reason = 'non_finite_pose'
        else:
            length = 0.0
            for a, b in zip(path, path[1:]):
                dx, dy = b[0] - a[0], b[1] - a[1]
                segment = math.hypot(dx, dy)
                if segment < 1.0e-5:
                    continue
                projection = dx * math.cos(a[2]) + dy * math.sin(a[2])
                if projection < -0.01:
                    reason = 'reverse_segment'
                    break
                length += segment
            if reason is None:
                for a, b, c in zip(path, path[1:], path[2:]):
                    ab = math.hypot(b[0] - a[0], b[1] - a[1])
                    bc = math.hypot(c[0] - b[0], c[1] - b[1])
                    ac = math.hypot(c[0] - a[0], c[1] - a[1])
                    if min(ab, bc, ac) < 1.0e-5:
                        continue
                    cross = abs((b[0] - a[0]) * (c[1] - a[1]) -
                                (b[1] - a[1]) * (c[0] - a[0]))
                    curvature = 2.0 * cross / (ab * bc * ac)
                    max_curvature = max(max_curvature, curvature)
                    if curvature > 1.0 / 0.35 + 1.0e-3:
                        reason = 'minimum_radius_violation'
                        break
            lengths.append(length)
        if reason is None:
            valid += 1
        else:
            reasons[reason] = reasons.get(reason, 0) + 1
    summary = {
        'paths': total,
        'valid': valid,
        'invalid': total - valid,
        'max_curvature_per_m': max_curvature,
        'max_curvature_radius_m': (1.0 / max_curvature
                                   if max_curvature > 1.0e-9 else None),
        'path_length_min_m': min(lengths) if lengths else None,
        'path_length_max_m': max(lengths) if lengths else None,
        'reject_reasons': reasons,
    }
    print(json.dumps(summary, indent=2))
    raise SystemExit(0 if valid == total else 2)


if __name__ == '__main__':
    main()
