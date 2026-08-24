#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Iterable

import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore


@dataclass
class Segment:
    pts: list[np.ndarray]


def polar_to_point(r: float, angle: float) -> np.ndarray:
    return np.array([r * math.cos(angle), r * math.sin(angle)], dtype=float)


def point_line_distance(p: np.ndarray, normal: np.ndarray, d: float) -> float:
    return abs(float(normal.dot(p) - d))


def fit_line_pca(pts: list[np.ndarray]):
    if len(pts) < 2:
        return None
    arr = np.asarray(pts, dtype=float)
    centroid = arr.mean(axis=0)
    centered = arr - centroid
    cov = centered.T @ centered
    evals, evecs = np.linalg.eigh(cov)
    normal = evecs[:, 0]
    nrm = np.linalg.norm(normal)
    if nrm < 1.0e-9:
        return None
    normal = normal / nrm
    if float(normal.dot(-centroid)) < 0.0:
        normal = -normal
    return {
        'normal': normal,
        'centroid': centroid,
        'd': float(normal.dot(centroid)),
        'support_points': [p.copy() for p in pts],
    }


def principal_span(pts: list[np.ndarray]):
    if not pts:
        return 0.0, 0.0
    arr = np.asarray(pts, dtype=float)
    centroid = arr.mean(axis=0)
    centered = arr - centroid
    cov = centered.T @ centered
    evals, _ = np.linalg.eigh(cov)
    minor = max(0.0, float(evals[0]))
    major = max(0.0, float(evals[1]))
    return 2.0 * math.sqrt(major), minor


def segment_scan(scan, min_range: float, max_range: float, seg_break_dist: float):
    segments: list[Segment] = []
    current: list[np.ndarray] = []
    have_prev_valid = False
    prev_idx = 0
    prev_point = np.zeros(2, dtype=float)

    for i, r in enumerate(scan.ranges):
        valid = math.isfinite(r) and min_range <= r <= max_range
        if not valid:
            if current:
                segments.append(Segment(current))
                current = []
            have_prev_valid = False
            continue

        angle = float(scan.angle_min) + float(i) * float(scan.angle_increment)
        p = polar_to_point(float(r), angle)

        break_segment = False
        if have_prev_valid:
            if i != prev_idx + 1:
                break_segment = True
            elif np.linalg.norm(p - prev_point) > seg_break_dist:
                break_segment = True

        if break_segment and current:
            segments.append(Segment(current))
            current = []

        current.append(p)
        have_prev_valid = True
        prev_idx = i
        prev_point = p

    if current:
        segments.append(Segment(current))

    span = float(scan.angle_max - scan.angle_min)
    if len(segments) > 1 and abs(span - 2.0 * math.pi) < 0.2:
        first = segments[0].pts[0]
        last = segments[-1].pts[-1]
        if np.linalg.norm(first - last) <= seg_break_dist:
            merged = Segment(segments[-1].pts + segments[0].pts)
            segments = [merged] + segments[1:-1]
    return segments


def fit_circle_fixed_r(pts: list[np.ndarray], radius: float, max_rms: float):
    if len(pts) < 3 or radius <= 0.0:
        return None
    arr = np.asarray(pts, dtype=float)
    centroid = arr.mean(axis=0)
    direction = centroid.copy()
    nrm = np.linalg.norm(direction)
    if nrm < 1.0e-9:
        direction = np.array([1.0, 0.0], dtype=float)
    else:
        direction /= nrm
    center = centroid + radius * direction

    for _ in range(5):
        H = np.zeros((2, 2), dtype=float)
        b = np.zeros(2, dtype=float)
        for p in arr:
            diff = center - p
            dist = max(np.linalg.norm(diff), 1.0e-9)
            residual = dist - radius
            J = diff / dist
            H += np.outer(J, J)
            b += J * residual
        H += np.eye(2) * 1.0e-9
        try:
            delta = -np.linalg.solve(H, b)
        except np.linalg.LinAlgError:
            return None
        if not np.isfinite(delta).all():
            return None
        center += delta
        if np.linalg.norm(delta) < 1.0e-5:
            break

    residuals = [np.linalg.norm(p - center) - radius for p in arr]
    rms = math.sqrt(sum(r * r for r in residuals) / len(arr))
    return {
        'center': center,
        'rms_error': rms,
        'valid': math.isfinite(rms) and rms <= max_rms,
    }


def extract_fence_lines(fence_segments: list[Segment], merge_angle_tol: float, merge_dist_tol: float):
    lines = []
    for seg in fence_segments:
        line = fit_line_pca(seg.pts)
        if line is not None:
            lines.append(line)

    merged = True
    while merged:
        merged = False
        i = 0
        while i < len(lines) and not merged:
            j = i + 1
            while j < len(lines):
                dot = float(lines[i]['normal'].dot(lines[j]['normal']))
                if dot > 0.0:
                    sin_angle = abs(
                        float(lines[i]['normal'][0] * lines[j]['normal'][1] -
                              lines[i]['normal'][1] * lines[j]['normal'][0])
                    )
                    if sin_angle < merge_angle_tol:
                        dist_ij = point_line_distance(lines[i]['centroid'], lines[j]['normal'], lines[j]['d'])
                        dist_ji = point_line_distance(lines[j]['centroid'], lines[i]['normal'], lines[i]['d'])
                        if max(dist_ij, dist_ji) < merge_dist_tol:
                            pts = lines[i]['support_points'] + lines[j]['support_points']
                            merged_line = fit_line_pca(pts)
                            if merged_line is not None:
                                lines[i] = merged_line
                                del lines[j]
                                merged = True
                                break
                j += 1
            i += 1
    return lines


def extract_landmarks(scan, args):
    result = {
        'cone_centers': [],
        'fences': [],
        'segments': 0,
        'cone_candidates': 0,
        'fence_candidates': 0,
        'small_segments': [],
        'fence_segments': [],
    }
    fence_segments = []
    segments = segment_scan(scan, args.min_range, args.max_range, args.seg_break_dist)
    result['segments'] = len(segments)

    for seg in segments:
        point_count = len(seg.pts)
        span, minor_var = principal_span(seg.pts)
        major_var = max(1.0e-9, 0.25 * span * span)
        aspect_ratio = major_var / max(minor_var, 1.0e-9)

        if args.cone_min_pts <= point_count <= args.cone_max_pts and span <= args.cone_max_span:
            result['cone_candidates'] += 1
            fit = fit_circle_fixed_r(seg.pts, args.cone_radius, args.cone_fit_max_rms)
            rms = None if fit is None else fit['rms_error']
            result['small_segments'].append({
                'point_count': point_count,
                'span': span,
                'rms': rms,
                'accepted': bool(fit and fit['valid']),
            })
            if fit and fit['valid']:
                result['cone_centers'].append(fit['center'])
                continue

        if point_count >= args.fence_min_pts and aspect_ratio >= args.fence_aspect_ratio:
            result['fence_candidates'] += 1
            result['fence_segments'].append({
                'point_count': point_count,
                'span': span,
                'aspect_ratio': aspect_ratio,
            })
            fence_segments.append(seg)

    result['fences'] = extract_fence_lines(
        fence_segments, args.fence_merge_angle_tol, args.fence_merge_dist_tol)
    return result


def quantiles(values: list[float], qs=(0.1, 0.5, 0.9, 0.99)):
    if not values:
        return {}
    arr = np.asarray(values, dtype=float)
    return {str(q): float(np.quantile(arr, q)) for q in qs}


def summarize_time_bins(rows: list[dict], bin_sec: float = 5.0):
    if not rows:
        return []
    bins = {}
    for row in rows:
        idx = int(row['rel_time'] // bin_sec)
        bins.setdefault(idx, {'count': 0, 'cones': [], 'fences': [], 'segments': []})
        entry = bins[idx]
        entry['count'] += 1
        entry['cones'].append(row['cone_count'])
        entry['fences'].append(row['fence_count'])
        entry['segments'].append(row['segment_count'])
    out = []
    for idx in sorted(bins):
        entry = bins[idx]
        out.append({
            'bin_start_sec': idx * bin_sec,
            'bin_end_sec': (idx + 1) * bin_sec,
            'frames': entry['count'],
            'cone_median': float(median(entry['cones'])),
            'fence_median': float(median(entry['fences'])),
            'segment_median': float(median(entry['segments'])),
        })
    return out


def load_scan_rows(bag_path: Path, args):
    store = get_typestore(Stores.ROS2_HUMBLE)
    rows = []
    with AnyReader([bag_path], default_typestore=store) as reader:
        first_scan_ts = None
        last_scan_ts = None
        for conn, ts, raw in reader.messages():
            if conn.topic != '/scan':
                continue
            if first_scan_ts is None:
                first_scan_ts = ts
            last_scan_ts = ts
            msg = reader.deserialize(raw, conn.msgtype)
            rel_time = (ts - first_scan_ts) / 1e9
            rows.append((ts, rel_time, msg))
    if not rows:
        return []
    total_duration = rows[-1][1]
    keep = []
    for ts, rel_time, msg in rows:
        if rel_time < args.trim_start_sec:
            continue
        if rel_time > total_duration - args.trim_end_sec:
            continue
        keep.append((ts, rel_time, msg))
    return keep


def inspect_topics_sqlite(db_path: Path):
    con = sqlite3.connect(str(db_path))
    try:
        cur = con.cursor()
        topics = list(cur.execute('select name, type from topics order by id'))
    finally:
        con.close()
    return topics


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--bag', type=Path, default=Path('/root/2025smartcar/cc_ws/ros_workspace/2026-06-27_18-27-41_0.db3'))
    ap.add_argument('--output', type=Path, default=Path('/root/2025smartcar/cc_ws/ros_workspace/tools/exports/lidar_landmark_stats.json'))
    ap.add_argument('--trim-start-sec', type=float, default=5.0)
    ap.add_argument('--trim-end-sec', type=float, default=5.0)
    ap.add_argument('--min-range', type=float, default=0.05)
    ap.add_argument('--max-range', type=float, default=5.5)
    ap.add_argument('--seg-break-dist', type=float, default=0.20)
    ap.add_argument('--cone-radius', type=float, default=0.10)
    ap.add_argument('--cone-min-pts', type=int, default=3)
    ap.add_argument('--cone-max-pts', type=int, default=15)
    ap.add_argument('--cone-max-span', type=float, default=0.25)
    ap.add_argument('--cone-fit-max-rms', type=float, default=0.015)
    ap.add_argument('--fence-min-pts', type=int, default=15)
    ap.add_argument('--fence-aspect-ratio', type=float, default=4.0)
    ap.add_argument('--fence-merge-angle-tol', type=float, default=0.05)
    ap.add_argument('--fence-merge-dist-tol', type=float, default=0.05)
    args = ap.parse_args()

    args.output.parent.mkdir(parents=True, exist_ok=True)

    topics = inspect_topics_sqlite(args.bag)
    rows = load_scan_rows(args.bag, args)

    per_scan = []
    cone_point_counts = []
    cone_spans = []
    cone_rms = []
    fence_point_counts = []
    fence_spans = []
    fence_aspects = []

    for _, rel_time, scan in rows:
        extracted = extract_landmarks(scan, args)
        for item in extracted['small_segments']:
            cone_point_counts.append(item['point_count'])
            cone_spans.append(item['span'])
            if item['rms'] is not None:
                cone_rms.append(item['rms'])
        for item in extracted['fence_segments']:
            fence_point_counts.append(item['point_count'])
            fence_spans.append(item['span'])
            fence_aspects.append(item['aspect_ratio'])

        per_scan.append({
            'rel_time': rel_time,
            'segment_count': extracted['segments'],
            'cone_candidate_count': extracted['cone_candidates'],
            'cone_count': len(extracted['cone_centers']),
            'fence_candidate_count': extracted['fence_candidates'],
            'fence_count': len(extracted['fences']),
            'fence_support_points': int(sum(len(line['support_points']) for line in extracted['fences'])),
        })

    summary = {
        'bag': str(args.bag),
        'topics': [{'name': name, 'type': typ} for name, typ in topics],
        'trim': {
            'start_sec': args.trim_start_sec,
            'end_sec': args.trim_end_sec,
        },
        'retained_scan_frames': len(per_scan),
        'time_bins_5s': summarize_time_bins(per_scan, 5.0),
        'cone_point_count_quantiles': quantiles(cone_point_counts, (0.1, 0.5, 0.9)),
        'cone_span_quantiles': quantiles(cone_spans, (0.1, 0.5, 0.9)),
        'cone_rms_quantiles': quantiles(cone_rms, (0.1, 0.5, 0.9, 0.99)),
        'fence_point_count_quantiles': quantiles(fence_point_counts, (0.1, 0.5, 0.9)),
        'fence_span_quantiles': quantiles(fence_spans, (0.1, 0.5, 0.9)),
        'fence_aspect_quantiles': quantiles(fence_aspects, (0.1, 0.5, 0.9)),
        'per_scan_preview': per_scan[:10],
    }

    args.output.write_text(json.dumps(summary, indent=2), encoding='utf-8')
    print(args.output)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
