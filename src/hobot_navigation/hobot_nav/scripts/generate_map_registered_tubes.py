#!/usr/bin/env python3
"""Generate map-registered Tube geometry for offline inspection.

The centerline is registered to the checked-in yellow occupancy geometry.
Inner/outer lanes are true normal offsets of the same rounded-rectangle
centerline. This generator proves nominal map, footprint, and curvature
validity; it still requires independent offline and real-vehicle validation.
"""

import csv
import math
from pathlib import Path

from PIL import Image


STEP_M = 0.015
MAP_RESOLUTION_M = 0.01
MIN_TURN_RADIUS_M = 0.35
LANES = {"inner": 0.03, "center": 0.0, "outer": -0.03}


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def append_line(path, x, y, yaw):
    sx, sy, _ = path[-1][:3]
    length = math.hypot(x - sx, y - sy)
    count = max(1, math.ceil(length / STEP_M))
    for index in range(1, count + 1):
        u = index / count
        path.append((sx + u * (x - sx), sy + u * (y - sy), yaw, ""))


def append_arc(path, cx, cy, radius, theta0, theta1):
    delta = theta1 - theta0
    count = max(1, math.ceil(abs(delta) * radius / STEP_M))
    sign = 1.0 if delta > 0.0 else -1.0
    for index in range(1, count + 1):
        theta = theta0 + delta * index / count
        yaw = normalize(theta + sign * math.pi / 2.0)
        path.append((cx + radius * math.cos(theta),
                     cy + radius * math.sin(theta), yaw, ""))


def mark(path, event):
    x, y, yaw, _ = path[-1]
    path[-1] = (x, y, yaw, event)


def build(direction, lane, inward_offset):
    left = 0.745 + inward_offset
    right = 4.255 - inward_offset
    bottom = 3.18 + inward_offset
    top = 4.34 - inward_offset
    corner = 0.40 - inward_offset
    branch = MIN_TURN_RADIUS_M
    path = [(2.50, 2.30, math.pi / 2.0, "GATE_BEGIN")]
    append_line(path, 2.50, 2.50, math.pi / 2.0)
    mark(path, "GATE")
    append_line(path, 2.50, bottom - branch, math.pi / 2.0)
    mark(path, "TURN_ENTRY")

    if direction == "CW":
        append_arc(path, 2.50 - branch, bottom - branch, branch, 0.0, math.pi / 2.0)
        mark(path, "SPLIT|C_RING_ENTRY")
        append_line(path, left + corner, bottom, math.pi)
        append_arc(path, left + corner, bottom + corner, corner,
                   -math.pi / 2.0, -math.pi)
        mark(path, "SW")
        append_line(path, left, top - corner, math.pi / 2.0)
        append_arc(path, left + corner, top - corner, corner,
                   math.pi, math.pi / 2.0)
        mark(path, "NW")
        append_line(path, right - corner, top, 0.0)
        append_arc(path, right - corner, top - corner, corner,
                   math.pi / 2.0, 0.0)
        mark(path, "NE")
        append_line(path, right, bottom + corner, -math.pi / 2.0)
        append_arc(path, right - corner, bottom + corner, corner,
                   0.0, -math.pi / 2.0)
        mark(path, "SE")
        append_line(path, 2.50 + branch, bottom, math.pi)
        mark(path, "C_RING_COMPLETE|RETURN_SPLIT")
        append_arc(path, 2.50 + branch, bottom - branch, branch,
                   math.pi / 2.0, math.pi)
    else:
        append_arc(path, 2.50 + branch, bottom - branch, branch,
                   math.pi, math.pi / 2.0)
        mark(path, "SPLIT|C_RING_ENTRY")
        append_line(path, right - corner, bottom, 0.0)
        append_arc(path, right - corner, bottom + corner, corner,
                   -math.pi / 2.0, 0.0)
        mark(path, "SE")
        append_line(path, right, top - corner, math.pi / 2.0)
        append_arc(path, right - corner, top - corner, corner,
                   0.0, math.pi / 2.0)
        mark(path, "NE")
        append_line(path, left + corner, top, math.pi)
        append_arc(path, left + corner, top - corner, corner,
                   math.pi / 2.0, math.pi)
        mark(path, "NW")
        append_line(path, left, bottom + corner, -math.pi / 2.0)
        append_arc(path, left + corner, bottom + corner, corner,
                   math.pi, 3.0 * math.pi / 2.0)
        mark(path, "SW")
        append_line(path, 2.50 - branch, bottom, 0.0)
        mark(path, "C_RING_COMPLETE|RETURN_SPLIT")
        append_arc(path, 2.50 - branch, bottom - branch, branch,
                   math.pi / 2.0, 0.0)

    append_line(path, 2.50, 2.50, -math.pi / 2.0)
    mark(path, "GATE_OUT")
    return path


def nominal_audit(path, occupancy):
    width, height = occupancy.size
    pixels = occupancy.load()
    for x, y, yaw, _ in path:
        c, s = math.cos(yaw), math.sin(yaw)
        local_x = -0.11
        while local_x <= 0.280001:
            local_y = -0.13
            while local_y <= 0.130001:
                mx = x + c * local_x - s * local_y
                my = y + s * local_x + c * local_y
                col = math.floor(mx / MAP_RESOLUTION_M)
                row = height - 1 - math.floor(my / MAP_RESOLUTION_M)
                if not (0 <= col < width and 0 <= row < height):
                    raise RuntimeError("footprint leaves map")
                if pixels[col, row] != 254:
                    raise RuntimeError(
                        f"footprint enters green/grid at ({mx:.3f},{my:.3f})")
                local_y += 0.02
            local_x += 0.02
    for first, second in zip(path, path[1:]):
        ds = math.hypot(second[0] - first[0], second[1] - first[1])
        if ds > 1.0e-6:
            curvature = abs(normalize(second[2] - first[2])) / ds
            if curvature > 1.0 / MIN_TURN_RADIUS_M + 0.02:
                raise RuntimeError(f"curvature {curvature:.4f} exceeds R=0.35")


def main():
    package = Path(__file__).resolve().parents[1]
    map_file = package / "maps" / "rdk_2026_hospital_static_1cm.pgm"
    output = package / "config" / "map_registered_tubes_v1.csv"
    occupancy = Image.open(map_file).convert("L")
    rows = []
    for direction in ("CW", "CCW"):
        for lane, offset in LANES.items():
            path = build(direction, lane, offset)
            nominal_audit(path, occupancy)
            tube_id = f"map_v1/{direction}/{lane}"
            for index, (x, y, yaw, event) in enumerate(path):
                rows.append((direction, tube_id, index, x, y, yaw, event))
    with output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("direction", "tube_id", "pose_index", "x", "y", "yaw", "event"))
        writer.writerows(rows)
    print(f"generated {output}: {len(rows)} poses, nominal audit PASS")


if __name__ == "__main__":
    main()
