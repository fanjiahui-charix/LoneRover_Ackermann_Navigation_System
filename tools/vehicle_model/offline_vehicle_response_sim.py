#!/usr/bin/env python3
"""Offline Ackermann/MPPI shadow simulation calibrated from a real rosbag.

This is deliberately a standalone model: it never creates a ROS node and never
publishes a command.  The recorded Smac paths, odometry, command chain and raw
costmaps are used as the field evidence/calibration set.  It is not a binary
replay of Nav2's MPPI plugin (the host does not have the target rclpy stack).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


PI = math.pi
_OBSTACLE_TREE: Any = None


def stamp_to_sec(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def wrap(a: float) -> float:
    return (a + PI) % (2.0 * PI) - PI


def quat_yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


@dataclass
class TimedCostmap:
    stamp: float
    frame: str
    resolution: float
    size_x: int
    size_y: int
    origin_x: float
    origin_y: float
    data: np.ndarray


@dataclass
class PathData:
    name: str
    frame: str
    xy: np.ndarray
    yaw: np.ndarray
    arc: np.ndarray
    stamp: float = 0.0

    @property
    def end(self) -> np.ndarray:
        return self.xy[-1]


@dataclass
class Tune:
    name: str
    lookahead: float
    cross_gain: float
    heading_gain: float
    terminal_guard: bool
    # The real MPPI goal critic must account for command/vehicle delay.  These
    # two fields are the point at which the shadow controller starts its
    # terminal speed profile, measured along the accepted Smac path.
    latch_arc: float = 0.65
    latch_dist: float = 0.80
    terminal_heading_gain: float = 1.8
    terminal_lookahead: float = 0.22
    feedforward_gain: float = 0.0
    lateral_accel: float = 0.80
    speed_scaled_terminal: bool = False
    latch_path_fraction: float = 0.0


@dataclass
class Fit:
    delay_sec: float
    tau_v_sec: float
    tau_w_sec: float
    gate_plan_stamp: float
    gate_actual_final: tuple[float, float, float]
    actual_xy_error: float
    actual_yaw_error_deg: float


@dataclass
class MapData:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    occupancy: np.ndarray


def register_costmap_types(ts: Any) -> None:
    msg_dir = Path("/opt/ros/humble/share/nav2_msgs/msg")
    ts.register(get_types_from_msg((msg_dir / "CostmapMetaData.msg").read_text(), "nav2_msgs/msg/CostmapMetaData"))
    ts.register(get_types_from_msg((msg_dir / "Costmap.msg").read_text(), "nav2_msgs/msg/Costmap"))


def deserialize_bag(bag: Path) -> tuple[dict[str, list[tuple[float, float, float]]], list[PathData], list[TimedCostmap], list[TimedCostmap], MapData, list[tuple[float, float, float, float, float]], list[tuple[float, float, float]]]:
    ts = get_typestore(Stores.ROS2_HUMBLE)
    register_costmap_types(ts)
    commands: dict[str, list[tuple[float, float, float]]] = {"/cmd_vel_nav_raw": [], "/cmd_vel_nav": [], "/cmd_vel_safe": []}
    plans: list[PathData] = []
    global_maps: list[TimedCostmap] = []
    local_maps: list[TimedCostmap] = []
    map_data: MapData | None = None
    odom: list[tuple[float, float, float, float, float]] = []
    map_pose: list[tuple[float, float, float]] = []

    with Reader(bag) as reader:
        for connection, bag_stamp_ns, raw in reader.messages():
            topic = connection.topic
            t = bag_stamp_ns * 1.0e-9
            if topic in commands:
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                commands[topic].append((t, float(msg.linear.x), float(msg.angular.z)))
            elif topic == "/odom":
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                p = msg.pose.pose.position
                odom.append((t, float(p.x), float(p.y), quat_yaw(msg.pose.pose.orientation), float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z)))
            elif topic == "/localization/current_pose":
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                p = msg.pose.position
                map_pose.append((t, float(p.x), float(p.y), quat_yaw(msg.pose.orientation)))
            elif topic == "/plan":
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                xy = np.array([[float(p.pose.position.x), float(p.pose.position.y)] for p in msg.poses], dtype=float)
                yaw = np.array([quat_yaw(p.pose.orientation) for p in msg.poses], dtype=float)
                if len(xy) >= 2:
                    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))
                    plans.append(PathData(f"recorded_plan_{len(plans)}", msg.header.frame_id, xy, yaw, arc, t))
            elif topic in ("/global_costmap/costmap_raw", "/local_costmap/costmap_raw"):
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                md = msg.metadata
                cm = TimedCostmap(
                    t,
                    str(msg.header.frame_id),
                    float(md.resolution),
                    int(md.size_x),
                    int(md.size_y),
                    float(md.origin.position.x),
                    float(md.origin.position.y),
                    np.asarray(msg.data, dtype=np.uint8).copy(),
                )
                (global_maps if topic.startswith("/global") else local_maps).append(cm)
            elif topic == "/map":
                msg = ts.deserialize_cdr(raw, connection.msgtype)
                info = msg.info
                map_data = MapData(float(info.resolution), int(info.width), int(info.height), float(info.origin.position.x), float(info.origin.position.y), np.asarray(msg.data, dtype=np.int8).copy().reshape((int(info.height), int(info.width))))

    if map_data is None or len(plans) < 2 or not global_maps:
        raise RuntimeError("bag lacks /map, /plan, or global costmap data")
    return commands, plans, global_maps, local_maps, map_data, odom, map_pose


def interp_series(items: list[tuple[float, float, float]], grid: np.ndarray, col: int) -> np.ndarray:
    if not items:
        return np.zeros_like(grid)
    x = np.asarray([v[0] for v in items])
    y = np.asarray([v[col] for v in items])
    return np.interp(grid, x, y, left=y[0], right=y[-1])


def fit_first_order(commands: dict[str, list[tuple[float, float, float]]], odom: list[tuple[float, float, float, float, float, float]], plans: list[PathData], map_pose: list[tuple[float, float, float]]) -> Fit:
    # Reader order is deterministic: the second /plan is the Gate plan.
    c = commands["/cmd_vel_safe"]
    o = odom
    if not c or not o:
        return Fit(0.08, 0.20, 0.20, 0.0, (0.0, 0.0, 0.0), 0.0, 0.0)
    ctimes = np.asarray([x[0] for x in c])
    cvals = np.asarray([[x[1], x[2]] for x in c])
    t0 = float(plans[1].stamp)
    t1 = min(float(ctimes[-1]), t0 + 12.0)
    grid = np.arange(t0, t1, 0.02)
    u_v = np.interp(grid, ctimes, cvals[:, 0])
    u_w = np.interp(grid, ctimes, cvals[:, 1])
    ot = np.asarray([x[0] for x in o])
    y_v = np.interp(grid, ot, np.asarray([x[4] for x in o]))
    y_w = np.interp(grid, ot, np.asarray([x[5] for x in o]))

    def fit_channel(u: np.ndarray, y: np.ndarray) -> tuple[float, float]:
        best = (float("inf"), 0.08, 0.20)
        for delay in np.arange(0.0, 0.401, 0.01):
            shifted = np.interp(grid - delay, grid, u, left=u[0], right=u[-1])
            for tau in np.arange(0.04, 0.81, 0.02):
                pred = np.empty_like(y); pred[0] = y[0]
                alpha = min(1.0, 0.02 / tau)
                for i in range(1, len(y)):
                    pred[i] = pred[i - 1] + alpha * (shifted[i - 1] - pred[i - 1])
                err = float(np.mean((pred - y) ** 2))
                if err < best[0]:
                    best = (err, float(delay), float(tau))
        return best[1], best[2]

    delay_v, tau_v = fit_channel(u_v, y_v)
    delay_w, tau_w = fit_channel(u_w, y_w)
    delay = float(np.median([delay_v, delay_w]))

    # The recorded actual terminal pose is the last map pose after Gate starts.
    # Use the last pose in the high-rate interval, which is the pose visible to
    # the Supervisor when the Gate deadline expires.
    mp = [(t, x, y, yaw) for t, x, y, yaw in map_pose if t >= t0 and t <= t1 + 2.0]
    last = mp[-1] if mp else (0.0, plans[1].end[0], plans[1].end[1], plans[1].yaw[-1])
    goal = plans[1]
    xyerr = float(np.linalg.norm(np.asarray(last[1:3]) - goal.end))
    yawerr = abs(wrap(last[3] - float(goal.yaw[-1]))) * 180.0 / PI
    gate_stamp = t0
    return Fit(delay, tau_v, tau_w, gate_stamp, (last[1], last[2], last[3]), xyerr, yawerr)


def nearest_costmap(maps: list[TimedCostmap], stamp: float) -> TimedCostmap:
    return min(maps, key=lambda cm: abs(cm.stamp - stamp))


def cost_at(cm: TimedCostmap, x: float, y: float) -> int | None:
    ix = int(math.floor((x - cm.origin_x) / cm.resolution))
    iy = int(math.floor((y - cm.origin_y) / cm.resolution))
    if ix < 0 or iy < 0 or ix >= cm.size_x or iy >= cm.size_y:
        return None
    idx = iy * cm.size_x + ix
    if idx >= len(cm.data):
        return None
    return int(cm.data[idx])


def footprint_samples(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    # Frozen contract: front 0.28, rear -0.11, half width 0.13, padding 0.
    out: list[tuple[float, float]] = []
    for lx in np.arange(-0.11, 0.2801, 0.055):
        for ly in np.arange(-0.13, 0.1301, 0.055):
            wx = x + math.cos(yaw) * lx - math.sin(yaw) * ly
            wy = y + math.sin(yaw) * lx + math.cos(yaw) * ly
            out.append((wx, wy))
    return out


def static_collision(map_data: MapData, x: float, y: float, yaw: float) -> tuple[bool, float | None]:
    samples = np.asarray(footprint_samples(x, y, yaw), dtype=float)
    ix = np.floor((samples[:, 0] - map_data.origin_x) / map_data.resolution).astype(int)
    iy = np.floor((samples[:, 1] - map_data.origin_y) / map_data.resolution).astype(int)
    if np.any(ix < 0) or np.any(iy < 0) or np.any(ix >= map_data.width) or np.any(iy >= map_data.height):
        return True, 0.0
    if np.any(map_data.occupancy[iy, ix] >= 50):
        return True, 0.0
    min_clear = None
    if _OBSTACLE_TREE is not None:
        min_clear = float(np.min(_OBSTACLE_TREE.query(samples)[0]))
    return False, min_clear


def dynamic_cost_collision(cm: TimedCostmap, x: float, y: float, yaw: float) -> tuple[bool, bool, float | None]:
    samples = np.asarray(footprint_samples(x, y, yaw), dtype=float)
    ix = np.floor((samples[:, 0] - cm.origin_x) / cm.resolution).astype(int)
    iy = np.floor((samples[:, 1] - cm.origin_y) / cm.resolution).astype(int)
    outside = (ix < 0) | (iy < 0) | (ix >= cm.size_x) | (iy >= cm.size_y)
    valid = ~outside
    vals = np.full(len(samples), 255, dtype=np.uint8)
    if np.any(valid):
        vals[valid] = cm.data[iy[valid] * cm.size_x + ix[valid]]
    # 255 is UNKNOWN, not lethal.  Keep it visible to the report, but do not
    # count rolling-window unknown as a static collision.  254 is the Nav2
    # lethal value (values 253+ are retained as lethal if a layer uses a
    # slightly lower encoded lethal threshold).
    return bool(np.any((vals >= 253) & (vals != 255))), bool(np.any(outside) or np.any(vals == 255)), None


def prepare_path(path: PathData, start_idx: int, name: str) -> PathData:
    xy = path.xy[start_idx:].copy()
    yaw = path.yaw[start_idx:].copy()
    arc = np.concatenate(([0.0], np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))))
    return PathData(name, path.frame, xy, yaw, arc)


def nearest_index(path: PathData, x: float, y: float) -> int:
    return int(np.argmin(np.sum((path.xy - np.asarray([x, y])) ** 2, axis=1)))


def controller_command(path: PathData, x: float, y: float, yaw: float, v: float, vmax: float, tune: Tune, goal_latched: bool) -> tuple[float, float, int, float, float, bool]:
    idx = nearest_index(path, x, y)
    tangent = float(path.yaw[idx])
    dx, dy = x - path.xy[idx, 0], y - path.xy[idx, 1]
    cross = -math.sin(tangent) * dx + math.cos(tangent) * dy
    target_s = min(path.arc[-1], path.arc[idx] + tune.lookahead)
    tidx = int(np.searchsorted(path.arc, target_s, side="left"))
    tidx = min(tidx, len(path.xy) - 1)
    desired = float(path.yaw[tidx])
    # Smac supplies a discrete heading profile.  Use its local curvature as
    # a bounded feed-forward term; relying only on heading error makes the
    # delayed Ackermann plant cut the inside of the first Gate bend.  The
    # terminal final segment is clipped to the frozen Rmin contract.
    if tune.feedforward_gain:
        # Use curvature at the vehicle's current path station.  Using only
        # the lookahead endpoint can see the straight section after the Gate
        # bend while the delayed chassis is still inside the bend.
        ci = max(1, min(len(path.xy) - 2, idx))
        ds = max(1.0e-3, float(path.arc[ci + 1] - path.arc[ci - 1]))
        pc = wrap(float(path.yaw[ci + 1] - path.yaw[ci - 1])) / ds
        path_curvature = float(np.clip(pc, -1.0 / 0.35, 1.0 / 0.35))
    else:
        path_curvature = 0.0
    dist_end = float(np.linalg.norm(np.asarray([x, y]) - path.end))
    if tune.terminal_guard and dist_end < 0.85:
        # Keep the path tangent but add a bounded terminal-heading contribution.
        blend = min(1.0, max(0.0, (0.85 - dist_end) / 0.55))
        desired = wrap((1.0 - blend) * desired + blend * float(path.yaw[-1]))
    herr = wrap(desired - yaw)
    # Once terminal mode is latched, the vehicle must actively settle into
    # the formal 70--110 degree handoff window.  Reusing the cruise gains at
    # v=0.06 m/s produces almost no yaw authority (and was the reason the
    # earlier shadow runs crept past the Gate with a 130--170 degree yaw).
    terminal_mode = bool(goal_latched)
    L = max(0.22, tune.terminal_lookahead if terminal_mode else tune.lookahead)
    hg = tune.terminal_heading_gain if terminal_mode else tune.heading_gain
    cg = tune.cross_gain if not terminal_mode else min(tune.cross_gain, 0.35)
    curvature = tune.feedforward_gain * path_curvature + hg * math.sin(herr) / L - cg * cross / max(L * L, 0.12)
    curvature = float(np.clip(curvature, -1.0 / 0.35, 1.0 / 0.35))
    v_ref = vmax
    if abs(curvature) > 1.0e-5:
        v_ref = min(v_ref, math.sqrt(max(0.05, tune.lateral_accel) / abs(curvature)))
    # Delay-aware preview speed: reserve the largest curvature between the
    # current station and the next ~0.65 m. This is the virtual equivalent of
    # MPPI's forward rollout seeing the bend before the smoothed command
    # reaches the chassis.
    if tune.feedforward_gain:
        end_s = min(float(path.arc[-1]), float(path.arc[idx]) + max(0.65, tune.lookahead))
        end_idx = int(np.searchsorted(path.arc, end_s, side="right"))
        pcs = []
        for pj in range(max(1, idx), min(len(path.xy) - 1, end_idx + 1)):
            pds = max(1.0e-3, float(path.arc[pj + 1] - path.arc[pj - 1]))
            pcs.append(abs(wrap(float(path.yaw[pj + 1] - path.yaw[pj - 1])) / pds))
        if pcs and max(pcs) > 1.0e-5:
            v_ref = min(v_ref, math.sqrt(max(0.05, tune.lateral_accel) / max(pcs)))
    v_ref = min(v_ref, math.sqrt(2.0 * 0.76369 * max(0.0, dist_end)))
    remaining_arc = float(path.arc[-1] - path.arc[idx])
    # Open-loop smoother/transport overrun is approximately v*delay plus the
    # braking term.  Latch before the final 0.65 m rather than waiting for the
    # nearest-index jump to the last pose; otherwise a fast virtual car passes
    # the Gate and can never satisfy the 0.30 m handoff radius.
    if tune.latch_path_fraction > 0.0:
        # A single absolute arc threshold is wrong for the historical suffix
        # scenarios: the same 1.0 m threshold would latch immediately on a
        # 1.1 m suffix. Scale the delay-aware terminal region to each accepted
        # connector length, capped by the field tuning limit.
        latch_arc = min(tune.latch_arc, tune.latch_path_fraction * float(path.arc[-1]))
    else:
        latch_arc = tune.latch_arc * max(0.5, vmax / 0.50) if tune.speed_scaled_terminal else tune.latch_arc
    latch_dist = tune.latch_dist * max(0.5, vmax / 0.50) if tune.speed_scaled_terminal else tune.latch_dist
    if not goal_latched and remaining_arc < latch_arc and dist_end < latch_dist:
        goal_latched = True
    if goal_latched:
        abs_heading_deg = math.degrees(wrap(yaw))
        in_formal_handoff = 70.0 <= abs_heading_deg <= 110.0
        if tune.terminal_guard and in_formal_handoff and dist_end <= 0.30:
            v_ref = 0.0
        elif tune.terminal_guard and abs(wrap(float(path.yaw[-1]) - yaw)) > math.radians(4.0):
            v_ref = 0.06
        else:
            v_ref = 0.0
    elif dist_end < 0.04 and (not tune.terminal_guard or abs(wrap(float(path.yaw[-1]) - yaw)) < math.radians(4.0)):
        v_ref = 0.0
    elif tune.terminal_guard and dist_end < 0.25 and abs(wrap(float(path.yaw[-1]) - yaw)) > math.radians(4.0):
        v_ref = max(0.06, min(v_ref, 0.16))
    w_ref = v_ref * curvature
    w_cap = min(1.50, 0.80 / max(v_ref, 0.05), max(0.01, v_ref / 0.35))
    w_ref = float(np.clip(w_ref, -w_cap, w_cap))
    return float(max(0.0, v_ref)), w_ref, idx, cross, dist_end, goal_latched


def simulate(path: PathData, vmax: float, tune: Tune, fit: Fit, global_maps: list[TimedCostmap], local_maps: list[TimedCostmap], map_data: MapData) -> tuple[dict[str, Any], list[dict[str, float]]]:
    dt = 0.02
    tau_v, tau_w = fit.tau_v_sec, fit.tau_w_sec
    delay = fit.delay_sec
    x, y, yaw = map(float, [path.xy[0, 0], path.xy[0, 1], path.yaw[0]])
    v = 0.0
    w = 0.0
    desired_history: list[tuple[float, float, float]] = []
    goal_latched = False
    rows: list[dict[str, float]] = []
    static_collisions = 0
    dynamic_lethal = 0
    dynamic_unknown = 0
    max_cross = 0.0
    sum_cross2 = 0.0
    min_clear = None
    max_steps = int(15.0 / dt)
    for k in range(max_steps):
        sim_t = k * dt
        cmd_v, cmd_w, idx, cross, dist_end, goal_latched = controller_command(path, x, y, yaw, v, vmax, tune, goal_latched)
        desired_history.append((sim_t, cmd_v, cmd_w))
        delayed_t = sim_t - delay
        # The command stream is generated on the fixed dt grid.  Indexing the
        # delayed sample directly is equivalent to the old np.interp call but
        # avoids rebuilding two growing arrays at every simulation step; this
        # matters when sweeping thousands of virtual runs.
        delay_steps = delay / dt
        if delay_steps >= k:
            dv, dw = desired_history[0][1], desired_history[0][2]
        else:
            q = max(0.0, k - delay_steps)
            lo = int(math.floor(q)); hi = min(k, lo + 1); frac = q - lo
            dv = float((1.0 - frac) * desired_history[lo][1] + frac * desired_history[hi][1])
            dw = float((1.0 - frac) * desired_history[lo][2] + frac * desired_history[hi][2])
        # Deployed velocity-smoother limits (profile safe_05) and angular slew.
        dv_limit = (0.53129 if dv >= v else 0.76369) * dt
        dw_limit = 2.0 * dt
        v_cmd = v + float(np.clip(dv - v, -dv_limit, dv_limit))
        w_cmd = w + float(np.clip(dw - w, -dw_limit, dw_limit))
        v += dt / max(tau_v, dt) * (v_cmd - v)
        w += dt / max(tau_w, dt) * (w_cmd - w)
        x += dt * v * math.cos(yaw)
        y += dt * v * math.sin(yaw)
        yaw = wrap(yaw + dt * w)
        scoll, clearance = static_collision(map_data, x, y, yaw)
        cm = nearest_costmap(global_maps, fit.gate_plan_stamp + sim_t)
        lethal, unknown, _ = dynamic_cost_collision(cm, x, y, yaw)
        static_collisions += int(scoll)
        dynamic_lethal += int(lethal)
        dynamic_unknown += int(unknown and not lethal)
        max_cross = max(max_cross, abs(cross))
        sum_cross2 += cross * cross
        if clearance is not None:
            min_clear = clearance if min_clear is None else min(min_clear, clearance)
        rows.append({"t": sim_t, "x": x, "y": y, "yaw": yaw, "v": v, "w": w, "cmd_v": cmd_v, "cmd_w": cmd_w, "nearest_idx": idx, "cross": cross, "dist_end": dist_end})
        if dist_end < 0.025 and abs(wrap(float(path.yaw[-1]) - yaw)) < math.radians(4.0):
            break

    final = rows[-1]
    xy_error = float(math.hypot(final["x"] - path.end[0], final["y"] - path.end[1]))
    yaw_error = abs(wrap(final["yaw"] - float(path.yaw[-1]))) * 180.0 / PI
    abs_yaw_deg = (final["yaw"] * 180.0 / PI) % 360.0
    abs_yaw_deg = abs_yaw_deg - 360.0 if abs_yaw_deg > 180.0 else abs_yaw_deg
    # Supervisor's formal handoff region is a 0.30 m radius around
    # (2.50,2.30), with the only accepted yaw interval 70--110 deg.
    handoff = bool(path.name.startswith("gate") and xy_error <= 0.30 and 70.0 <= abs_yaw_deg <= 110.0)
    metrics = {
        "path": path.name,
        "tune": tune.name,
        "vmax": vmax,
        "delay_sec": delay,
        "tau_v_sec": tau_v,
        "tau_w_sec": tau_w,
        "duration_sec": float(final["t"]),
        "final_x": final["x"],
        "final_y": final["y"],
        "final_yaw_deg": abs_yaw_deg,
        "xy_error_m": xy_error,
        "terminal_yaw_error_deg": yaw_error,
        "handoff_70_110": handoff,
        "max_cross_track_m": max_cross,
        "rms_cross_track_m": math.sqrt(sum_cross2 / max(1, len(rows))),
        "min_static_clearance_m": min_clear if min_clear is not None else -1.0,
        "static_collision_samples": static_collisions,
        "global_lethal_samples": dynamic_lethal,
        "global_unknown_samples": dynamic_unknown,
        "footprint_safe": static_collisions == 0 and dynamic_lethal == 0,
    }
    return metrics, rows


def nearest_path_error(path: PathData, x: float, y: float) -> float:
    return float(np.sqrt(np.min(np.sum((path.xy - np.asarray([x, y])) ** 2, axis=1))))


def save_csv(path: Path, rows: Iterable[dict[str, Any]], fields: list[str]) -> None:
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", type=Path, required=True)
    ap.add_argument("--out", type=Path, default=Path("logs/offline_mppi_shadow"))
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    commands, plans, global_maps, local_maps, map_data, odom, map_pose = deserialize_bag(args.bag)
    global _OBSTACLE_TREE
    try:
        from scipy.spatial import cKDTree
        obstacle = np.argwhere(map_data.occupancy >= 50)
        if len(obstacle):
            obstacle_xy = np.column_stack(((obstacle[:, 1] + 0.5) * map_data.resolution + map_data.origin_x, (obstacle[:, 0] + 0.5) * map_data.resolution + map_data.origin_y))
            _OBSTACLE_TREE = cKDTree(obstacle_xy)
    except Exception:
        _OBSTACLE_TREE = None
    if len(plans) < 2:
        raise SystemExit("the supplied data must contain at least two Path segments")
    segment_a = PathData("segment_a", plans[0].frame, plans[0].xy, plans[0].yaw, plans[0].arc)
    segment_b = PathData("segment_b", plans[1].frame, plans[1].xy, plans[1].yaw, plans[1].arc)
    paths = [segment_a, segment_b, prepare_path(segment_b, 4, "segment_b_mid"), prepare_path(segment_b, 8, "segment_b_late")]
    fit = fit_first_order(commands, odom, plans, map_pose)
    # The fit routine has no bag header time for /plan; use the command segment
    # start, which is the timestamp-aligned Gate action start in this capture.
    if fit.gate_plan_stamp <= 0.0:
        fit.gate_plan_stamp = commands["/cmd_vel_safe"][0][0]
    tunes = [
        # Gains are intentionally lower than a textbook pure-pursuit law:
        # Keep the angular command below the configured vehicle limit; the
        # shadow is intended to compare delay-aware tracking, not saturation.
        # ceiling, and the field problem was late/oscillatory turns, not a
        # lack of raw angular authority.
        Tune("field_xy_priority", 0.45, 0.60, 0.60, False),
        Tune("field_formal_guard", 0.45, 0.60, 0.60, True, 0.95, 1.05),
        Tune("field_guard_heading075", 0.45, 0.80, 0.75, True, 0.95, 1.05),
        Tune("field_guard_short", 0.35, 0.60, 0.70, True, 0.95, 1.05),
        # Delay-aware terminal variants: start braking before the last
        # ~0.65 m so the fitted 0.205 s transport delay cannot carry the
        # virtual chassis through the 70--110 deg handoff window.
        Tune("field_guard_early", 0.40, 0.45, 0.45, True, 1.10, 1.20, 1.8, 0.22),
        Tune("field_guard_early_short", 0.32, 0.45, 0.55, True, 1.10, 1.20, 2.2, 0.22),
        Tune("ff_formal_guard", 0.40, 0.32, 0.35, True, 0.95, 1.05, 1.8, 0.22, 0.75, 0.80),
        Tune("ff_formal_low", 0.35, 0.25, 0.30, True, 0.95, 1.05, 1.8, 0.22, 0.55, 0.80),
        Tune("ff_early_guard", 0.36, 0.28, 0.30, True, 1.10, 1.20, 2.0, 0.22, 0.65, 0.80),
        Tune("ff_strong_guard", 0.30, 0.22, 0.25, True, 1.10, 1.20, 2.2, 0.22, 0.90, 0.80),
        # Robust field variants account for transport delay by reducing the
        # lateral-acceleration budget in the high-curvature Gate connector.
        Tune("ff_robust_lat035", 0.40, 0.30, 0.35, True, 1.00, 1.10, 2.0, 0.22, 0.85, 0.35),
        Tune("ff_robust_lat025", 0.36, 0.28, 0.30, True, 1.05, 1.15, 2.2, 0.22, 0.85, 0.25),
        Tune("ff_robust_lat045", 0.42, 0.32, 0.38, True, 1.00, 1.10, 1.8, 0.22, 0.85, 0.45),
        Tune("ff_speed_scaled", 0.38, 0.28, 0.32, True, 0.95, 1.05, 2.0, 0.22, 0.80, 0.40, True),
        Tune("ff_speed_scaled_low", 0.34, 0.25, 0.30, True, 1.00, 1.10, 2.2, 0.22, 0.85, 0.35, True),
        Tune("ff_robust_adaptive", 0.36, 0.25, 0.30, True, 1.05, 1.15, 2.2, 0.22, 0.85, 0.25, False, 0.63),
        Tune("ff_robust_adaptive_lat026", 0.36, 0.25, 0.30, True, 1.05, 1.15, 2.2, 0.22, 0.85, 0.26, False, 0.63),
    ]
    all_metrics: list[dict[str, Any]] = []
    path_rows: list[dict[str, Any]] = []
    for p in paths:
        for vmax in (0.50, 0.60, 0.70, 0.80):
            for tune in tunes:
                metrics, rows = simulate(p, vmax, tune, fit, global_maps, local_maps, map_data)
                all_metrics.append(metrics)
                stem = f"{p.name}_{tune.name}_v{vmax:.2f}".replace(".", "p")
                save_csv(args.out / f"{stem}.csv", rows, ["t", "x", "y", "yaw", "v", "w", "cmd_v", "cmd_w", "nearest_idx", "cross", "dist_end"])
    (args.out / "metrics.json").write_text(json.dumps(all_metrics, indent=2))
    with (args.out / "metrics.csv").open("w", newline="") as f:
        if all_metrics:
            fields = list(all_metrics[0].keys())
            w = csv.DictWriter(f, fieldnames=fields); w.writeheader(); w.writerows(all_metrics)
    path_rows = [{"path": p.name, "frame": p.frame, "poses": len(p.xy), "length_m": float(p.arc[-1]), "start_x": float(p.xy[0, 0]), "start_y": float(p.xy[0, 1]), "start_yaw_deg": float(p.yaw[0] * 180.0 / PI), "end_x": float(p.end[0]), "end_y": float(p.end[1]), "end_yaw_deg": float(p.yaw[-1] * 180.0 / PI), "static_path_legal": all(not static_collision(map_data, float(x), float(y), float(yaw))[0] for (x, y), yaw in zip(p.xy, p.yaw))} for p in paths]
    (args.out / "paths.json").write_text(json.dumps(path_rows, indent=2))
    summary = {
        "bag": str(args.bag),
        "model": "standalone Ackermann shadow; calibrated from /cmd_vel_safe and /odom; no ROS node/publisher",
        "formal_handoff_yaw_deg": [70.0, 110.0],
        "rmin_m": 0.35,
        "speeds_mps": [0.50, 0.60, 0.70, 0.80],
        "fit": {"delay_sec": fit.delay_sec, "tau_v_sec": fit.tau_v_sec, "tau_w_sec": fit.tau_w_sec, "gate_actual_final": fit.gate_actual_final, "gate_actual_xy_error_m": fit.actual_xy_error, "gate_actual_yaw_error_deg": fit.actual_yaw_error_deg},
        "costmap_topics": {"global_count": len(global_maps), "local_count": len(local_maps), "global_frame": global_maps[0].frame, "local_frame": local_maps[0].frame if local_maps else None},
        "paths": path_rows,
        "results_csv": str(args.out / "metrics.csv"),
    }
    (args.out / "summary.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
