from __future__ import annotations

import heapq
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any, Sequence

N = 12
BLOCKED_PENALTY = 1000000.0

SQRT2 = math.sqrt(2.0)
DIRS4: tuple[tuple[int, int], ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))
DIRS8: tuple[tuple[int, int], ...] = DIRS4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))
DIR_NAMES: dict[tuple[int, int], str] = {
    (1, 0): "E",
    (-1, 0): "W",
    (0, 1): "S",
    (0, -1): "N",
    (1, 1): "SE",
    (-1, 1): "SW",
    (1, -1): "NE",
    (-1, -1): "NW",
    (0, 0): "WAIT",
}


@dataclass(slots=True)
class RouterConfig:
    diagonal: bool = True
    cost_mode: str = "exact"
    congestion_weight: float = 4.5
    turn_penalty: float = 0.0
    clearance_weight: float = 0.0
    clearance_radius: int = 2
    cell_size_m: float = 1.0
    worker_speed_mps: float = 1.4
    optimize_order: bool = False
    snap_start_if_blocked: bool = True
    snap_radius: int = 2
    decel_mps2: float = 1.3


def edge_weight(distance_cells: float, rho_v: float, blocked: bool = False, weight: float = 4.5) -> float:
    return distance_cells * (1.0 + weight * rho_v * rho_v) + BLOCKED_PENALTY * (1 if blocked else 0)


def _normalize_grid(grid: Sequence[Sequence[Any]]) -> tuple[int, int, list[int]]:
    if not grid or not grid[0]:
        return 0, 0, []
    height = len(grid)
    width = len(grid[0])
    flat = [0] * (width * height)
    for y in range(height):
        row = grid[y]
        row_len = len(row)
        base = y * width
        for x in range(min(width, row_len)):
            flat[base + x] = 1 if row[x] else 0
    return width, height, flat


def _normalize_rho(
    rho: Sequence[Sequence[Any]] | None,
    width: int,
    height: int,
) -> list[float]:
    flat = [0.0] * (width * height)
    if rho is None:
        return flat
    for y in range(min(height, len(rho))):
        row = rho[y]
        base = y * width
        for x in range(min(width, len(row))):
            try:
                val = float(row[x])
            except (TypeError, ValueError):
                val = 0.0
            if not math.isfinite(val):
                val = 0.0
            flat[base + x] = 0.0 if val < 0.0 else (1.0 if val > 1.0 else val)
    return flat


def _compute_clearance_penalty(
    flat_grid: list[int],
    width: int,
    height: int,
    radius: int,
) -> list[float]:
    size = width * height
    if radius <= 0 or size == 0:
        return [0.0] * size
    dist = [radius + 1] * size
    q: deque[int] = deque()
    for idx, blocked in enumerate(flat_grid):
        if blocked:
            dist[idx] = 0
            q.append(idx)
    if not q:
        return [0.0] * size
    while q:
        u = q.popleft()
        du = dist[u]
        if du >= radius:
            continue
        ux = u % width
        uy = u // width
        nd = du + 1
        for dx, dy in DIRS8:
            vx = ux + dx
            vy = uy + dy
            if 0 <= vx < width and 0 <= vy < height:
                v = vy * width + vx
                if nd < dist[v]:
                    dist[v] = nd
                    q.append(v)
    inv_r = 1.0 / (radius + 1.0)
    return [0.0 if d > radius else ((radius + 1 - d) * inv_r) ** 2 for d in dist]


def _heuristic(ux: int, uy: int, tx: int, ty: int, diagonal: bool) -> float:
    dx = abs(ux - tx)
    dy = abs(uy - ty)
    if diagonal:
        mn = dx if dx < dy else dy
        mx = dy if dx < dy else dx
        return mx + (SQRT2 - 1.0) * mn
    return float(dx + dy)


def _find_nearest_free(
    flat_grid: list[int],
    width: int,
    height: int,
    start: tuple[int, int],
    max_radius: int,
) -> tuple[int, int] | None:
    sx, sy = start
    if 0 <= sx < width and 0 <= sy < height and flat_grid[sy * width + sx] == 0:
        return start
    best: tuple[int, int] | None = None
    best_dist = math.inf
    for r in range(1, max_radius + 1):
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if abs(dx) != r and abs(dy) != r:
                    continue
                nx, ny = sx + dx, sy + dy
                if 0 <= nx < width and 0 <= ny < height and flat_grid[ny * width + nx] == 0:
                    d = dx * dx + dy * dy
                    if d < best_dist:
                        best_dist = d
                        best = (nx, ny)
        if best is not None:
            return best
    return None


def _turn_cost(prev_dir: int, next_dir: int, dirs: tuple[tuple[int, int], ...], penalty: float) -> float:
    if prev_dir < 0 or prev_dir == next_dir or penalty <= 0.0:
        return 0.0
    pdx, pdy = dirs[prev_dir]
    ndx, ndy = dirs[next_dir]
    p_len = SQRT2 if (pdx != 0 and pdy != 0) else 1.0
    n_len = SQRT2 if (ndx != 0 and ndy != 0) else 1.0
    cos_theta = max(-1.0, min(1.0, (pdx * ndx + pdy * ndy) / (p_len * n_len)))
    return penalty * (1.0 - cos_theta) * 0.5


def _edge_cost(
    step_len: float,
    rho_v: float,
    clear_v: float,
    cfg: RouterConfig,
) -> float:
    cost = edge_weight(step_len, rho_v, False, cfg.congestion_weight)
    if cfg.cost_mode == "industrial":
        cost += step_len * cfg.clearance_weight * clear_v
    return cost


def astar_flat(
    flat_grid: list[int],
    flat_rho: list[float],
    clearance: list[float],
    width: int,
    height: int,
    start: tuple[int, int],
    target: tuple[int, int],
    cfg: RouterConfig,
) -> tuple[list[tuple[int, int]], float, int] | None:
    sx, sy = start
    tx, ty = target
    if not (0 <= sx < width and 0 <= sy < height and 0 <= tx < width and 0 <= ty < height):
        return None
    s_idx = sy * width + sx
    t_idx = ty * width + tx
    if flat_grid[s_idx] or flat_grid[t_idx]:
        return None
    if s_idx == t_idx:
        return [start], 0.0, 1
    dirs = DIRS8 if cfg.diagonal else DIRS4
    step_lens = tuple(SQRT2 if (dx != 0 and dy != 0) else 1.0 for dx, dy in dirs)
    use_turns = cfg.cost_mode == "industrial" and cfg.turn_penalty > 0.0
    start_state = (s_idx, -1)
    h0 = _heuristic(sx, sy, tx, ty, cfg.diagonal)
    pq: list[tuple[float, float, float, int, int]] = [(h0, h0, 0.0, s_idx, -1)]
    g_score: dict[tuple[int, int], float] = {start_state: 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    closed: set[tuple[int, int]] = set()
    expanded = 0
    while pq:
        _, _, g, u_idx, prev_dir = heapq.heappop(pq)
        u_state = (u_idx, prev_dir)
        if u_state in closed:
            continue
        if g > g_score.get(u_state, math.inf) + 1e-12:
            continue
        expanded += 1
        if u_idx == t_idx:
            path: list[tuple[int, int]] = []
            curr = u_state
            while True:
                idx = curr[0]
                path.append((idx % width, idx // width))
                if curr not in came_from:
                    break
                curr = came_from[curr]
            path.reverse()
            return path, g, expanded
        closed.add(u_state)
        ux = u_idx % width
        uy = u_idx // width
        for d_idx, (dx, dy) in enumerate(dirs):
            vx = ux + dx
            vy = uy + dy
            if not (0 <= vx < width and 0 <= vy < height):
                continue
            v_idx = vy * width + vx
            if flat_grid[v_idx]:
                continue
            if dx != 0 and dy != 0:
                if flat_grid[uy * width + vx] or flat_grid[vy * width + ux]:
                    continue
            next_dir = d_idx if use_turns else -1
            v_state = (v_idx, next_dir)
            if v_state in closed:
                continue
            step_c = _edge_cost(step_lens[d_idx], flat_rho[v_idx], clearance[v_idx], cfg)
            if use_turns:
                step_c += _turn_cost(prev_dir, d_idx, dirs, cfg.turn_penalty)
            ng = g + step_c
            if ng < g_score.get(v_state, math.inf) - 1e-12:
                g_score[v_state] = ng
                came_from[v_state] = u_state
                hv = _heuristic(vx, vy, tx, ty, cfg.diagonal)
                heapq.heappush(pq, (ng + hv, hv, ng, v_idx, next_dir))
    return None


def astar(
    grid: Sequence[Sequence[Any]],
    rho: Sequence[Sequence[Any]] | None,
    start: tuple[int, int],
    target: tuple[int, int],
    diagonal: bool = True,
    config: RouterConfig | None = None,
) -> tuple[list[tuple[int, int]], float] | None:
    width, height, flat_grid = _normalize_grid(grid)
    if width == 0 or height == 0:
        return None
    flat_rho = _normalize_rho(rho, width, height)
    cfg = config or RouterConfig(diagonal=diagonal)
    clearance = (
        _compute_clearance_penalty(flat_grid, width, height, cfg.clearance_radius)
        if cfg.clearance_weight > 0.0 and cfg.cost_mode == "industrial"
        else [0.0] * (width * height)
    )
    res = astar_flat(flat_grid, flat_rho, clearance, width, height, start, target, cfg)
    if res is None:
        return None
    path, cost, _ = res
    return path, cost


def optimize_waypoint_order(
    flat_grid: list[int],
    flat_rho: list[float],
    clearance: list[float],
    width: int,
    height: int,
    start: tuple[int, int],
    targets: list[tuple[int, int]],
    cfg: RouterConfig,
) -> tuple[list[tuple[int, int]], dict[tuple[tuple[int, int], tuple[int, int]], tuple[list[tuple[int, int]], float, int]]] | None:
    unique_targets: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for t in targets:
        if t not in seen:
            seen.add(t)
            unique_targets.append(t)
    k = len(unique_targets)
    if k <= 1:
        return unique_targets, {}
    nodes = [start] + unique_targets
    cache: dict[tuple[tuple[int, int], tuple[int, int]], tuple[list[tuple[int, int]], float, int]] = {}
    dist = [[math.inf] * (k + 1) for _ in range(k + 1)]
    for i in range(k + 1):
        for j in range(1, k + 1):
            if i == j:
                dist[i][j] = 0.0
                continue
            seg = astar_flat(flat_grid, flat_rho, clearance, width, height, nodes[i], nodes[j], cfg)
            if seg is None:
                if i == 0:
                    return None
                continue
            cache[(nodes[i], nodes[j])] = seg
            dist[i][j] = seg[1]
    if k <= 11:
        full_mask = (1 << k) - 1
        dp: list[list[float]] = [[math.inf] * k for _ in range(1 << k)]
        parent: list[list[int]] = [[-1] * k for _ in range(1 << k)]
        for i in range(k):
            dp[1 << i][i] = dist[0][i + 1]
        for mask in range(1, full_mask + 1):
            for last in range(k):
                curr_cost = dp[mask][last]
                if not math.isfinite(curr_cost):
                    continue
                for nxt in range(k):
                    if mask & (1 << nxt):
                        continue
                    edge = dist[last + 1][nxt + 1]
                    if not math.isfinite(edge):
                        continue
                    nmask = mask | (1 << nxt)
                    cand = curr_cost + edge
                    if cand < dp[nmask][nxt]:
                        dp[nmask][nxt] = cand
                        parent[nmask][nxt] = last
        best_last = min(range(k), key=lambda i: dp[full_mask][i])
        if not math.isfinite(dp[full_mask][best_last]):
            return None
        order_idx: list[int] = []
        mask = full_mask
        curr = best_last
        while curr != -1:
            order_idx.append(curr)
            prev = parent[mask][curr]
            mask ^= 1 << curr
            curr = prev
        order_idx.reverse()
        return [unique_targets[i] for i in order_idx], cache
    unvisited = set(range(1, k + 1))
    order: list[int] = []
    cur = 0
    while unvisited:
        nxt = min(unvisited, key=lambda idx: dist[cur][idx])
        if not math.isfinite(dist[cur][nxt]):
            return None
        order.append(nxt)
        unvisited.remove(nxt)
        cur = nxt
    improved = True
    while improved:
        improved = False
        for i in range(len(order) - 1):
            for j in range(i + 1, len(order)):
                new_order = order[:i] + order[i : j + 1][::-1] + order[j + 1 :]
                old_c = dist[0][order[0]] + sum(dist[order[m]][order[m + 1]] for m in range(len(order) - 1))
                new_c = dist[0][new_order[0]] + sum(dist[new_order[m]][new_order[m + 1]] for m in range(len(new_order) - 1))
                if new_c + 1e-9 < old_c:
                    order = new_order
                    improved = True
    return [nodes[idx] for idx in order], cache


def compress_steps(steps: list[dict[str, Any]], cell_size_m: float = 1.0) -> list[dict[str, Any]]:
    if not steps:
        return []
    instructions: list[dict[str, Any]] = []
    curr_dir = steps[0]["dir"]
    curr_from = steps[0]["from"]
    curr_to = steps[0]["to"]
    cells = 1
    seg_dist = steps[0]["distance"]
    for s in steps[1:]:
        if s["dir"] == curr_dir:
            curr_to = s["to"]
            cells += 1
            seg_dist += s["distance"]
        else:
            instructions.append(
                {
                    "action": "WAIT" if curr_dir == "WAIT" else f"Move {curr_dir}",
                    "dir": curr_dir,
                    "cells": cells,
                    "distance_m": round(seg_dist * cell_size_m, 2),
                    "from": curr_from,
                    "to": curr_to,
                }
            )
            curr_dir = s["dir"]
            curr_from = s["from"]
            curr_to = s["to"]
            cells = 1
            seg_dist = s["distance"]
    instructions.append(
        {
            "action": "WAIT" if curr_dir == "WAIT" else f"Move {curr_dir}",
            "dir": curr_dir,
            "cells": cells,
            "distance_m": round(seg_dist * cell_size_m, 2),
            "from": curr_from,
            "to": curr_to,
        }
    )
    return instructions


def _empty_result(reason: str, compute_ms: float = 0.0) -> dict[str, Any]:
    return {
        "path": [],
        "steps": [],
        "instructions": [],
        "cost": 0.0,
        "distance": 0.0,
        "estimated_time_sec": 0.0,
        "turns": 0,
        "avg_congestion": 0.0,
        "max_congestion": 0.0,
        "reachable": False,
        "reason": reason,
        "waypoints_order": [],
        "snapped_start": None,
        "nodes_expanded": 0,
        "compute_time_ms": round(compute_ms, 3),
    }


def route(
    grid: Sequence[Sequence[Any]],
    rho: Sequence[Sequence[Any]] | None,
    start: Sequence[int],
    targets: Sequence[Sequence[int]],
    diagonal: bool = True,
    config: RouterConfig | None = None,
) -> dict[str, Any]:
    t0 = time.perf_counter()
    width, height, flat_grid = _normalize_grid(grid)
    if width == 0 or height == 0:
        return _empty_result("invalid_grid", (time.perf_counter() - t0) * 1000.0)
    if not targets:
        return _empty_result("no_targets", (time.perf_counter() - t0) * 1000.0)
    cfg = config or RouterConfig(diagonal=diagonal)
    if config is None:
        cfg.diagonal = diagonal
    raw_start = (int(start[0]), int(start[1]))
    if not (0 <= raw_start[0] < width and 0 <= raw_start[1] < height):
        return _empty_result("start_out_of_bounds", (time.perf_counter() - t0) * 1000.0)
    eff_start = raw_start
    snapped_start: list[int] | None = None
    if flat_grid[raw_start[1] * width + raw_start[0]]:
        if cfg.snap_start_if_blocked:
            rescued = _find_nearest_free(flat_grid, width, height, raw_start, cfg.snap_radius)
            if rescued is None:
                return _empty_result("start_blocked", (time.perf_counter() - t0) * 1000.0)
            eff_start = rescued
            snapped_start = [rescued[0], rescued[1]]
        else:
            return _empty_result("start_blocked", (time.perf_counter() - t0) * 1000.0)
    norm_targets: list[tuple[int, int]] = []
    for t in targets:
        tx, ty = int(t[0]), int(t[1])
        if not (0 <= tx < width and 0 <= ty < height):
            return _empty_result("target_out_of_bounds", (time.perf_counter() - t0) * 1000.0)
        if flat_grid[ty * width + tx]:
            return _empty_result("target_blocked", (time.perf_counter() - t0) * 1000.0)
        norm_targets.append((tx, ty))
    flat_rho = _normalize_rho(rho, width, height)
    clearance = (
        _compute_clearance_penalty(flat_grid, width, height, cfg.clearance_radius)
        if cfg.clearance_weight > 0.0 and cfg.cost_mode == "industrial"
        else [0.0] * (width * height)
    )
    seg_cache: dict[tuple[tuple[int, int], tuple[int, int]], tuple[list[tuple[int, int]], float, int]] = {}
    ordered_targets = norm_targets
    if cfg.optimize_order and len(norm_targets) > 1:
        opt = optimize_waypoint_order(
            flat_grid, flat_rho, clearance, width, height, eff_start, norm_targets, cfg
        )
        if opt is None:
            return _empty_result("no_path", (time.perf_counter() - t0) * 1000.0)
        ordered_targets, seg_cache = opt
    full_path: list[list[int]] = [[eff_start[0], eff_start[1]]]
    steps: list[dict[str, Any]] = []
    total_cost = 0.0
    total_distance = 0.0
    total_expanded = 0
    congestion_samples: list[float] = []
    turns = 0
    prev_dir: str | None = None
    curr = eff_start
    for target in ordered_targets:
        seg = seg_cache.get((curr, target))
        if seg is None:
            seg = astar_flat(flat_grid, flat_rho, clearance, width, height, curr, target, cfg)
        if seg is None:
            return _empty_result("no_path", (time.perf_counter() - t0) * 1000.0)
        seg_path, seg_cost, seg_expanded = seg
        total_expanded += seg_expanded
        total_cost += seg_cost
        for i in range(1, len(seg_path)):
            ax, ay = seg_path[i - 1]
            bx, by = seg_path[i]
            dx, dy = bx - ax, by - ay
            d_name = DIR_NAMES.get((dx, dy), "MOVE")
            step_d = SQRT2 if (dx != 0 and dy != 0) else 1.0
            b_idx = by * width + bx
            rv = flat_rho[b_idx]
            step_c = _edge_cost(step_d, rv, clearance[b_idx], cfg)
            congestion_samples.append(rv)
            if prev_dir is not None and d_name != prev_dir:
                turns += 1
            prev_dir = d_name
            steps.append(
                {
                    "step": len(steps) + 1,
                    "from": [ax, ay],
                    "to": [bx, by],
                    "dir": d_name,
                    "distance": round(step_d, 4),
                    "congestion": round(rv, 4),
                    "cost": round(step_c, 4),
                }
            )
            total_distance += step_d
            full_path.append([bx, by])
        curr = target
    avg_rho = sum(congestion_samples) / len(congestion_samples) if congestion_samples else 0.0
    max_rho = max(congestion_samples) if congestion_samples else 0.0
    speed = max(0.1, cfg.worker_speed_mps)
    effective_speed = speed / (1.0 + 0.6 * avg_rho)
    eta_sec = (total_distance * cfg.cell_size_m) / effective_speed + turns * 0.25
    elapsed_ms = (time.perf_counter() - t0) * 1000.0
    return {
        "path": full_path,
        "steps": steps,
        "instructions": compress_steps(steps, cfg.cell_size_m),
        "cost": round(total_cost, 4),
        "distance": round(total_distance, 4),
        "estimated_time_sec": round(eta_sec, 2),
        "turns": turns,
        "avg_congestion": round(avg_rho, 4),
        "max_congestion": round(max_rho, 4),
        "reachable": True,
        "reason": "ok",
        "waypoints_order": [[t[0], t[1]] for t in ordered_targets],
        "snapped_start": snapped_start,
        "nodes_expanded": total_expanded,
        "compute_time_ms": round(elapsed_ms, 3),
    }


def _spacetime_astar(
    flat_grid: list[int],
    flat_rho: list[float],
    clearance: list[float],
    width: int,
    height: int,
    start: tuple[int, int],
    target: tuple[int, int],
    reserved_cells: set[tuple[int, int]],
    reserved_edges: set[tuple[int, int, int]],
    cfg: RouterConfig,
    start_time: int = 0,
) -> tuple[list[tuple[int, int]], float, int] | None:
    sx, sy = start
    tx, ty = target
    s_idx = sy * width + sx
    t_idx = ty * width + tx
    if flat_grid[s_idx] or flat_grid[t_idx]:
        return None
    if s_idx == t_idx:
        return [start], 0.0, 1
    dirs = (DIRS8 if cfg.diagonal else DIRS4) + ((0, 0),)
    max_time = start_time + min(256, width * height * 2)
    h0 = _heuristic(sx, sy, tx, ty, cfg.diagonal)
    pq: list[tuple[float, float, float, int, int]] = [(h0, h0, 0.0, s_idx, start_time)]
    g_score: dict[tuple[int, int], float] = {(s_idx, start_time): 0.0}
    came_from: dict[tuple[int, int], tuple[int, int]] = {}
    expanded = 0
    while pq:
        _, _, g, u_idx, t = heapq.heappop(pq)
        state = (u_idx, t)
        if g > g_score.get(state, math.inf) + 1e-12:
            continue
        expanded += 1
        if u_idx == t_idx:
            path: list[tuple[int, int]] = []
            curr = state
            while True:
                idx = curr[0]
                path.append((idx % width, idx // width))
                if curr not in came_from:
                    break
                curr = came_from[curr]
            path.reverse()
            return path, g, expanded
        if t >= max_time:
            continue
        ux = u_idx % width
        uy = u_idx // width
        nt = t + 1
        for dx, dy in dirs:
            vx = ux + dx
            vy = uy + dy
            if not (0 <= vx < width and 0 <= vy < height):
                continue
            v_idx = vy * width + vx
            if flat_grid[v_idx]:
                continue
            if dx != 0 and dy != 0:
                if flat_grid[uy * width + vx] or flat_grid[vy * width + ux]:
                    continue
            if (nt, v_idx) in reserved_cells:
                continue
            if (nt, v_idx, u_idx) in reserved_edges:
                continue
            if dx == 0 and dy == 0:
                step_c = 1.05
            else:
                step_len = SQRT2 if (dx != 0 and dy != 0) else 1.0
                step_c = _edge_cost(step_len, flat_rho[v_idx], clearance[v_idx], cfg)
            ng = g + step_c
            nstate = (v_idx, nt)
            if ng < g_score.get(nstate, math.inf) - 1e-12:
                g_score[nstate] = ng
                came_from[nstate] = state
                hv = _heuristic(vx, vy, tx, ty, cfg.diagonal)
                heapq.heappush(pq, (ng + hv, hv, ng, v_idx, nt))
    return None


def route_fleet(
    grid: Sequence[Sequence[Any]],
    rho: Sequence[Sequence[Any]] | None,
    workers: Sequence[Sequence[int]],
    targets: Sequence[Sequence[int]],
    config: RouterConfig | None = None,
) -> list[dict[str, Any]]:
    if not workers:
        return []
    cfg = config or RouterConfig()
    if len(workers) == 1:
        res = route(grid, rho, workers[0], targets, diagonal=cfg.diagonal, config=cfg)
        res["worker_id"] = 0
        return [res]
    width, height, flat_grid = _normalize_grid(grid)
    if width == 0 or height == 0 or not targets:
        return [
            {**_empty_result("no_targets" if width else "invalid_grid"), "worker_id": i}
            for i in range(len(workers))
        ]
    flat_rho = _normalize_rho(rho, width, height)
    clearance = (
        _compute_clearance_penalty(flat_grid, width, height, cfg.clearance_radius)
        if cfg.clearance_weight > 0.0 and cfg.cost_mode == "industrial"
        else [0.0] * (width * height)
    )
    reserved_cells: set[tuple[int, int]] = set()
    reserved_edges: set[tuple[int, int, int]] = set()
    results: list[dict[str, Any]] = []
    for w_idx, worker in enumerate(workers):
        t0 = time.perf_counter()
        raw_start = (int(worker[0]), int(worker[1]))
        if not (0 <= raw_start[0] < width and 0 <= raw_start[1] < height):
            results.append({**_empty_result("start_out_of_bounds"), "worker_id": w_idx})
            continue
        eff_start = raw_start
        snapped: list[int] | None = None
        if flat_grid[raw_start[1] * width + raw_start[0]]:
            rescued = (
                _find_nearest_free(flat_grid, width, height, raw_start, cfg.snap_radius)
                if cfg.snap_start_if_blocked
                else None
            )
            if rescued is None:
                results.append({**_empty_result("start_blocked"), "worker_id": w_idx})
                continue
            eff_start = rescued
            snapped = [rescued[0], rescued[1]]
        assigned = (
            [tuple(int(v) for v in targets[w_idx % len(targets)])]
            if len(targets) >= len(workers)
            else [tuple(int(v) for v in t) for t in targets]
        )
        curr = eff_start
        curr_t = 0
        full_path: list[list[int]] = [[curr[0], curr[1]]]
        steps: list[dict[str, Any]] = []
        total_cost = 0.0
        total_distance = 0.0
        total_expanded = 0
        cong_samples: list[float] = []
        turns = 0
        prev_dir: str | None = None
        ok = True
        fail_reason = "no_path"
        for tgt in assigned:
            if not (0 <= tgt[0] < width and 0 <= tgt[1] < height):
                ok, fail_reason = False, "target_out_of_bounds"
                break
            if flat_grid[tgt[1] * width + tgt[0]]:
                ok, fail_reason = False, "target_blocked"
                break
            seg = _spacetime_astar(
                flat_grid,
                flat_rho,
                clearance,
                width,
                height,
                curr,
                tgt,
                reserved_cells,
                reserved_edges,
                cfg,
                start_time=curr_t,
            )
            if seg is None:
                ok = False
                break
            seg_path, seg_cost, seg_exp = seg
            total_cost += seg_cost
            total_expanded += seg_exp
            for i in range(1, len(seg_path)):
                ax, ay = seg_path[i - 1]
                bx, by = seg_path[i]
                dx, dy = bx - ax, by - ay
                d_name = DIR_NAMES.get((dx, dy), "MOVE")
                step_d = 0.0 if (dx == 0 and dy == 0) else (SQRT2 if (dx != 0 and dy != 0) else 1.0)
                b_idx = by * width + bx
                a_idx = ay * width + ax
                rv = flat_rho[b_idx]
                cong_samples.append(rv)
                if prev_dir is not None and d_name != prev_dir:
                    turns += 1
                prev_dir = d_name
                curr_t += 1
                reserved_cells.add((curr_t, b_idx))
                reserved_edges.add((curr_t, a_idx, b_idx))
                steps.append(
                    {
                        "step": len(steps) + 1,
                        "from": [ax, ay],
                        "to": [bx, by],
                        "dir": d_name,
                        "distance": round(step_d, 4),
                        "congestion": round(rv, 4),
                        "cost": round(_edge_cost(step_d or 1.0, rv, clearance[b_idx], cfg), 4),
                    }
                )
                total_distance += step_d
                full_path.append([bx, by])
            curr = tgt
        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if not ok:
            results.append({**_empty_result(fail_reason, elapsed_ms), "worker_id": w_idx})
            continue
        avg_rho = sum(cong_samples) / len(cong_samples) if cong_samples else 0.0
        max_rho = max(cong_samples) if cong_samples else 0.0
        eff_speed = max(0.1, cfg.worker_speed_mps) / (1.0 + 0.6 * avg_rho)
        eta_sec = (total_distance * cfg.cell_size_m) / eff_speed + turns * 0.25
        results.append(
            {
                "worker_id": w_idx,
                "path": full_path,
                "steps": steps,
                "instructions": compress_steps(steps, cfg.cell_size_m),
                "cost": round(total_cost, 4),
                "distance": round(total_distance, 4),
                "estimated_time_sec": round(eta_sec, 2),
                "turns": turns,
                "avg_congestion": round(avg_rho, 4),
                "max_congestion": round(max_rho, 4),
                "reachable": True,
                "reason": "ok",
                "waypoints_order": [[t[0], t[1]] for t in assigned],
                "snapped_start": snapped,
                "nodes_expanded": total_expanded,
                "compute_time_ms": round(elapsed_ms, 3),
            }
        )
    return results
