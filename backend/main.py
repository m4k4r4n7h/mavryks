from __future__ import annotations

import asyncio
import logging
import math
import os
import time
from collections import deque
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field

from diagnostics import Diagnostics
from robotics import DriveConfig, PurePursuit, RobotPose, battery_drain, fleet_utilization, motion_dict
from router import N, RouterConfig, route, route_fleet
from safety import SafetyMonitor
from store import Store
from tasks import KINDS, TaskQueue

logging.basicConfig(
    level=os.getenv("LOG_LEVEL", "INFO").upper(),
    format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
)
logger = logging.getLogger("mavryks.backend")

GRID_SIZE = max(4, int(os.getenv("GRID_SIZE", str(N))))
VISION_STALE_SEC = float(os.getenv("VISION_STALE_SEC", "5.0"))
DB_PATH = os.getenv("MAVRYKS_DB", str(Path(__file__).with_name("mavryks.db")))


class VisionUpdate(BaseModel):
    obstacles: list[list[int]] = Field(default_factory=list)
    rho: list[list[float]] | None = None
    workers: list[list[int]] | None = None
    targets: list[list[int]] | None = None
    robots: list[dict[str, Any]] | None = None
    quality: dict[str, Any] | None = None
    proximity: list[dict[str, Any]] | None = None
    diagonal: bool | None = None
    fps: float | None = None
    estop: bool | None = None


class TargetUpdate(BaseModel):
    x: int | None = None
    y: int | None = None
    targets: list[list[int]] | None = None
    workers: list[list[int]] | None = None
    diagonal: bool | None = None
    optimize_order: bool | None = None
    append: bool = False


class StaticZonesUpdate(BaseModel):
    obstacles: list[list[int]] = Field(default_factory=list)
    mode: str = Field(default="replace", pattern="^(replace|add|remove|toggle)$")


class ConfigUpdate(BaseModel):
    diagonal: bool | None = None
    cost_mode: str | None = Field(default=None, pattern="^(exact|industrial)$")
    congestion_weight: float | None = Field(default=None, ge=0.0, le=50.0)
    turn_penalty: float | None = Field(default=None, ge=0.0, le=10.0)
    clearance_weight: float | None = Field(default=None, ge=0.0, le=10.0)
    clearance_radius: int | None = Field(default=None, ge=0, le=6)
    cell_size_m: float | None = Field(default=None, gt=0.05, le=50.0)
    worker_speed_mps: float | None = Field(default=None, gt=0.1, le=20.0)
    optimize_order: bool | None = None
    snap_start_if_blocked: bool | None = None
    decel_mps2: float | None = Field(default=None, gt=0.1, le=10.0)


class TaskCreate(BaseModel):
    kind: str = "pick"
    x: int | None = None
    y: int | None = None
    target: list[int] | None = None
    priority: int = 0
    robot_id: int | None = None
    payload_kg: float = 0.0
    label: str = ""


class SpeedUpdate(BaseModel):
    speed_mps: float = Field(gt=0.0, le=3.0)


class EstopUpdate(BaseModel):
    active: bool


class GridStateManager:
    def __init__(self, size: int = GRID_SIZE) -> None:
        self.size = size
        self.lock = asyncio.Lock()
        self.clients: set[WebSocket] = set()
        self.started_at = time.time()
        self.store = Store(DB_PATH)
        self.tasks = TaskQueue(self.store)
        self.diagnostics = Diagnostics()
        self.drive_cfg = DriveConfig()
        self.followers: dict[int, PurePursuit] = {}
        self.safety_monitor = SafetyMonitor(
            decel_mps2=float(os.getenv("DECEL_MPS2", "1.3")),
            min_clearance_cells=float(os.getenv("MIN_CLEARANCE_CELLS", "1.1")),
        )

        base_cfg = RouterConfig()
        saved_cfg = self.store.kv_get("config", {})
        if isinstance(saved_cfg, dict):
            for key in (
                "diagonal",
                "cost_mode",
                "congestion_weight",
                "turn_penalty",
                "clearance_weight",
                "clearance_radius",
                "cell_size_m",
                "worker_speed_mps",
                "optimize_order",
                "snap_start_if_blocked",
                "decel_mps2",
            ):
                if key in saved_cfg and hasattr(base_cfg, key):
                    setattr(base_cfg, key, getattr(base_cfg, key).__class__(saved_cfg[key]))
        self.config = base_cfg

        saved_zones = self.store.kv_get("static_obstacles", [])
        self.static_obstacles: set[tuple[int, int]] = set()
        for p in saved_zones or []:
            if len(p) >= 2:
                x, y = int(p[0]), int(p[1])
                if 0 <= x < self.size and 0 <= y < self.size:
                    self.static_obstacles.add((x, y))

        self.dynamic_obstacles: set[tuple[int, int]] = set()
        self.rho: list[list[float]] = [[0.0] * size for _ in range(size)]
        self.workers: list[list[int]] = [[0, 0]]
        self.robots: list[dict[str, Any]] = []
        self.targets: list[list[int]] = []
        self.quality: dict[str, Any] = {}
        self.proximity: list[dict[str, Any]] = []
        self.estop = bool(self.store.kv_get("estop", False))
        self.speed_limit_mps = float(self.store.kv_get("speed_limit", self.config.worker_speed_mps))
        self.commands: list[dict[str, Any]] = []
        self.safety: dict[str, Any] = {}
        self.events: list[dict[str, Any]] = self.store.event_list(100)
        self.last_safety_alerts: set[str] = set()
        self.primary_route: dict[str, Any] = self._blank_route("no_targets")
        self.fleet_routes: list[dict[str, Any]] = []

        self.updated_at = time.time()
        self.last_vision_at: float | None = None
        self.vision_fps: float = 0.0
        self.total_updates = 0
        self.unreachable_count = 0
        self.avg_solve_ms = 0.0
        self.history: deque[dict[str, Any]] = deque(maxlen=100)
        self.recompute_locked()

    def _blank_route(self, reason: str) -> dict[str, Any]:
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
            "compute_time_ms": 0.0,
        }

    def sanitize_pairs(self, values: list[list[int]] | None) -> list[list[int]]:
        out: list[list[int]] = []
        for p in values or []:
            if len(p) >= 2:
                try:
                    x, y = int(p[0]), int(p[1])
                except (TypeError, ValueError):
                    continue
                if 0 <= x < self.size and 0 <= y < self.size:
                    out.append([x, y])
        return out

    def sanitize_rho(self, values: list[list[float]] | None) -> list[list[float]]:
        r = [[0.0] * self.size for _ in range(self.size)]
        if values is None:
            return r
        for y in range(min(self.size, len(values))):
            row = values[y]
            for x in range(min(self.size, len(row))):
                try:
                    v = float(row[x])
                except (TypeError, ValueError):
                    v = 0.0
                if not math.isfinite(v):
                    v = 0.0
                r[y][x] = max(0.0, min(1.0, v))
        return r

    def sanitize_robots(self, values: list[dict[str, Any]] | None) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for item in values or []:
            if not isinstance(item, dict):
                continue
            cell = item.get("cell") or item.get("pos") or []
            if len(cell) < 2:
                continue
            try:
                x, y = int(cell[0]), int(cell[1])
            except (TypeError, ValueError):
                continue
            if not (0 <= x < self.size and 0 <= y < self.size):
                continue
            try:
                robot_id = int(item.get("robot_id", 0))
                heading = float(item.get("heading_deg", 0.0))
                speed = max(0.0, float(item.get("speed_mps", 0.0)))
                battery = max(0.0, min(100.0, float(item.get("battery_pct", 100.0))))
            except (TypeError, ValueError):
                continue
            out.append(
                {
                    "robot_id": robot_id,
                    "cell": [x, y],
                    "heading_deg": heading,
                    "speed_mps": speed,
                    "battery_pct": battery,
                    "source": str(item.get("source", "vision")),
                }
            )
        return out

    def merged_grid(self) -> list[list[int]]:
        g = [[0] * self.size for _ in range(self.size)]
        for x, y in self.dynamic_obstacles | self.static_obstacles:
            g[y][x] = 1
        return g

    def event_locked(self, severity: str, message: str, data: dict[str, Any] | None = None) -> None:
        item = {
            "id": int(time.time() * 1000),
            "ts": time.time(),
            "severity": severity,
            "message": message,
            "data": data or {},
        }
        self.events.insert(0, item)
        del self.events[200:]
        self.store.event_add(severity, message, data)

    def persist_config_locked(self) -> None:
        self.store.kv_set(
            "config",
            {
                "diagonal": self.config.diagonal,
                "cost_mode": self.config.cost_mode,
                "congestion_weight": self.config.congestion_weight,
                "turn_penalty": self.config.turn_penalty,
                "clearance_weight": self.config.clearance_weight,
                "clearance_radius": self.config.clearance_radius,
                "cell_size_m": self.config.cell_size_m,
                "worker_speed_mps": self.config.worker_speed_mps,
                "optimize_order": self.config.optimize_order,
                "snap_start_if_blocked": self.config.snap_start_if_blocked,
                "decel_mps2": self.config.decel_mps2,
            },
        )

    def persist_zones_locked(self) -> None:
        self.store.kv_set("static_obstacles", [[x, y] for x, y in sorted(self.static_obstacles)])

    def route_workers_locked(self) -> list[list[int]]:
        if self.robots:
            return [r["cell"] for r in self.robots]
        return self.workers if self.workers else [[0, 0]]

    def recompute_locked(self) -> None:
        grid = self.merged_grid()
        worker_list = self.route_workers_locked()
        if len(worker_list) > 1:
            self.fleet_routes = route_fleet(grid, self.rho, worker_list, self.targets, self.config)
            self.primary_route = self.fleet_routes[0]
        else:
            r = route(grid, self.rho, worker_list[0], self.targets, self.config.diagonal, self.config)
            r["worker_id"] = 0
            self.primary_route = r
            self.fleet_routes = [r]

        convert = float(self.primary_route.get("compute_time_ms", 0.0))
        self.diagnostics.record("solve_ms", convert)
        if self.vision_fps > 0.0:
            self.diagnostics.record("vision_fps", self.vision_fps)
        self.safety_monitor.decel_mps2 = max(0.1, self.config.decel_mps2)

        report = self.safety_monitor.assess(
            sorted(self.dynamic_obstacles | self.static_obstacles),
            self.rho,
            worker_list,
            self.speed_limit_mps,
            self.size,
            self.config.cell_size_m,
            self.estop,
            self.quality or None,
            self.proximity,
        )
        self.safety = report.as_dict()
        alerts = set(report.alerts)
        if alerts != self.last_safety_alerts:
            for alert in sorted(alerts - self.last_safety_alerts):
                self.event_locked("warning", alert, {"speed": report.recommended_speed_mps})
            self.last_safety_alerts = alerts

        self.commands = []
        speeds: list[float] = []
        for idx, worker in enumerate(worker_list):
            robot_pose = None
            if idx < len(self.robots):
                item = self.robots[idx]
                robot_pose = RobotPose(
                    int(item["robot_id"]),
                    float(item["cell"][0]) * self.config.cell_size_m,
                    float(item["cell"][1]) * self.config.cell_size_m,
                    math.radians(float(item["heading_deg"])),
                    float(item["speed_mps"]),
                    float(item["battery_pct"]),
                    str(item.get("source", "vision")),
                )
            else:
                robot_pose = RobotPose(idx, float(worker[0]), float(worker[1]), 0.0, 0.0, 100.0, "grid")
            rpath = self.fleet_routes[idx].get("path", []) if idx < len(self.fleet_routes) else []
            follower = self.followers.setdefault(idx, PurePursuit(self.drive_cfg))
            command = follower.command(
                robot_pose,
                rpath,
                self.config.cell_size_m,
                0.05,
                0.0 if self.safety["estop"] else float(self.safety["recommended_speed_mps"]),
            )
            cmd = motion_dict(command)
            cmd["battery_drain_pct"] = battery_drain(
                self.fleet_routes[idx].get("distance", 0.0) if idx < len(self.fleet_routes) else 0.0,
                self.fleet_routes[idx].get("turns", 0) if idx < len(self.fleet_routes) else 0,
                next(
                    (
                        float(t.get("payload_kg", 0.0))
                        for t in self.tasks.active()
                        if t.get("robot_id") in (None, idx)
                    ),
                    0.0,
                ),
            )
            self.commands.append(cmd)
            speeds.append(cmd["linear_mps"])

        self.commands.append(
            {
                "robot_id": "fleet",
                "utilization": fleet_utilization(speeds, self.config.worker_speed_mps),
            }
        )

        self.updated_at = time.time()
        self.total_updates += 1
        solve_ms = float(self.primary_route.get("compute_time_ms", 0.0))
        self.avg_solve_ms = (
            solve_ms if self.total_updates == 1 else (0.9 * self.avg_solve_ms + 0.1 * solve_ms)
        )
        if self.targets and not self.primary_route.get("reachable", False):
            self.unreachable_count += 1

        self.history.append(
            {
                "ts": round(self.updated_at, 3),
                "workers": len(worker_list),
                "obstacles": len(self.dynamic_obstacles | self.static_obstacles),
                "targets": len(self.targets),
                "reachable": self.primary_route.get("reachable", False),
                "reason": self.primary_route.get("reason", "unknown"),
                "distance": self.primary_route.get("distance", 0.0),
                "eta_sec": self.primary_route.get("estimated_time_sec", 0.0),
                "solve_ms": solve_ms,
                "estop": self.safety["estop"],
            }
        )

    def activate_next_task_locked(self) -> None:
        pending = self.tasks.next()
        if pending is None:
            return
        target = pending.get("target")
        if not target:
            return
        self.tasks.assign(str(pending["id"]), 0)
        self.targets = [[int(target[0]), int(target[1])]]
        self.event_locked(
            "info",
            "TASK_DISPATCHED",
            {"id": pending["id"], "kind": pending.get("kind"), "target": target},
        )

    def snapshot_locked(self) -> dict[str, Any]:
        grid = self.merged_grid()
        all_obs = sorted(self.dynamic_obstacles | self.static_obstacles)
        vision_online = (
            self.last_vision_at is not None
            and (time.time() - self.last_vision_at) <= VISION_STALE_SEC
        )
        pr = self.primary_route
        return {
            "grid_size": self.size,
            "grid": grid,
            "rho": self.rho,
            "obstacles": [[x, y] for x, y in all_obs],
            "dynamic_obstacles": [[x, y] for x, y in sorted(self.dynamic_obstacles)],
            "static_obstacles": [[x, y] for x, y in sorted(self.static_obstacles)],
            "workers": [list(w) for w in self.workers],
            "robots": [dict(r) for r in self.robots],
            "targets": [list(t) for t in self.targets],
            "path": pr.get("path", []),
            "steps": pr.get("steps", []),
            "instructions": pr.get("instructions", []),
            "cost": pr.get("cost", 0.0),
            "distance": pr.get("distance", 0.0),
            "estimated_time_sec": pr.get("estimated_time_sec", 0.0),
            "turns": pr.get("turns", 0),
            "avg_congestion": pr.get("avg_congestion", 0.0),
            "max_congestion": pr.get("max_congestion", 0.0),
            "reachable": pr.get("reachable", False),
            "reason": pr.get("reason", "no_targets"),
            "snapped_start": pr.get("snapped_start"),
            "fleet_routes": self.fleet_routes,
            "commands": [dict(c) for c in self.commands],
            "tasks": self.tasks.list(),
            "task_stats": self.tasks.stats(),
            "safety": dict(self.safety),
            "quality": dict(self.quality),
            "proximity": [dict(p) for p in self.proximity],
            "estop": self.estop,
            "speed_limit_mps": self.speed_limit_mps,
            "diagonal": self.config.diagonal,
            "config": {
                "diagonal": self.config.diagonal,
                "cost_mode": self.config.cost_mode,
                "congestion_weight": self.config.congestion_weight,
                "turn_penalty": self.config.turn_penalty,
                "clearance_weight": self.config.clearance_weight,
                "clearance_radius": self.config.clearance_radius,
                "cell_size_m": self.config.cell_size_m,
                "worker_speed_mps": self.config.worker_speed_mps,
                "optimize_order": self.config.optimize_order,
                "snap_start_if_blocked": self.config.snap_start_if_blocked,
                "decel_mps2": self.config.decel_mps2,
            },
            "vision": {
                "online": vision_online,
                "fps": round(self.vision_fps, 1),
                "last_update": self.last_vision_at,
            },
            "metrics": {
                "solve_ms": pr.get("compute_time_ms", 0.0),
                "avg_solve_ms": round(self.avg_solve_ms, 3),
                "nodes_expanded": pr.get("nodes_expanded", 0),
                "clients": len(self.clients),
                "total_updates": self.total_updates,
                "unreachable_events": self.unreachable_count,
            },
            "diagnostics": self.diagnostics.snapshot(),
            "events": [dict(e) for e in self.events[:100]],
            "updated_at": self.updated_at,
        }

    async def broadcast(self, message: dict[str, Any]) -> None:
        if not self.clients:
            return
        items = list(self.clients)

        async def safe_send(ws: WebSocket) -> WebSocket | None:
            try:
                await asyncio.wait_for(ws.send_json(message), timeout=1.5)
                return None
            except Exception:
                return ws

        dead = await asyncio.gather(*(safe_send(c) for c in items))
        for ws in dead:
            if ws is not None:
                self.clients.discard(ws)


state_mgr = GridStateManager(GRID_SIZE)


@asynccontextmanager
async def lifespan(_app: FastAPI):
    logger.info("MAVRYKS Backend started (grid=%dx%d)", state_mgr.size, state_mgr.size)
    yield
    for ws in list(state_mgr.clients):
        try:
            await ws.close()
        except Exception:
            pass
    state_mgr.clients.clear()
    logger.info("MAVRYKS Backend stopped cleanly")


app = FastAPI(
    title="MAVRYKS Physical AI Warehouse API",
    version="3.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/health")
async def health() -> dict[str, Any]:
    now = time.time()
    vision_age = None if state_mgr.last_vision_at is None else round(now - state_mgr.last_vision_at, 2)
    async with state_mgr.lock:
        return {
            "status": "ok",
            "uptime_sec": round(now - state_mgr.started_at, 2),
            "grid_size": state_mgr.size,
            "ws_clients": len(state_mgr.clients),
            "vision_online": vision_age is not None and vision_age <= VISION_STALE_SEC,
            "vision_age_sec": vision_age,
            "estop": state_mgr.estop,
            "pending_tasks": len(state_mgr.tasks.list("pending")),
            "safety_estop": bool(state_mgr.safety.get("estop", False)),
        }


@app.get("/api/metrics")
async def get_metrics() -> dict[str, Any]:
    async with state_mgr.lock:
        snap = state_mgr.snapshot_locked()
        return {
            **snap["metrics"],
            **snap["diagnostics"],
            "active_workers": len(state_mgr.workers),
            "active_targets": len(state_mgr.targets),
            "dynamic_obstacles": len(state_mgr.dynamic_obstacles),
            "static_obstacles": len(state_mgr.static_obstacles),
            "vision_fps": round(state_mgr.vision_fps, 1),
            "ws_clients": len(state_mgr.clients),
        }


@app.get("/api/history")
async def get_history(limit: int = 50) -> list[dict[str, Any]]:
    limit = max(1, min(100, limit))
    async with state_mgr.lock:
        return list(state_mgr.history)[-limit:]


@app.get("/api/state")
async def get_state() -> dict[str, Any]:
    async with state_mgr.lock:
        return state_mgr.snapshot_locked()


@app.post("/api/vision/update")
async def vision_update(payload: VisionUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        state_mgr.dynamic_obstacles = {
            (p[0], p[1]) for p in state_mgr.sanitize_pairs(payload.obstacles)
        }
        state_mgr.rho = state_mgr.sanitize_rho(payload.rho)
        if payload.workers is not None:
            parsed_workers = state_mgr.sanitize_pairs(payload.workers)
            state_mgr.workers = parsed_workers if parsed_workers else [[0, 0]]
        if payload.targets is not None:
            state_mgr.targets = state_mgr.sanitize_pairs(payload.targets)
        if payload.robots is not None:
            state_mgr.robots = state_mgr.sanitize_robots(payload.robots)
        if payload.quality is not None:
            state_mgr.quality = dict(payload.quality)
        if payload.proximity is not None:
            state_mgr.proximity = list(payload.proximity)
        if payload.diagonal is not None:
            state_mgr.config.diagonal = payload.diagonal
        if payload.fps is not None and math.isfinite(payload.fps):
            state_mgr.vision_fps = max(0.0, float(payload.fps))
            state_mgr.diagnostics.record("vision_fps", state_mgr.vision_fps)
        if payload.estop is not None and bool(payload.estop) != state_mgr.estop:
            state_mgr.estop = bool(payload.estop)
            state_mgr.store.kv_set("estop", state_mgr.estop)
            if state_mgr.estop:
                state_mgr.diagnostics.increment("estops")
            state_mgr.event_locked(
                "critical" if state_mgr.estop else "info",
                "ESTOP_ACTIVE" if state_mgr.estop else "ESTOP_CLEARED",
                {"source": "vision"},
            )
        state_mgr.last_vision_at = time.time()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/target")
async def set_target(payload: TargetUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        if payload.targets is not None:
            new_targets = state_mgr.sanitize_pairs(payload.targets)
            state_mgr.targets = (state_mgr.targets + new_targets) if payload.append else new_targets
        elif payload.x is not None and payload.y is not None:
            if not (0 <= payload.x < state_mgr.size and 0 <= payload.y < state_mgr.size):
                raise HTTPException(status_code=422, detail="Target coordinates out of bounds")
            pt = [payload.x, payload.y]
            if payload.append:
                if pt not in state_mgr.targets:
                    state_mgr.targets.append(pt)
            else:
                state_mgr.targets = [pt]
        if payload.workers is not None:
            parsed = state_mgr.sanitize_pairs(payload.workers)
            state_mgr.workers = parsed if parsed else [[0, 0]]
        if payload.diagonal is not None:
            state_mgr.config.diagonal = payload.diagonal
        if payload.optimize_order is not None:
            state_mgr.config.optimize_order = payload.optimize_order
        state_mgr.persist_config_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/zones/static")
async def update_static_zones(payload: StaticZonesUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        pts = {(p[0], p[1]) for p in state_mgr.sanitize_pairs(payload.obstacles)}
        if payload.mode == "replace":
            state_mgr.static_obstacles = pts
        elif payload.mode == "add":
            state_mgr.static_obstacles |= pts
        elif payload.mode == "remove":
            state_mgr.static_obstacles -= pts
        elif payload.mode == "toggle":
            for pt in pts:
                if pt in state_mgr.static_obstacles:
                    state_mgr.static_obstacles.remove(pt)
                else:
                    state_mgr.static_obstacles.add(pt)
        state_mgr.persist_zones_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.delete("/api/zones/static")
async def clear_static_zones() -> dict[str, Any]:
    async with state_mgr.lock:
        state_mgr.static_obstacles.clear()
        state_mgr.persist_zones_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/config")
async def update_config(payload: ConfigUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        for field_name, value in payload.model_dump(exclude_none=True).items():
            setattr(state_mgr.config, field_name, value)
        state_mgr.persist_config_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/reset")
async def reset_state() -> dict[str, Any]:
    async with state_mgr.lock:
        state_mgr.dynamic_obstacles.clear()
        state_mgr.targets.clear()
        state_mgr.robots = []
        state_mgr.rho = [[0.0] * state_mgr.size for _ in range(state_mgr.size)]
        state_mgr.tasks.clear()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.get("/api/tasks")
async def get_tasks() -> dict[str, Any]:
    async with state_mgr.lock:
        return {"tasks": state_mgr.tasks.list(), "stats": state_mgr.tasks.stats()}


@app.post("/api/tasks")
async def add_task(payload: TaskCreate) -> dict[str, Any]:
    if payload.target is not None and len(payload.target) >= 2:
        target = [int(payload.target[0]), int(payload.target[1])]
    elif payload.x is not None and payload.y is not None:
        target = [int(payload.x), int(payload.y)]
    else:
        raise HTTPException(status_code=422, detail="Task target is required")
    if not (0 <= target[0] < state_mgr.size and 0 <= target[1] < state_mgr.size):
        raise HTTPException(status_code=422, detail="Task target out of bounds")
    async with state_mgr.lock:
        task = state_mgr.tasks.add(
            payload.kind,
            target,
            payload.priority,
            payload.robot_id,
            payload.payload_kg,
            payload.label,
        )
        state_mgr.event_locked(
            "info",
            "TASK_CREATED",
            {"id": task["id"], "kind": task["kind"], "target": target},
        )
        if not state_mgr.tasks.active():
            state_mgr.activate_next_task_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return {"task": task, "state": snap}


@app.post("/api/tasks/{task_id}/start")
async def start_task(task_id: str) -> dict[str, Any]:
    async with state_mgr.lock:
        task = state_mgr.tasks.get(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="Task not found")
        task = state_mgr.tasks.start(task_id, int(task.get("robot_id") or 0))
        if task and task.get("target"):
            state_mgr.targets = [list(task["target"])]
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return {"task": task, "state": snap}


@app.post("/api/tasks/{task_id}/complete")
async def complete_task(task_id: str) -> dict[str, Any]:
    async with state_mgr.lock:
        task = state_mgr.tasks.complete(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="Task not found")
        state_mgr.diagnostics.increment("tasks_done")
        state_mgr.event_locked("info", "TASK_DONE", {"id": task_id})
        state_mgr.targets = []
        state_mgr.activate_next_task_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return {"task": task, "state": snap}


@app.post("/api/tasks/{task_id}/fail")
async def fail_task(task_id: str, reason: str = "") -> dict[str, Any]:
    async with state_mgr.lock:
        task = state_mgr.tasks.fail(task_id, reason)
        if task is None:
            raise HTTPException(status_code=404, detail="Task not found")
        state_mgr.diagnostics.increment("tasks_failed")
        state_mgr.event_locked("error", "TASK_FAILED", {"id": task_id, "reason": reason})
        state_mgr.targets = []
        state_mgr.activate_next_task_locked()
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return {"task": task, "state": snap}


@app.post("/api/tasks/{task_id}/cancel")
async def cancel_task(task_id: str) -> dict[str, Any]:
    async with state_mgr.lock:
        task = state_mgr.tasks.cancel(task_id)
        if task is None:
            raise HTTPException(status_code=404, detail="Task not found")
        state_mgr.event_locked("warning", "TASK_CANCELLED", {"id": task_id})
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return {"task": task, "state": snap}


@app.delete("/api/tasks/{task_id}")
async def remove_task(task_id: str) -> dict[str, Any]:
    async with state_mgr.lock:
        if not state_mgr.tasks.remove(task_id):
            raise HTTPException(status_code=404, detail="Task not found")
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/estop")
async def set_estop(payload: EstopUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        if bool(payload.active) != state_mgr.estop:
            state_mgr.estop = bool(payload.active)
            state_mgr.store.kv_set("estop", state_mgr.estop)
            if state_mgr.estop:
                state_mgr.diagnostics.increment("estops")
            state_mgr.event_locked(
                "critical" if state_mgr.estop else "info",
                "ESTOP_ACTIVE" if state_mgr.estop else "ESTOP_CLEARED",
                {"source": "operator"},
            )
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.post("/api/speed")
async def set_speed(payload: SpeedUpdate) -> dict[str, Any]:
    async with state_mgr.lock:
        state_mgr.speed_limit_mps = float(payload.speed_mps)
        state_mgr.store.kv_set("speed_limit", state_mgr.speed_limit_mps)
        state_mgr.recompute_locked()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.get("/api/events")
async def get_events(limit: int = 100) -> list[dict[str, Any]]:
    async with state_mgr.lock:
        return [dict(e) for e in state_mgr.events[: max(1, min(200, limit))]]


@app.delete("/api/events")
async def clear_events() -> dict[str, Any]:
    async with state_mgr.lock:
        state_mgr.events = []
        state_mgr.store.event_clear()
        snap = state_mgr.snapshot_locked()
    await state_mgr.broadcast(snap)
    return snap


@app.get("/api/system")
async def get_system() -> dict[str, Any]:
    async with state_mgr.lock:
        return {
            "version": "3.0.0",
            "db_path": DB_PATH,
            "grid_size": state_mgr.size,
            "diagnostics": state_mgr.diagnostics.snapshot(),
            "task_stats": state_mgr.tasks.stats(),
            "routes": len(state_mgr.fleet_routes),
            "commands": len(state_mgr.commands),
        }


@app.websocket("/ws/live")
async def ws_live(ws: WebSocket) -> None:
    await ws.accept()
    state_mgr.clients.add(ws)
    try:
        async with state_mgr.lock:
            init_snap = state_mgr.snapshot_locked()
        await ws.send_json(init_snap)
        while True:
            raw = await ws.receive_json()
            if not isinstance(raw, dict):
                continue
            cmd = raw.get("cmd")
            if cmd == "ping":
                await ws.send_json({"type": "pong", "ts": time.time()})
            elif cmd == "get_state":
                async with state_mgr.lock:
                    await ws.send_json(state_mgr.snapshot_locked())
            elif cmd == "set_target":
                x, y = raw.get("x"), raw.get("y")
                if x is not None and y is not None:
                    await set_target(TargetUpdate(x=int(x), y=int(y), append=bool(raw.get("append", False))))
            elif cmd == "toggle_static":
                x, y = raw.get("x"), raw.get("y")
                if x is not None and y is not None:
                    await update_static_zones(
                        StaticZonesUpdate(obstacles=[[int(x), int(y)]], mode="toggle")
                    )
            elif cmd == "add_task":
                x, y = raw.get("x"), raw.get("y")
                if x is not None and y is not None:
                    await add_task(
                        TaskCreate(
                            kind=str(raw.get("kind", "pick")),
                            x=int(x),
                            y=int(y),
                            priority=int(raw.get("priority", 0)),
                            payload_kg=float(raw.get("payload_kg", 0.0)),
                        )
                    )
            elif cmd == "complete_task":
                task_id = str(raw.get("id", ""))
                if task_id:
                    await complete_task(task_id)
            elif cmd == "cancel_task":
                task_id = str(raw.get("id", ""))
                if task_id:
                    await cancel_task(task_id)
            elif cmd == "estop":
                await set_estop(EstopUpdate(active=bool(raw.get("active", True))))
            elif cmd == "speed":
                await set_speed(SpeedUpdate(speed_mps=float(raw.get("speed_mps", 1.4))))
            elif cmd == "config":
                keys = set(ConfigUpdate.model_fields)
                data = {k: v for k, v in raw.items() if k in keys}
                if data:
                    await update_config(ConfigUpdate(**data))
            elif cmd == "clear_targets":
                await set_target(TargetUpdate(targets=[]))
            elif cmd == "clear_zones":
                await clear_static_zones()
    except (WebSocketDisconnect, RuntimeError):
        pass
    except Exception as exc:
        logger.debug("WebSocket closed with error: %s", exc)
    finally:
        state_mgr.clients.discard(ws)


@app.get("/", response_class=HTMLResponse)
async def dashboard() -> HTMLResponse:
    content = Path(__file__).with_name("dashboard.html").read_text(encoding="utf-8")
    return HTMLResponse(content=content)
