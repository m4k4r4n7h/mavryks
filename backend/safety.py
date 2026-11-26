from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence


@dataclass(slots=True)
class SafetyReport:
    estop: bool
    recommended_speed_mps: float
    commanded_speed_mps: float
    min_clearance_m: float
    brake_distance_m: float
    congestion_limit: float
    alerts: list[str]

    def as_dict(self) -> dict[str, Any]:
        return {
            "estop": self.estop,
            "recommended_speed_mps": round(self.recommended_speed_mps, 4),
            "commanded_speed_mps": round(self.commanded_speed_mps, 4),
            "min_clearance_m": round(self.min_clearance_m, 4),
            "brake_distance_m": round(self.brake_distance_m, 4),
            "congestion_limit": round(self.congestion_limit, 4),
            "alerts": list(self.alerts),
        }


class SafetyMonitor:
    def __init__(self, decel_mps2: float = 1.3, min_clearance_cells: float = 1.1) -> None:
        self.decel_mps2 = max(0.1, float(decel_mps2))
        self.min_clearance_cells = max(0.5, float(min_clearance_cells))

    def assess(
        self,
        obstacles: Sequence[Sequence[int]],
        rho: Sequence[Sequence[float]],
        workers: Sequence[Sequence[int]],
        commanded_speed_mps: float,
        grid_size: int,
        cell_size_m: float,
        estop: bool,
        quality: dict[str, Any] | None = None,
        proximity: Sequence[dict[str, Any]] | None = None,
    ) -> SafetyReport:
        alerts: list[str] = []
        n = max(1, int(grid_size))
        cell = max(0.01, float(cell_size_m))
        points = [(int(p[0]), int(p[1])) for p in workers if len(p) >= 2]
        obs = {(int(p[0]), int(p[1])) for p in obstacles if len(p) >= 2}

        min_dist = math.hypot(n, n)
        rho_peak = 0.0
        for wx, wy in points:
            wx = max(0, min(n - 1, wx))
            wy = max(0, min(n - 1, wy))
            rho_peak = max(rho_peak, float(rho[wy][wx]) if wy < len(rho) and wx < len(rho[wy]) else 0.0)
            for ox, oy in obs:
                d = math.hypot(ox - wx, oy - wy)
                if d < min_dist:
                    min_dist = d

        clearance_m = min_dist * cell
        speed = max(0.0, min(3.0, float(commanded_speed_mps)))
        brake = speed * speed / (2.0 * self.decel_mps2)
        limit = math.sqrt(max(0.0, clearance_m - 0.12) * 2.0 * self.decel_mps2)
        congestion_limit = 1.0 / (1.0 + 3.0 * rho_peak * rho_peak)
        recommended = min(speed, limit, speed * congestion_limit)
        active_estop = bool(estop)

        if min_dist <= self.min_clearance_cells:
            active_estop = True
            alerts.append("OBSTACLE_CLEARANCE")
        if brake >= max(0.0, clearance_m - 0.05) and speed > 0.01:
            active_estop = True
            alerts.append("BRAKE_DISTANCE")
        if quality and float(quality.get("score", 1.0)) < 0.35:
            recommended = min(recommended, 0.25)
            alerts.append("VISION_QUALITY_LOW")
        for item in proximity or []:
            level = str(item.get("level", ""))
            if level == "critical":
                active_estop = True
            if level in ("warning", "critical"):
                alerts.append(str(item.get("reason", "PROXIMITY")))

        if active_estop:
            recommended = 0.0
        return SafetyReport(
            active_estop,
            recommended,
            recommended,
            clearance_m,
            brake,
            congestion_limit,
            alerts,
        )
