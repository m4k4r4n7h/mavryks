from __future__ import annotations

import math
from typing import Any, Sequence


class ProximityMonitor:
    def __init__(self, warning_cells: float = 2.5, critical_cells: float = 1.2) -> None:
        self.warning_cells = float(warning_cells)
        self.critical_cells = float(critical_cells)

    def assess(
        self,
        person_points: Sequence[Sequence[float]],
        robots: Sequence[dict[str, Any]],
        planned_path: Sequence[Sequence[int]],
        cell_size_m: float,
    ) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        if not person_points:
            return out
        cell = max(0.01, float(cell_size_m))
        path = [(float(p[0]), float(p[1])) for p in planned_path]
        for item in robots:
            rcell = item.get("cell") or [0, 0]
            rx, ry = float(rcell[0]), float(rcell[1])
            best = math.inf
            for p in person_points:
                dx, dy = float(p[0]) - rx, float(p[1]) - ry
                d = math.hypot(dx, dy)
                if d < best:
                    best = d
            if best <= self.critical_cells:
                level = "critical"
                reason = "PERSON_ROBOT_CRITICAL"
            elif best <= self.warning_cells:
                level = "warning"
                reason = "PERSON_ROBOT_NEAR"
            else:
                continue
            out.append(
                {
                    "level": level,
                    "reason": reason,
                    "robot_id": item.get("robot_id", 0),
                    "distance_m": round(best * cell, 3),
                    "path_cells": len(path),
                }
            )
        return out
