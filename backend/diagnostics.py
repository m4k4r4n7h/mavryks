from __future__ import annotations

import math
import time
from collections import deque
from typing import Any


class Diagnostics:
    def __init__(self, maxlen: int = 600) -> None:
        self.started_at = time.time()
        self.solve_ms: deque[float] = deque(maxlen=maxlen)
        self.vision_fps: deque[float] = deque(maxlen=maxlen)
        self.command_ms: deque[float] = deque(maxlen=maxlen)
        self.estops = 0
        self.tasks_done = 0
        self.tasks_failed = 0
        self.near_misses = 0

    def record(self, key: str, value: float) -> None:
        if not math.isfinite(float(value)):
            return
        if key == "solve_ms":
            self.solve_ms.append(float(value))
        elif key == "vision_fps":
            self.vision_fps.append(float(value))
        elif key == "command_ms":
            self.command_ms.append(float(value))

    def increment(self, key: str) -> None:
        if key == "estops":
            self.estops += 1
        elif key == "tasks_done":
            self.tasks_done += 1
        elif key == "tasks_failed":
            self.tasks_failed += 1
        elif key == "near_misses":
            self.near_misses += 1

    @staticmethod
    def _avg(values: deque[float]) -> float:
        return round(sum(values) / len(values), 3) if values else 0.0

    def snapshot(self) -> dict[str, Any]:
        return {
            "uptime_sec": round(time.time() - self.started_at, 2),
            "avg_solve_ms": self._avg(self.solve_ms),
            "max_solve_ms": round(max(self.solve_ms), 3) if self.solve_ms else 0.0,
            "avg_vision_fps": self._avg(self.vision_fps),
            "avg_command_ms": self._avg(self.command_ms),
            "estops": self.estops,
            "tasks_done": self.tasks_done,
            "tasks_failed": self.tasks_failed,
            "near_misses": self.near_misses,
        }
