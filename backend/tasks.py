from __future__ import annotations

import time
import uuid
from typing import Any

from store import Store

KINDS = ("pick", "place", "dock", "inspect", "charge")
OPEN_STATUSES = ("pending", "assigned", "active")


class TaskQueue:
    def __init__(self, store: Store) -> None:
        self.store = store
        self.tasks = store.task_list()

    def _save(self, task: dict[str, Any]) -> None:
        for idx, item in enumerate(self.tasks):
            if item.get("id") == task.get("id"):
                self.tasks[idx] = task
                break
        else:
            self.tasks.append(task)
        self.store.task_add(task)

    def add(
        self,
        kind: str,
        target: list[int],
        priority: int = 0,
        robot_id: int | None = None,
        payload_kg: float = 0.0,
        label: str = "",
    ) -> dict[str, Any]:
        task = {
            "id": uuid.uuid4().hex[:10],
            "kind": kind if kind in KINDS else "pick",
            "target": [int(target[0]), int(target[1])],
            "priority": max(-20, min(20, int(priority))),
            "robot_id": robot_id,
            "payload_kg": max(0.0, float(payload_kg)),
            "label": label[:80],
            "status": "pending",
            "created_at": time.time(),
            "started_at": None,
            "finished_at": None,
            "reason": "",
        }
        self._save(task)
        return task

    def get(self, task_id: str) -> dict[str, Any] | None:
        for task in self.tasks:
            if task.get("id") == task_id:
                return task
        return None

    def start(self, task_id: str, robot_id: int | None = None) -> dict[str, Any] | None:
        task = self.get(task_id)
        if task is None:
            return None
        task["status"] = "active"
        task["robot_id"] = robot_id if robot_id is not None else task.get("robot_id")
        task["started_at"] = time.time()
        task["finished_at"] = None
        task["reason"] = ""
        self._save(task)
        return task

    def assign(self, task_id: str, robot_id: int) -> dict[str, Any] | None:
        task = self.get(task_id)
        if task is None:
            return None
        task["status"] = "assigned"
        task["robot_id"] = int(robot_id)
        self._save(task)
        return task

    def complete(self, task_id: str) -> dict[str, Any] | None:
        task = self.get(task_id)
        if task is None:
            return None
        task["status"] = "done"
        task["finished_at"] = time.time()
        task["reason"] = ""
        self._save(task)
        return task

    def fail(self, task_id: str, reason: str = "") -> dict[str, Any] | None:
        task = self.get(task_id)
        if task is None:
            return None
        task["status"] = "failed"
        task["finished_at"] = time.time()
        task["reason"] = reason[:120]
        self._save(task)
        return task

    def cancel(self, task_id: str) -> dict[str, Any] | None:
        task = self.get(task_id)
        if task is None:
            return None
        task["status"] = "cancelled"
        task["finished_at"] = time.time()
        task["reason"] = "cancelled"
        self._save(task)
        return task

    def remove(self, task_id: str) -> bool:
        task = self.get(task_id)
        if task is None:
            return False
        self.tasks = [item for item in self.tasks if item.get("id") != task_id]
        self.store.task_delete(task_id)
        return True

    def clear(self) -> None:
        self.tasks = []
        self.store.task_clear()

    def list(self, status: str | None = None) -> list[dict[str, Any]]:
        if status is None:
            return [dict(task) for task in self.tasks]
        return [dict(task) for task in self.tasks if task.get("status") == status]

    def next(self, robot_id: int | None = None) -> dict[str, Any] | None:
        candidates = [
            task
            for task in self.tasks
            if task.get("status") == "pending"
            and (robot_id is None or task.get("robot_id") in (None, robot_id))
        ]
        if not candidates:
            return None
        candidates.sort(key=lambda t: (-int(t.get("priority", 0)), float(t.get("created_at", 0.0))))
        return candidates[0]

    def active(self) -> list[dict[str, Any]]:
        return [dict(task) for task in self.tasks if task.get("status") in OPEN_STATUSES]

    def stats(self) -> dict[str, int]:
        out = {
            "pending": 0,
            "assigned": 0,
            "active": 0,
            "done": 0,
            "failed": 0,
            "cancelled": 0,
        }
        for task in self.tasks:
            key = str(task.get("status", "pending"))
            if key in out:
                out[key] += 1
        return out
