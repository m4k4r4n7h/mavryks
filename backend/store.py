from __future__ import annotations

import json
import os
import sqlite3
import threading
import time
from typing import Any


class Store:
    def __init__(self, path: str) -> None:
        self.path = os.path.abspath(path)
        self.lock = threading.RLock()
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        with self.lock:
            conn = self._connect()
            conn.execute("CREATE TABLE IF NOT EXISTS kv (key TEXT PRIMARY KEY, value TEXT NOT NULL)")
            conn.execute(
                "CREATE TABLE IF NOT EXISTS tasks (id TEXT PRIMARY KEY, status TEXT NOT NULL, created_at REAL NOT NULL, data TEXT NOT NULL)"
            )
            conn.execute(
                "CREATE TABLE IF NOT EXISTS events (id INTEGER PRIMARY KEY AUTOINCREMENT, ts REAL NOT NULL, severity TEXT NOT NULL, message TEXT NOT NULL, data TEXT NOT NULL)"
            )
            conn.commit()
            conn.close()

    def _connect(self) -> sqlite3.Connection:
        return sqlite3.connect(self.path, timeout=5.0, check_same_thread=False)

    def kv_get(self, key: str, default: Any = None) -> Any:
        with self.lock:
            conn = self._connect()
            row = conn.execute("SELECT value FROM kv WHERE key=?", (key,)).fetchone()
            conn.close()
        if row is None:
            return default
        try:
            return json.loads(row[0])
        except (TypeError, ValueError):
            return default

    def kv_set(self, key: str, value: Any) -> None:
        raw = json.dumps(value, separators=(",", ":"), sort_keys=True)
        with self.lock:
            conn = self._connect()
            conn.execute(
                "INSERT INTO kv(key,value) VALUES(?,?) ON CONFLICT(key) DO UPDATE SET value=excluded.value",
                (key, raw),
            )
            conn.commit()
            conn.close()

    def task_add(self, task: dict[str, Any]) -> None:
        raw = json.dumps(task, separators=(",", ":"), sort_keys=True)
        with self.lock:
            conn = self._connect()
            conn.execute(
                "INSERT INTO tasks(id,status,created_at,data) VALUES(?,?,?,?) ON CONFLICT(id) DO UPDATE SET status=excluded.status, data=excluded.data",
                (str(task["id"]), str(task.get("status", "pending")), float(task.get("created_at", time.time())), raw),
            )
            conn.commit()
            conn.close()

    def task_update(self, task_id: str, task: dict[str, Any]) -> None:
        self.task_add(task)

    def task_delete(self, task_id: str) -> None:
        with self.lock:
            conn = self._connect()
            conn.execute("DELETE FROM tasks WHERE id=?", (str(task_id),))
            conn.commit()
            conn.close()

    def task_clear(self) -> None:
        with self.lock:
            conn = self._connect()
            conn.execute("DELETE FROM tasks")
            conn.commit()
            conn.close()

    def task_list(self) -> list[dict[str, Any]]:
        with self.lock:
            conn = self._connect()
            rows = conn.execute("SELECT data FROM tasks ORDER BY created_at ASC, id ASC").fetchall()
            conn.close()
        out: list[dict[str, Any]] = []
        for row in rows:
            try:
                item = json.loads(row[0])
                if isinstance(item, dict):
                    out.append(item)
            except (TypeError, ValueError):
                continue
        return out

    def event_add(self, severity: str, message: str, data: dict[str, Any] | None = None) -> None:
        raw = json.dumps(data or {}, separators=(",", ":"), sort_keys=True)
        with self.lock:
            conn = self._connect()
            conn.execute(
                "INSERT INTO events(ts,severity,message,data) VALUES(?,?,?,?)",
                (time.time(), str(severity), str(message), raw),
            )
            conn.commit()
            conn.close()

    def event_list(self, limit: int = 100) -> list[dict[str, Any]]:
        limit = max(1, min(500, int(limit)))
        with self.lock:
            conn = self._connect()
            rows = conn.execute(
                "SELECT id,ts,severity,message,data FROM events ORDER BY id DESC LIMIT ?",
                (limit,),
            ).fetchall()
            conn.close()
        out: list[dict[str, Any]] = []
        for row in rows:
            try:
                payload = json.loads(row[4])
            except (TypeError, ValueError):
                payload = {}
            out.append(
                {
                    "id": int(row[0]),
                    "ts": float(row[1]),
                    "severity": str(row[2]),
                    "message": str(row[3]),
                    "data": payload,
                }
            )
        return out

    def event_clear(self) -> None:
        with self.lock:
            conn = self._connect()
            conn.execute("DELETE FROM events")
            conn.commit()
            conn.close()
