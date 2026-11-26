#!/usr/bin/env python3
from __future__ import annotations

import os
import signal
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent
PY = ROOT / "venv" / "bin" / "python"
LOG_DIR = ROOT / "run"
STOPPING = False


def ensure_setup() -> None:
    if not PY.exists() or not os.access(PY, os.X_OK) or not (ROOT / "frontend" / "node_modules").exists():
        subprocess.run(["bash", str(ROOT / "scripts" / "setup.sh")], cwd=ROOT, check=True)


def request_stop(_signum: int, _frame: object) -> None:
    global STOPPING
    STOPPING = True


def wait_ready() -> None:
    for _ in range(30):
        try:
            with urllib.request.urlopen("http://127.0.0.1:8000/api/health", timeout=1) as response:
                response.read()
            print("BACKEND_READY", flush=True)
            return
        except Exception:
            time.sleep(1)
    print("BACKEND_NOT_READY", flush=True)


def start_services() -> list[subprocess.Popen[bytes]]:
    LOG_DIR.mkdir(exist_ok=True)
    specs = [
        ("backend", [str(PY), "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"], ROOT / "backend"),
        ("frontend", ["npm", "run", "dev", "--", "--host", "0.0.0.0", "--port", "5173"], ROOT / "frontend"),
        ("vision", [str(PY), "tracker.py"], ROOT / "vision"),
    ]
    procs: list[subprocess.Popen[bytes]] = []
    for name, cmd, cwd in specs:
        log = open(LOG_DIR / f"{name}.log", "ab", buffering=0)
        proc = subprocess.Popen(cmd, cwd=cwd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        (LOG_DIR / f"{name}.pid").write_text(str(proc.pid))
        procs.append(proc)
    return procs


def stop_services(procs: list[subprocess.Popen[bytes]]) -> None:
    for proc in procs:
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
    for proc in procs:
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
    for name in ("backend", "frontend", "vision"):
        (LOG_DIR / f"{name}.pid").unlink(missing_ok=True)


def main() -> int:
    ensure_setup()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    procs = start_services()
    print("CONSOLE http://localhost:5173", flush=True)
    print("API http://localhost:8000/api/health", flush=True)
    print("STATE http://localhost:8000/api/state", flush=True)
    print(f"LOGS {LOG_DIR}", flush=True)
    print("CTRL-C TO STOP", flush=True)
    wait_ready()
    while not STOPPING:
        if any(proc.poll() is not None for proc in procs):
            print("SERVICE_EXITED", flush=True)
            break
        time.sleep(0.5)
    stop_services(procs)
    print("STOPPED", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
