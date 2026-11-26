#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/venv/bin/python"
if [ ! -x "$PY" ]; then
  PY=python3
fi
mkdir -p "$ROOT/run"
(
  cd "$ROOT/backend"
  exec "$PY" -m uvicorn main:app --host 0.0.0.0 --port 8000
) >"$ROOT/run/backend.log" 2>&1 &
echo $! >"$ROOT/run/backend.pid"
(
  cd "$ROOT/frontend"
  exec npm run dev -- --host 0.0.0.0 --port 5173
) >"$ROOT/run/frontend.log" 2>&1 &
echo $! >"$ROOT/run/frontend.pid"
(
  cd "$ROOT/vision"
  exec "$PY" tracker.py
) >"$ROOT/run/vision.log" 2>&1 &
echo $! >"$ROOT/run/vision.pid"
echo "STARTED"
