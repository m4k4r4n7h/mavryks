#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for name in backend frontend vision; do
  if [ -f "$ROOT/run/$name.pid" ]; then
    kill "$(cat "$ROOT/run/$name.pid")" 2>/dev/null || true
    rm -f "$ROOT/run/$name.pid"
  fi
done
pkill -f "uvicorn main:app --host 0.0.0.0 --port 8000" 2>/dev/null || true
pkill -f "vite --host 0.0.0.0 --port 5173" 2>/dev/null || true
pkill -f "python tracker.py" 2>/dev/null || true
echo "STOPPED"
