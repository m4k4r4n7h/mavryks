#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -x "$ROOT/venv/bin/python" ] || [ ! -d "$ROOT/frontend/node_modules" ]; then
  bash "$ROOT/scripts/setup.sh"
fi
bash "$ROOT/scripts/run.sh"
for _ in $(seq 1 30); do
  if "$ROOT/venv/bin/python" -c 'import urllib.request; urllib.request.urlopen("http://127.0.0.1:8000/api/health", timeout=1).read()' 2>/dev/null; then
    echo "BACKEND_READY"
    break
  fi
  sleep 1
done
echo "CONSOLE http://localhost:5173"
echo "API http://localhost:8000/api/health"
echo "STATE http://localhost:8000/api/state"
echo "LOGS $ROOT/run"
echo "STOP bash $ROOT/stop.sh"
