#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="$ROOT/venv/bin/python"
if [ ! -x "$PY" ]; then
  PY=python3
fi
"$PY" -m py_compile "$ROOT"/backend/*.py "$ROOT"/vision/*.py "$ROOT"/scripts/*.py
"$PY" "$ROOT/scripts/check_clean.py"
(
  cd "$ROOT/frontend"
  npm run build
)
echo "VALIDATION_OK"
