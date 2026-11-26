#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 -m venv "$ROOT/venv"
"$ROOT/venv/bin/python" -m pip install --upgrade pip
"$ROOT/venv/bin/python" -m pip install -r "$ROOT/backend/requirements.txt" -r "$ROOT/vision/requirements.txt"
(
  cd "$ROOT/frontend"
  npm install
)
echo "SETUP_OK"
