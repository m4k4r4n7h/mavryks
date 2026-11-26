#!/usr/bin/env bash
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ ! -f "$ROOT/venv/bin/activate" ]; then
  bash "$ROOT/scripts/setup.sh"
fi
if [ -f "$ROOT/.env" ]; then
  set -a
  source "$ROOT/.env"
  set +a
fi
source "$ROOT/venv/bin/activate"
export MAVRYKS_DB="${MAVRYKS_DB:-$ROOT/backend/mavryks.db}"
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$ROOT"
echo "MAVRYKS ENVIRONMENT ACTIVE"
