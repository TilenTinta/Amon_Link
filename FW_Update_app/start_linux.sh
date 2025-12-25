#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ -f "${ROOT_DIR}/.venv/bin/python" ]]; then
  PYTHON_BIN="${ROOT_DIR}/.venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
  PYTHON_BIN="python3"
elif command -v python >/dev/null 2>&1; then
  PYTHON_BIN="python"
else
  echo "Python not found. Install Python or create .venv in ${ROOT_DIR}."
  exit 1
fi

export PYTHON="${PYTHON_BIN}"

"${PYTHON_BIN}" -m pip install -r "${ROOT_DIR}/requirements.txt"
npm install

"${PYTHON_BIN}" "${ROOT_DIR}/backend_server.py" &
BACKEND_PID=$!
trap 'kill ${BACKEND_PID} >/dev/null 2>&1 || true' EXIT

START_BACKEND=0 npm start
