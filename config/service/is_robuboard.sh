#!/bin/bash
set -eo pipefail

VENV="/home/robu/work/.venvs/robu"
PYTHON="$VENV/bin/python"

source /opt/ros/jazzy/setup.bash

# Workspace relativ zum Skript
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
WS_SETUP="$SCRIPT_DIR/../../install/setup.bash"

source "$WS_SETUP"

"$PYTHON" - <<'PY'
from robuboard.rpi.utils import is_robuboard_v1

result = is_robuboard_v1()
print(f"RobuBoard v1 detected: {result}")
raise SystemExit(0 if result else 1)
PY