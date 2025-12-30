#!/bin/bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash;
#source /home/robu/work/.robu/install/setup.bash;

# Workspace relativ zum Skript
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"
WS_SETUP="$SCRIPT_DIR/../../install/setup.bash"

#echo "[is_robuboard] SCRIPT_PATH = $SCRIPT_PATH"
#echo "[is_robuboard] SCRIPT_DIR  = $SCRIPT_DIR"
#echo "[is_robuboard] WS_SETUP   = $WS_SETUP"

source "$WS_SETUP"

/usr/bin/python3 - <<'PY'
from robuboard.rpi.utils import is_robuboard_v1

result = is_robuboard_v1()
print(f"RobuBoard v1 detected: {result}")
raise SystemExit(0 if result else 1)
PY
