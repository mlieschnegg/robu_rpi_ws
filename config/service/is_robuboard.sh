#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/../../install/setup.bash"

/usr/bin/python3 - <<'PY'
from robuboard.rpi.utils import is_robuboard_v1
raise SystemExit(0 if is_robuboard_v1() else 1)
PY
