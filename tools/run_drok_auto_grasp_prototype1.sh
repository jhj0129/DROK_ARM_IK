#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] Build first:"
  echo "  bash $WS_ROOT/tools/first_setup.sh"
  exit 1
fi

source "$WS_ROOT/install/setup.bash"
export DROK_GRASP_WS="$WS_ROOT"

exec python3 \
  "$WS_ROOT/tools/drok_auto_grasp_prototype1.py"
