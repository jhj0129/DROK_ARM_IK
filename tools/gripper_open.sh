#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  source "$WS_ROOT/install/setup.bash"
fi

export DROK_GRASP_WS="$WS_ROOT"

exec python3   "$WS_ROOT/tools/drok_gripper_preset.py" open
