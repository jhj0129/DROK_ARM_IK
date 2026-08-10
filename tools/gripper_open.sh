#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IK_ROOT="${DROK_IK_ROOT:-$HOME/IK_solver_MuJoCo}"

source /opt/ros/humble/setup.bash

if [[ -f "$IK_ROOT/install/setup.bash" ]]; then
  source "$IK_ROOT/install/setup.bash"
fi

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  source "$WS_ROOT/install/setup.bash"
fi

export DROK_GRASP_WS="$WS_ROOT"
export DROK_IK_ROOT="$IK_ROOT"

exec python3 "$WS_ROOT/tools/drok_gripper_preset.py" open
