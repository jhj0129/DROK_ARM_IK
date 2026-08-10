#!/usr/bin/env bash
set -Eeo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IK_ROOT="${DROK_IK_ROOT:-$HOME/IK_solver_MuJoCo}"

source /opt/ros/humble/setup.bash
if [[ ! -f "$IK_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] IK workspace setup 없음: $IK_ROOT/install/setup.bash"
  echo "필요하면 DROK_IK_ROOT를 지정하세요."
  exit 1
fi
if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] grasp workspace build 필요: $WS_ROOT/install/setup.bash"
  echo "먼저: bash $WS_ROOT/tools/build.sh"
  exit 1
fi
source "$IK_ROOT/install/setup.bash"
source "$WS_ROOT/install/setup.bash"
export DROK_GRASP_WS="$WS_ROOT"
export DROK_IK_ROOT="$IK_ROOT"
exec python3 "$WS_ROOT/tools/drok_auto_grasp_prototype1.py"
