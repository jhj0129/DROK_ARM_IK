#!/usr/bin/env bash
set -Eeo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
source /opt/ros/humble/setup.bash
if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] install/setup.bash 없음"
  echo "먼저 실행: bash $WS_ROOT/tools/build.sh"
  exit 1
fi
source "$WS_ROOT/install/setup.bash"
ros2 launch drok_real_arm_bridge real_arm_bridge.launch.py \
  dry_run:=false \
  default_max_speed:=30
