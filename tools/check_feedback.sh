#!/usr/bin/env bash
set -Eeo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] $WS_ROOT/install/setup.bash 없음"
  echo "먼저: bash $WS_ROOT/tools/build.sh"
  exit 1
fi
source "$WS_ROOT/install/setup.bash"
echo "===== RAW MOTOR TOPICS ====="
ros2 topic list | grep '^/motor_angles/' | sort || true
echo
echo "===== LOGICAL JOINT STATES ====="
ros2 topic echo /joint_states --once
