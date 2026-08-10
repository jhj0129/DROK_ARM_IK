#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IK_ROOT="${DROK_IK_ROOT:-$HOME/IK_solver_MuJoCo}"

echo "============================================================"
echo " DROK_ARM_IK - first setup"
echo "============================================================"
echo "WS_ROOT : $WS_ROOT"
echo "IK_ROOT : $IK_ROOT"
echo
echo "This script does NOT change CAN interface state/bitrate."
echo "This script does NOT write motor ROM/limits."
echo

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble not found."
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "[ERROR] colcon not found."
  exit 1
fi

if [[ ! -d "$IK_ROOT" ]]; then
  echo "[ERROR] IK workspace directory not found:"
  echo "        $IK_ROOT"
  echo
  echo "Put IK_solver_MuJoCo at ~/IK_solver_MuJoCo"
  echo "or set:"
  echo "        export DROK_IK_ROOT=/path/to/IK_solver_MuJoCo"
  exit 1
fi

chmod +x "$WS_ROOT"/tools/*.sh
chmod +x "$WS_ROOT"/tools/*.py

source /opt/ros/humble/setup.bash

if [[ ! -f "$IK_ROOT/install/setup.bash" ]]; then
  echo
  echo "[INFO] IK workspace install/setup.bash not found."
  echo "[INFO] Building IK workspace first..."
  cd "$IK_ROOT"
  colcon build --symlink-install
fi

source "$IK_ROOT/install/setup.bash"

echo
echo "[INFO] Building DROK_ARM_IK workspace..."
cd "$WS_ROOT"
bash "$WS_ROOT/tools/build.sh"

echo
echo "============================================================"
echo " SETUP COMPLETE"
echo "============================================================"
echo "Next terminal:"
echo
echo "  source $WS_ROOT/tools/source_env.sh"
echo
echo "Then use:"
echo "  bash $WS_ROOT/tools/run_real.sh"
echo "  bash $WS_ROOT/tools/run_drok_auto_grasp_prototype1.sh"
echo "  bash $WS_ROOT/tools/trigger_fixed_ik_grasp.sh"
