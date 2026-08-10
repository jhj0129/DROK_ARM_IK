#!/usr/bin/env bash
# Source this file:
#   source ~/DROK_ARM_IK/tools/source_env.sh
#
# Optional:
#   export DROK_IK_ROOT=/path/to/IK_solver_MuJoCo

_DROK_TOOLS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DROK_GRASP_WS="$(cd "$_DROK_TOOLS_DIR/.." && pwd)"
export DROK_IK_ROOT="${DROK_IK_ROOT:-$HOME/IK_solver_MuJoCo}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble setup not found: /opt/ros/humble/setup.bash"
  return 1
fi

if [[ ! -f "$DROK_IK_ROOT/install/setup.bash" ]]; then
  echo "[ERROR] IK workspace is not built:"
  echo "        $DROK_IK_ROOT/install/setup.bash"
  echo "Set another path with:"
  echo "        export DROK_IK_ROOT=/path/to/IK_solver_MuJoCo"
  return 1
fi

if [[ ! -f "$DROK_GRASP_WS/install/setup.bash" ]]; then
  echo "[ERROR] DROK grasp workspace is not built:"
  echo "        $DROK_GRASP_WS/install/setup.bash"
  echo "Run:"
  echo "        bash $DROK_GRASP_WS/tools/build.sh"
  return 1
fi

source /opt/ros/humble/setup.bash
source "$DROK_IK_ROOT/install/setup.bash"
source "$DROK_GRASP_WS/install/setup.bash"

echo "[DROK ENV READY]"
echo "  DROK_GRASP_WS=$DROK_GRASP_WS"
echo "  DROK_IK_ROOT=$DROK_IK_ROOT"
