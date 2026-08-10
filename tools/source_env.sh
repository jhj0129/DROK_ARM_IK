#!/usr/bin/env bash
# source ~/DROK_ARM_IK/tools/source_env.sh

_TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DROK_ARM_IK_WS="$(cd "$_TOOLS/.." && pwd)"
export DROK_GRASP_WS="$DROK_ARM_IK_WS"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble not found."
  return 1
fi

if [[ ! -f "$DROK_ARM_IK_WS/install/setup.bash" ]]; then
  echo "[ERROR] Workspace not built."
  echo "Run:"
  echo "  bash $DROK_ARM_IK_WS/tools/first_setup.sh"
  return 1
fi

source /opt/ros/humble/setup.bash
source "$DROK_ARM_IK_WS/install/setup.bash"

echo "[DROK ARM IK ENV READY]"
echo "  WS=$DROK_ARM_IK_WS"
echo "  external IK workspace: NOT REQUIRED"
