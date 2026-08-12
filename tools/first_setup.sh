#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "============================================================"
echo " DROK_ARM_IK standalone setup"
echo "============================================================"
echo "External IK workspace: NOT REQUIRED"
echo "MuJoCo: NOT REQUIRED for fixed-practice real grasp"
echo

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "[ERROR] ROS 2 Humble not found."
  exit 1
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "[ERROR] colcon not found."
  exit 1
fi

missing=()

if ! dpkg-query -W -f='${Status}' libeigen3-dev 2>/dev/null \
    | grep -q "install ok installed"; then
  missing+=("libeigen3-dev")
fi

if ! dpkg-query -W -f='${Status}' libyaml-cpp-dev 2>/dev/null \
    | grep -q "install ok installed"; then
  missing+=("libyaml-cpp-dev")
fi

if (( ${#missing[@]} > 0 )); then
  echo "[ERROR] Missing packages:"
  printf '  %s\n' "${missing[@]}"
  echo
  echo "Install:"
  echo "  sudo apt update"
  echo "  sudo apt install -y ${missing[*]}"
  exit 2
fi

chmod +x "$WS_ROOT"/tools/*.sh
chmod +x "$WS_ROOT"/tools/*.py

source /opt/ros/humble/setup.bash

bash "$WS_ROOT/tools/build.sh"

IK_EXE="$WS_ROOT/install/drok_arm_kinematics/lib/drok_arm_kinematics/solve_ik_pose"
FK_EXE="$WS_ROOT/install/drok_arm_kinematics/lib/drok_arm_kinematics/test_fk"

[[ -x "$IK_EXE" ]] || {
  echo "[ERROR] solve_ik_pose was not built."
  exit 3
}

[[ -x "$FK_EXE" ]] || {
  echo "[ERROR] test_fk was not built."
  exit 4
}

echo
echo "[SETUP COMPLETE]"
echo "Next:"
echo "  source $WS_ROOT/tools/source_env.sh"
