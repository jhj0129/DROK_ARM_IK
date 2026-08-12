#!/usr/bin/env bash
set -Eeo pipefail
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS_ROOT"
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --packages-select \
    drok_arm_kinematics \
    drok_real_arm_bridge
