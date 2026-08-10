#!/usr/bin/env bash
set -Eeo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/humble/setup.bash

if [[ -f "$WS_ROOT/install/setup.bash" ]]; then
  source "$WS_ROOT/install/setup.bash"
fi

ros2 topic pub --once \
  /drok_arm_auto/enable \
  std_msgs/msg/Bool \
  "{data: true}"
