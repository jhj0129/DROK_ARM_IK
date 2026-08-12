#!/bin/bash
set -e

source ~/DROK_ARM_IK-main/tools/source_env.sh

python3 - <<'PY'
import importlib.util
import math
import rclpy
from pathlib import Path

p = Path.home() / "DROK_ARM_IK-main/tools/interactive_box_ik_grasp_v11.py"

spec = importlib.util.spec_from_file_location("core", p)
core = importlib.util.module_from_spec(spec)
spec.loader.exec_module(core)

rclpy.init()
node = core.RealFeedbackNode()

try:
    if not node.wait_for_feedback(5.0):
        raise RuntimeError("joint feedback 없음")

    node.refresh()

    if node.current_q is None:
        raise RuntimeError("현재 joint feedback 없음")

    # ARM: JOINT1은 현재 위치를 그대로 유지한다.
    # ARM: JOINT2~JOINT6만 기존 HOME_Q로 이동한다.
    target_q = core.HOME_Q.copy()
    target_q[0] = float(node.current_q[0])

    print()
    print("============================================")
    print(" HOME - JOINT1 HOLD")
    print("============================================")
    print(
        "JOINT1 HOLD =",
        f"{math.degrees(target_q[0]):+.4f} deg"
    )
    print(
        "Current q    =",
        core.format_q_deg(node.current_q)
    )
    print(
        "Target q     =",
        core.format_q_deg(target_q)
    )
    print("============================================")

    exe = core.DirectArmRmdExecutor(node)

    try:
        ok = exe.move_poly5(
            target_q,
            6.0,
            "MANUAL HOME -> JOINT1 HOLD",
        )
        print("HOME RESULT =", ok)
    finally:
        exe.close()

finally:
    node.destroy_node()
    rclpy.shutdown()
PY
