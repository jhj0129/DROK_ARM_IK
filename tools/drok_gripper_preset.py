#!/usr/bin/env python3
"""
DROK gripper preset command
===========================

Usage:
  python3 drok_gripper_preset.py open
  python3 drok_gripper_preset.py grasp
  python3 drok_gripper_preset.py status

Uses the OPEN / GRASP protocol values currently saved in:
  interactive_box_ik_grasp_v11.py

No CAN interface state/bitrate changes.
No motor ROM/limit writes.
"""

from __future__ import annotations

import argparse
import importlib.util
import math
import os
from pathlib import Path

import rclpy


SCRIPT_DIR = Path(__file__).resolve().parent
V11_PATH = Path(
    os.environ.get(
        "DROK_V11_PATH",
        str(SCRIPT_DIR / "interactive_box_ik_grasp_v11.py"),
    )
).expanduser().resolve()


def load_core():
    if not V11_PATH.is_file():
        raise FileNotFoundError(
            f"v11 core not found: {V11_PATH}"
        )

    source = V11_PATH.read_text(encoding="utf-8")
    compile(source, str(V11_PATH), "exec")

    spec = importlib.util.spec_from_file_location(
        "drok_gripper_preset_core",
        V11_PATH,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("failed to load v11 core")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "command",
        choices=["open", "grasp", "close", "status"],
    )
    args = parser.parse_args()

    core = load_core()

    if args.command == "status":
        print(
            f"OPEN : {core.GRIPPER_OPEN_GAP_CM:.2f} cm | "
            f"protocol={core.GRIPPER_OPEN_PROTOCOL_DEG:+.6f} deg | "
            f"topic={core.GRIPPER_OPEN_PROTOCOL_DEG/core.GRIPPER_TOPIC_DIVISOR:+.6f} deg"
        )
        print(
            f"GRASP: {core.GRIPPER_CLOSE_GAP_CM:.2f} cm | "
            f"protocol={core.GRIPPER_CLOSE_PROTOCOL_DEG:+.6f} deg | "
            f"topic={core.GRIPPER_CLOSE_PROTOCOL_DEG/core.GRIPPER_TOPIC_DIVISOR:+.6f} deg"
        )
        return 0

    target_is_open = args.command == "open"

    target_gap = (
        core.GRIPPER_OPEN_GAP_CM
        if target_is_open
        else core.GRIPPER_CLOSE_GAP_CM
    )

    target_protocol = (
        core.GRIPPER_OPEN_PROTOCOL_DEG
        if target_is_open
        else core.GRIPPER_CLOSE_PROTOCOL_DEG
    )

    label = "FULL OPEN" if target_is_open else "GRASP/CLOSE"

    print("=" * 76)
    print(f" DROK GRIPPER PRESET -> {label}")
    print("=" * 76)
    print(f"gap target      : {target_gap:.2f} cm")
    print(f"protocol target : {target_protocol:+.6f} deg")
    print("CAN setup change: NONE")
    print("ROM write       : NONE")
    print("=" * 76)

    rclpy.init()
    node = None

    try:
        node = core.RealFeedbackNode()

        # Wait for ROS feedback infrastructure.
        if not node.wait_for_feedback(timeout_sec=8.0):
            print("[ERROR] /joint_states feedback unavailable.")
            return 2

        node.refresh(0.3)

        ok = core.send_gripper_protocol(
            node,
            float(target_protocol),
        )

        if not ok:
            print(f"[ERROR] {label} arrival failed.")
            return 3

        print(f"[OK] {label} reached.")
        return 0

    except KeyboardInterrupt:
        return 130

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
