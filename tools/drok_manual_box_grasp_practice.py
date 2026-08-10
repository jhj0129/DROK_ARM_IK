#!/usr/bin/env python3
"""
DROK MANUAL BOX GRASP PRACTICE
==============================

Manual practice source for the existing Prototype 1 pipeline.

Flow:
  manual ARM_BASE_LINK XYZ
    -> optional Prototype 1 offsets
    -> exact HOME_Q staging
    -> v11 top-down IK feasibility from HOME
    -> optional real execution
    -> grasp
    -> lift
    -> HOME while holding object

No YOLO, no TF, no MuJoCo.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
from typing import List

import rclpy


# ============================================================================
# MANUAL TEST SWITCH
# ============================================================================
#
# 기본은 OFF.
# 사용할 때 아래 True 줄의 주석(#)만 제거하면 됩니다.
#
MANUAL_TEST_ENABLED = False
MANUAL_TEST_ENABLED = True


# ============================================================================
# MANUAL BOX POSITION — ARM_BASE_LINK 기준
# ============================================================================
#
# +X = 전방
# +Y = 왼쪽
# +Z = 위
# 단위 = meter
#
MANUAL_BOX_X_M = 0.4000
MANUAL_BOX_Y_M = 0.0000
MANUAL_BOX_Z_M = -0.0325


# ============================================================================
# OFFSET POLICY
# ============================================================================
#
# True:
#   drok_auto_grasp_prototype1.py에 설정한
#   ROBOT_OFFSET_FORWARD_CM / RIGHT_CM / UP_CM 을 그대로 추가 적용.
#
# False:
#   MANUAL_BOX_X/Y/Z 자체를 최종 grasp 위치로 사용.
#
APPLY_PROTOTYPE_ROBOT_OFFSETS = False

# ============================================================================
# REAL EXECUTION SWITCH
# ============================================================================
#
# False = IK 검사만 하고 실제 모터는 움직이지 않음.
# True  = IK 가능 시 실제 grasp -> lift -> HOME 실행.
#
EXECUTE_REAL_MOTION = False
EXECUTE_REAL_MOTION = True ##실제 움직임 미사용시 주석처리


PROTOTYPE_PATH = (
    Path(__file__).resolve().parent
    / "drok_auto_grasp_prototype1.py"
)


def load_prototype():
    if not PROTOTYPE_PATH.is_file():
        raise FileNotFoundError(
            "Prototype 1 not found:\n"
            f"{PROTOTYPE_PATH}"
        )

    spec = importlib.util.spec_from_file_location(
        "drok_auto_grasp_prototype1",
        PROTOTYPE_PATH,
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(
            "Failed to import Prototype 1"
        )

    module = importlib.util.module_from_spec(
        spec
    )

    spec.loader.exec_module(
        module
    )

    return module


def raw_manual_target() -> List[float]:
    return [
        float(MANUAL_BOX_X_M),
        float(MANUAL_BOX_Y_M),
        float(MANUAL_BOX_Z_M),
    ]


def corrected_target(
    prototype,
) -> List[float]:
    xyz = raw_manual_target()

    if not APPLY_PROTOTYPE_ROBOT_OFFSETS:
        return xyz

    return [
        xyz[0]
        + prototype.ROBOT_OFFSET_FORWARD_CM
        / 100.0,

        xyz[1]
        - prototype.ROBOT_OFFSET_RIGHT_CM
        / 100.0,

        xyz[2]
        + prototype.ROBOT_OFFSET_UP_CM
        / 100.0,
    ]


def fmt(
    xyz,
) -> str:
    return (
        f"({xyz[0]:+.4f}, "
        f"{xyz[1]:+.4f}, "
        f"{xyz[2]:+.4f})"
    )


def main() -> int:
    print("=" * 76)
    print(
        " DROK MANUAL BOX GRASP PRACTICE"
    )
    print("=" * 76)

    if not MANUAL_TEST_ENABLED:
        print(
            "[OFF] MANUAL_TEST_ENABLED = False"
        )

        print()
        print(
            "이 파일은 현재 비활성화 상태입니다."
        )

        print(
            "사용하려면 파일 위쪽에서:"
        )

        print()
        print(
            "  MANUAL_TEST_ENABLED = False"
        )

        print(
            "  # MANUAL_TEST_ENABLED = True"
        )

        print()
        print(
            "두 번째 줄의 #만 제거하세요."
        )

        print()
        print(
            "실제 모터 동작은 EXECUTE_REAL_MOTION을 "
            "별도로 True로 해야 합니다."
        )

        print("=" * 76)

        return 0

    prototype = load_prototype()

    raw_xyz = raw_manual_target()

    target_xyz = corrected_target(
        prototype
    )

    print(
        "MANUAL TEST: ENABLED"
    )

    print()
    print(
        "Raw manual box [ARM_BASE_LINK]:"
    )

    print(
        " ",
        fmt(raw_xyz),
    )

    print()
    print(
        "Prototype offsets:"
    )

    print(
        f"  forward = "
        f"{prototype.ROBOT_OFFSET_FORWARD_CM:+.2f} cm"
    )

    print(
        f"  right   = "
        f"{prototype.ROBOT_OFFSET_RIGHT_CM:+.2f} cm"
    )

    print(
        f"  up      = "
        f"{prototype.ROBOT_OFFSET_UP_CM:+.2f} cm"
    )

    print()
    print(
        f"Apply offsets: "
        f"{APPLY_PROTOTYPE_ROBOT_OFFSETS}"
    )

    print()
    print(
        "Final grasp target [ARM_BASE_LINK]:"
    )

    print(
        " ",
        fmt(target_xyz),
    )

    print()
    print(
        f"EXECUTE_REAL_MOTION = "
        f"{EXECUTE_REAL_MOTION}"
    )

    print(
        "Start policy: EXACT HOME_Q -> IK CHECK -> GRASP"
    )

    print("=" * 76)

    if not prototype.AutoGraspPrototype1.in_workspace(
        target_xyz
    ):
        print(
            "[BLOCK] Manual target is outside "
            "Prototype 1 workspace gate."
        )

        return 2

    rclpy.init()

    real_node = None

    try:
        core = prototype.load_v11_core()

        real_node = core.RealFeedbackNode()

        if not real_node.wait_for_feedback():
            print(
                "[ERROR] JOINT1~6 feedback 없음"
            )

            return 3

        print()

        if EXECUTE_REAL_MOTION:
            print(
                "[0/2] Move real arm to exact HOME_Q..."
            )

            if not prototype.move_real_arm_to_exact_home(
                core,
                real_node,
                prototype.START_HOME_SEC,
            ):
                print(
                    "[ERROR] Start HOME staging failed."
                )
                return 4

            planner_start_override = None

        else:
            print(
                "[0/2] DRY RUN: real arm does NOT move."
            )

            print(
                "IK is calculated as if the arm starts at exact HOME_Q."
            )

            planner_start_override = (
                core.HOME_Q
            )

        print()
        print(
            "[1/2] IK feasibility check from HOME..."
        )

        plan = prototype.build_topdown_plan(
            core,
            real_node,
            target_xyz,
            start_q_override=planner_start_override,
        )

        if not plan["ok"]:
            print()
            print("=" * 76)
            print(
                " IK UNREACHABLE"
            )

            print(
                f" failed stage: "
                f"{plan['failed']}"
            )

            print(
                " REAL MOTOR COMMAND = NONE"
            )

            print("=" * 76)

            return 4

        print()
        print("=" * 76)
        print(
            " IK REACHABLE"
        )
        print("=" * 76)

        print(
            "GRASP:",
            fmt(
                plan["grasp_xyz"]
            ),
        )

        print(
            "NEAR :",
            fmt(
                plan["near_xyz"]
            ),
        )

        print(
            "LIFT :",
            fmt(
                plan["lift_xyz"]
            ),
        )

        print(
            f"Selected Pitch [deg]: "
            f"{math.degrees(plan['approach_pitch_rad']):+.2f}"
        )

        print("=" * 76)

        if not EXECUTE_REAL_MOTION:
            print()
            print(
                "[DRY RUN COMPLETE]"
            )

            print(
                "IK는 도달 가능합니다."
            )

            print(
                "EXECUTE_REAL_MOTION = False 이므로 "
                "실제 로봇은 움직이지 않습니다."
            )

            print()
            print(
                "실제 grasp 연습 시:"
            )

            print(
                "  EXECUTE_REAL_MOTION = False"
            )

            print(
                "  # EXECUTE_REAL_MOTION = True"
            )

            print()
            print(
                "두 번째 줄의 #를 제거하세요."
            )

            return 0

        print()
        print(
            "[2/2] REAL grasp execution..."
        )

        ok = (
            prototype.execute_plan_and_return_home(
                core,
                real_node,
                plan,
            )
        )

        if not ok:
            print()
            print(
                "[ERROR] Real grasp execution failed."
            )

            return 5

        print()
        print("=" * 76)
        print(
            " MANUAL PRACTICE COMPLETE"
        )

        print(
            " Object is held and arm returned HOME."
        )

        print("=" * 76)

        return 0

    except KeyboardInterrupt:
        return 130

    except Exception as exc:
        print()
        print(
            "[ERROR]",
            exc,
        )

        return 10

    finally:
        if real_node is not None:
            try:
                real_node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(
        main()
    )
