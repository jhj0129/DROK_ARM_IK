#!/usr/bin/env python3
"""
DROK AUTO GRASP PROTOTYPE 1
===========================

Goal
----
Two target-source modes are supported.

FIXED PRACTICE mode (default):
1) IK mode ON (/drok_arm_auto/enable=True).
2) Ignore YOLO XYZ / TF completely.
3) Use the configured fixed ARM_BASE_LINK target.
4) Move to exact HOME_Q -> FULL OPEN -> TOP-DOWN IK -> GRASP -> LIFT -> HOME.
5) Publish DONE=True and turn arm-auto mode OFF.

CAMERA mode (optional):
1) Receive YOLO object position.
2) Transform camera_link position -> ARM_BASE_LINK with TF.
3) Apply user-editable offsets and stabilise samples.
4) Execute the same grasp sequence.

No MuJoCo preview is launched.

IMPORTANT
---------
- This prototype reuses the already-tested v11 IK/RMD/gripper implementation as
  a library. It NEVER calls preview_plan_mujoco().
- CAN interface state/bitrate is not changed.
- Motor ROM/limits are not written.
- In the default FIXED PRACTICE mode, /drok_arm_auto/enable=True is the IK-mode
  trigger. Incoming object coordinates are not used.
- CAMERA mode can be restored by setting USE_FIXED_PRACTICE_TARGET=False.
"""

from __future__ import annotations

import importlib.util
import math
import statistics
import threading
import time
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


# ============================================================================
# USER CONFIGURATION — edit these values
# ============================================================================

# ---------------------------------------------------------------------------
# TARGET SOURCE MODE
#
# True (default for current practice):
#   /drok_arm_auto/enable=True immediately starts a grasp at the fixed target.
#   YOLO XYZ and camera TF are NOT used.
#
# False (future camera mode):
#   wait for TARGET_CLASS + YOLO XYZ, transform with TF, then grasp that target.
# ---------------------------------------------------------------------------
USE_FIXED_PRACTICE_TARGET = True

# Current practice target in ARM_BASE_LINK [m].
#   +X = forward, +Y = left, +Z = up
FIXED_GRASP_X_M = 0.4000
FIXED_GRASP_Y_M = 0.0000
FIXED_GRASP_Z_M = -0.0325

# Target class published by the supplied YOLO node (CAMERA mode only).
TARGET_CLASS = "supply_box"

YOLO_CLASS_TOPIC = "/yolo_detected_object"
YOLO_XYZ_TOPIC = "/yolo_object_xyz"

# The YOLO code labels XYZ as camera_link.
DEFAULT_CAMERA_FRAME = "camera_link"

# IK/FK model target frame.
ROBOT_TARGET_FRAME = "ARM_BASE_LINK"

# ---------------------------------------------------------------------------
# Robot position calibration offset (applies to FIXED and CAMERA modes)
#
# ARM_BASE_LINK convention:
#   +X = forward
#   +Y = left
#   +Z = up
#
# User-facing convention below:
#   FORWARD +  -> target X increases
#   RIGHT   +  -> target Y decreases
#   UP      +  -> target Z increases
#
# Example:
#   if you want the gripper target 10 cm to the RIGHT of the transformed
#   detected point:
#
#       ROBOT_OFFSET_RIGHT_CM = 10.0
# ---------------------------------------------------------------------------
ROBOT_OFFSET_FORWARD_CM = 0.0
ROBOT_OFFSET_RIGHT_CM = 0.0
ROBOT_OFFSET_UP_CM = 0.0

# Default: node waits for an explicit IK-mode command.
# Publish /drok_arm_auto/enable=True to start one mission.
# If True, fixed mode starts one mission automatically shortly after launch.
START_ENABLED = False

AUTO_ENABLE_TOPIC = "/drok_arm_auto/enable"
STATUS_TOPIC = "/drok_arm_auto/status"
DONE_TOPIC = "/drok_arm_auto/done"

# Current YOLO publishes class and XYZ on separate topics.
# Treat an XYZ as belonging to TARGET_CLASS only when the last class message
# arrived very recently.
CLASS_XYZ_ASSOCIATION_MAX_SEC = 0.25

# Use several transformed samples and the coordinate-wise median instead of
# grabbing from one noisy depth pixel.
REQUIRED_SAMPLES = 5
SAMPLE_WINDOW_SEC = 1.0
MAX_MEDIAN_DEVIATION_M = 0.08

# Latest TF is used in Prototype 1. This is more tolerant of unsynchronised
# clocks between the camera computer and arm computer.
USE_LATEST_TF = True
TF_TIMEOUT_SEC = 0.30

# Simple robot-frame workspace gate before IK.
WORKSPACE_X_MIN_M = 0.10
WORKSPACE_X_MAX_M = 0.75
WORKSPACE_Y_MIN_M = -0.55
WORKSPACE_Y_MAX_M = +0.55
WORKSPACE_Z_MIN_M = -0.40
WORKSPACE_Z_MAX_M = 0.65

# Retry after unreachable/noisy target.
FAILURE_COOLDOWN_SEC = 3.0

# After grasp/lift, carry the object back to the known physical HOME q.
# Mission start: always move to the exact known HOME_Q first.
START_HOME_SEC = 3.0*2

RETURN_HOME_SEC = 3.0*2

# Proven v11 core. No MuJoCo viewer function is called by this prototype.
# Portable: v11 is expected next to this Prototype file in the tools folder.
# Optional override:
#   export DROK_V11_PATH=/absolute/path/to/interactive_box_ik_grasp_v11.py
V11_CORE_PATH = Path(
    __import__("os").environ.get(
        "DROK_V11_PATH",
        str(
            Path(__file__).resolve().parent
            / "interactive_box_ik_grasp_v11.py"
        ),
    )
).expanduser().resolve()


# ============================================================================
# Small math helpers
# ============================================================================

def quaternion_rotate(
    qx: float,
    qy: float,
    qz: float,
    qw: float,
    point: Sequence[float],
) -> Tuple[float, float, float]:
    """
    Rotate a 3D point with quaternion q * p * q^-1.
    """
    norm = math.sqrt(
        qx * qx
        + qy * qy
        + qz * qz
        + qw * qw
    )

    if norm < 1.0e-12:
        raise RuntimeError("TF quaternion norm is zero")

    qx /= norm
    qy /= norm
    qz /= norm
    qw /= norm

    px, py, pz = (
        float(point[0]),
        float(point[1]),
        float(point[2]),
    )

    # Efficient quaternion-vector rotation.
    tx = 2.0 * (
        qy * pz
        - qz * py
    )
    ty = 2.0 * (
        qz * px
        - qx * pz
    )
    tz = 2.0 * (
        qx * py
        - qy * px
    )

    rx = (
        px
        + qw * tx
        + (
            qy * tz
            - qz * ty
        )
    )

    ry = (
        py
        + qw * ty
        + (
            qz * tx
            - qx * tz
        )
    )

    rz = (
        pz
        + qw * tz
        + (
            qx * ty
            - qy * tx
        )
    )

    return (
        rx,
        ry,
        rz,
    )


def median_xyz(
    samples: Sequence[Sequence[float]],
) -> List[float]:
    return [
        float(
            statistics.median(
                sample[index]
                for sample in samples
            )
        )
        for index in range(3)
    ]


def euclidean_distance(
    a: Sequence[float],
    b: Sequence[float],
) -> float:
    return math.sqrt(
        sum(
            (
                float(x)
                - float(y)
            ) ** 2
            for x, y in zip(a, b)
        )
    )


def format_xyz(
    xyz: Sequence[float],
) -> str:
    return (
        f"({xyz[0]:+.4f}, "
        f"{xyz[1]:+.4f}, "
        f"{xyz[2]:+.4f})"
    )


# ============================================================================
# Load tested v11 core
# ============================================================================

def load_v11_core():
    if not V11_CORE_PATH.is_file():
        raise FileNotFoundError(
            "v11 core not found:\n"
            f"{V11_CORE_PATH}\n"
            "Copy interactive_box_ik_grasp_v11.py into "
            "~/DROK_grasp_ws/tools first."
        )

    spec = importlib.util.spec_from_file_location(
        "drok_v11_grasp_core",
        V11_CORE_PATH,
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(
            "Failed to create v11 import spec"
        )

    module = importlib.util.module_from_spec(
        spec
    )

    spec.loader.exec_module(
        module
    )

    return module



# ============================================================================
# Exact HOME staging
# ============================================================================

def move_real_arm_to_exact_home(
    core,
    real_node,
    duration_sec: float = START_HOME_SEC,
) -> bool:
    """
    Mission-start preparation:
      1) move the real arm to exact HOME_Q and verify arrival,
      2) fully OPEN the gripper using the saved v11 OPEN calibration,
      3) verify gripper arrival.

    No CAN configuration changes.
    No ROM writes.
    """
    real_node.refresh()

    if real_node.current_q is None:
        print(
            "[BLOCK] Cannot read current arm q before HOME staging"
        )
        return False

    print()
    print("=" * 76)
    print(" START PREPARATION: EXACT HOME_Q -> GRIPPER FULL OPEN")
    print("=" * 76)

    print(
        "Current q [deg]:",
        core.format_q_deg(
            real_node.current_q
        ),
    )

    print(
        "Exact HOME q [deg]:",
        core.format_q_deg(
            core.HOME_Q
        ),
    )

    executor = core.DirectArmRmdExecutor(
        real_node
    )

    try:
        ok = executor.move_poly5(
            core.HOME_Q,
            float(duration_sec),
            "MISSION START -> EXACT HOME_Q",
        )
    finally:
        executor.close()

    if not ok:
        print(
            "[BLOCK] Failed to reach exact HOME_Q"
        )
        return False

    real_node.refresh()

    if real_node.current_q is None:
        print(
            "[BLOCK] HOME command finished but joint feedback was lost"
        )
        return False

    print(
        "[HOME READY] exact HOME_Q arrival verified."
    )

    print()
    print(
        f"[START GRIPPER] FULL OPEN "
        f"{core.GRIPPER_OPEN_GAP_CM:.2f} cm"
    )
    print(
        f"[START GRIPPER] protocol target = "
        f"{core.GRIPPER_OPEN_PROTOCOL_DEG:.6f} deg"
    )

    if not core.send_gripper_protocol(
        real_node,
        core.GRIPPER_OPEN_PROTOCOL_DEG,
    ):
        print(
            "[BLOCK] FULL OPEN gripper arrival failed."
        )
        return False

    print(
        "[GRIPPER READY] FULL OPEN arrival verified."
    )

    print("=" * 76)

    return True


# ============================================================================
# Absolute-target v11 planner
# ============================================================================

def build_topdown_plan(
    core,
    real_node,
    grasp_xyz: Sequence[float],
    start_q_override: Optional[Sequence[float]] = None,
):
    """
    Same top-down geometry/pitch search as v11, but:
      - target is an ABSOLUTE ARM_BASE_LINK XYZ,
      - no interactive input,
      - no MuJoCo preview.
    """
    baseline = core.load_baseline_ik_module()

    limits = baseline.load_joint_limits(
        core.URDF_PATH
    )

    real_node.refresh()

    if start_q_override is not None:
        start_q = [
            float(value)
            for value in start_q_override
        ]

        print(
            "[PLANNER] start q override:",
            core.format_q_deg(
                start_q
            ),
        )

    else:
        if real_node.current_q is None:
            raise RuntimeError(
                "No current JOINT1..JOINT6 feedback"
            )

        start_q = real_node.current_q.copy()

    home_tcp, home_rpy_raw = (
        core.run_project_fk_pose(
            core.HOME_Q
        )
    )

    home_rpy = core.canonicalize_zyx_rpy(
        home_rpy_raw
    )

    home_pitch_rad = float(
        home_rpy[1]
    )

    grasp = [
        float(grasp_xyz[0]),
        float(grasp_xyz[1]),
        float(grasp_xyz[2]),
    ]

    near = [
        grasp[0],
        grasp[1],
        grasp[2]
        + core.NEAR_STANDOFF_M,
    ]

    lift = [
        grasp[0],
        grasp[1],
        grasp[2]
        + core.LIFT_HEIGHT_M,
    ]

    requested_pitch = (
        core.approach_pitch_from_vector(
            near,
            grasp,
        )
    )

    prealign_rpy = (
        core.make_roll_yaw_locked_rpy(
            home_pitch_rad
        )
    )

    print()
    print("=" * 76)
    print(" AUTO IK FEASIBILITY — v11 TOP-DOWN")
    print("=" * 76)

    print(
        "HOME TCP :",
        format_xyz(home_tcp),
    )

    print(
        "GRASP    :",
        format_xyz(grasp),
    )

    print(
        "NEAR     :",
        format_xyz(near),
    )

    print(
        "LIFT     :",
        format_xyz(lift),
    )

    print(
        f"Requested top-down pitch: "
        f"{math.degrees(requested_pitch):+.2f} deg"
    )

    # 0) PREALIGN
    q_prealign = core.solve_waypoint(
        baseline,
        "AUTO PREALIGN @ HOME TCP",
        home_tcp,
        prealign_rpy,
        start_q,
        limits,
    )

    if q_prealign is None:
        return {
            "ok": False,
            "failed": "PREALIGN_HOME",
        }

    selected_pitch = None
    selected_approach1 = None
    selected_approach2 = None
    selected_lift = None

    for candidate_pitch in (
        core.make_pitch_search_candidates(
            requested_pitch
        )
    ):
        print()
        print(
            f"[AUTO PITCH TEST] "
            f"{math.degrees(candidate_pitch):+.2f} deg"
        )

        approach1_path = (
            core.solve_pitch_cartesian_path(
                baseline,
                "AUTO APPROACH1",
                home_tcp,
                near,
                q_prealign,
                limits,
                core.APPROACH1_IK_STEP_M,
                home_pitch_rad,
                candidate_pitch,
            )
        )

        if approach1_path is None:
            continue

        q_near = list(
            approach1_path[-1]
        )

        approach2_path = (
            core.solve_pitch_cartesian_path(
                baseline,
                "AUTO APPROACH2",
                near,
                grasp,
                q_near,
                limits,
                core.APPROACH2_IK_STEP_M,
                candidate_pitch,
                candidate_pitch,
            )
        )

        if approach2_path is None:
            continue

        q_grasp = list(
            approach2_path[-1]
        )

        lift_path = (
            core.solve_pitch_cartesian_path(
                baseline,
                "AUTO LIFT",
                grasp,
                lift,
                q_grasp,
                limits,
                core.LIFT_IK_STEP_M,
                candidate_pitch,
                candidate_pitch,
            )
        )

        if lift_path is None:
            continue

        selected_pitch = float(
            candidate_pitch
        )
        selected_approach1 = (
            approach1_path
        )
        selected_approach2 = (
            approach2_path
        )
        selected_lift = lift_path
        break

    if (
        selected_pitch is None
        or selected_approach1 is None
        or selected_approach2 is None
        or selected_lift is None
    ):
        return {
            "ok": False,
            "failed": "PITCH_FEASIBILITY_SEARCH",
        }

    print()
    print(
        f"[IK REACHABLE] selected pitch = "
        f"{math.degrees(selected_pitch):+.2f} deg"
    )

    return {
        "ok": True,
        "start_q": start_q,
        "home_tcp": home_tcp,
        "home_pitch_rad": home_pitch_rad,
        "grasp_xyz": grasp,
        "near_xyz": near,
        "lift_xyz": lift,
        "q_prealign": list(
            q_prealign
        ),
        "q_near": list(
            selected_approach1[-1]
        ),
        "q_grasp": list(
            selected_approach2[-1]
        ),
        "q_lift": list(
            selected_lift[-1]
        ),
        "approach_pitch_rad": (
            selected_pitch
        ),
        "approach1_path": (
            selected_approach1
        ),
        "approach2_path": (
            selected_approach2
        ),
        "lift_path": (
            selected_lift
        ),
    }


# ============================================================================
# Real automatic execution
# ============================================================================

def execute_plan_and_return_home(
    core,
    real_node,
    plan,
) -> bool:
    """
    Automatic:
      OPEN
      PREALIGN
      APPROACH1
      re-read real NEAR q
      re-calculate APPROACH2/LIFT
      APPROACH2
      CLOSE
      LIFT
      HOME while holding object

    No user confirmation and no MuJoCo preview.
    """
    executor = core.DirectArmRmdExecutor(
        real_node
    )

    try:
        print()
        print("=" * 76)
        print(" AUTO REAL GRASP START")
        print("=" * 76)

        print(
            "[GRIPPER] FULL OPEN already completed before IK check."
        )

        if not executor.move_poly5(
            plan["q_prealign"],
            core.REAL_CURRENT_TO_PREALIGN_SEC,
            "AUTO CURRENT -> PREALIGN",
        ):
            return False

        if not executor.move_locked_path(
            plan["approach1_path"],
            core.REAL_APPROACH1_SEC,
            "AUTO APPROACH1 -> NEAR",
        ):
            return False

        # Re-plan from measured NEAR q exactly as v11 does.
        real_node.refresh()

        if real_node.current_q is None:
            print(
                "[BLOCK] Cannot read actual NEAR q"
            )
            return False

        actual_near_q = (
            real_node.current_q.copy()
        )

        baseline = (
            core.load_baseline_ik_module()
        )

        limits = (
            baseline.load_joint_limits(
                core.URDF_PATH
            )
        )

        real_approach2_path = (
            core.solve_pitch_cartesian_path(
                baseline,
                "AUTO REAL APPROACH2",
                plan["near_xyz"],
                plan["grasp_xyz"],
                actual_near_q,
                limits,
                core.APPROACH2_IK_STEP_M,
                plan["approach_pitch_rad"],
                plan["approach_pitch_rad"],
            )
        )

        if real_approach2_path is None:
            print(
                "[BLOCK] Actual NEAR -> GRASP IK failed"
            )
            return False

        real_q_grasp = list(
            real_approach2_path[-1]
        )

        real_lift_path = (
            core.solve_pitch_cartesian_path(
                baseline,
                "AUTO REAL LIFT",
                plan["grasp_xyz"],
                plan["lift_xyz"],
                real_q_grasp,
                limits,
                core.LIFT_IK_STEP_M,
                plan["approach_pitch_rad"],
                plan["approach_pitch_rad"],
            )
        )

        if real_lift_path is None:
            print(
                "[BLOCK] Actual LIFT IK failed"
            )
            return False

        if not executor.move_locked_path(
            real_approach2_path,
            core.REAL_APPROACH2_SEC,
            "AUTO APPROACH2: NEAR -> GRASP",
        ):
            return False

        print(
            f"[GRIPPER] CLOSE "
            f"{core.GRIPPER_CLOSE_GAP_CM:.2f} cm"
        )

        if not core.send_gripper_protocol(
            real_node,
            core.GRIPPER_CLOSE_PROTOCOL_DEG,
        ):
            return False

        if not executor.move_locked_path(
            real_lift_path,
            core.REAL_GRASP_TO_LIFT_SEC,
            "AUTO LIFT",
        ):
            return False

        print()
        print(
            "[ARM] RETURN HOME WITH OBJECT"
        )

        if not executor.move_poly5(
            core.HOME_Q,
            RETURN_HOME_SEC,
            "AUTO LIFT -> HOME",
        ):
            return False

        print()
        print("=" * 76)
        print(
            " AUTO GRASP COMPLETE — OBJECT HELD AT HOME"
        )
        print("=" * 76)

        # Gripper intentionally remains CLOSED.
        return True

    finally:
        executor.close()


# ============================================================================
# Autonomous ROS node
# ============================================================================

class AutoGraspPrototype1(Node):
    def __init__(self) -> None:
        super().__init__(
            "drok_auto_grasp_prototype1"
        )

        self.core = load_v11_core()

        # Fixed-practice mode does not need a live camera TF tree.
        self.tf_buffer = None
        self.tf_listener = None

        if not USE_FIXED_PRACTICE_TARGET:
            self.tf_buffer = Buffer()

            self.tf_listener = TransformListener(
                self.tf_buffer,
                self,
            )

        self.status_pub = self.create_publisher(
            String,
            STATUS_TOPIC,
            10,
        )

        self.done_pub = self.create_publisher(
            Bool,
            DONE_TOPIC,
            10,
        )

        # YOLO subscriptions are created only in CAMERA mode.
        if not USE_FIXED_PRACTICE_TARGET:
            self.create_subscription(
                String,
                YOLO_CLASS_TOPIC,
                self.class_callback,
                20,
            )

            self.create_subscription(
                Vector3Stamped,
                YOLO_XYZ_TOPIC,
                self.xyz_callback,
                20,
            )

        self.create_subscription(
            Bool,
            AUTO_ENABLE_TOPIC,
            self.enable_callback,
            10,
        )

        self.enabled = bool(
            START_ENABLED
        )

        self.busy = False
        self.mission_complete = False

        self.last_class = ""
        self.last_class_monotonic = -1.0

        self.samples: List[
            Tuple[
                float,
                List[float],
            ]
        ] = []

        self.cooldown_until = 0.0

        self.worker: Optional[
            threading.Thread
        ] = None

        self.publish_done(
            False
        )

        self.publish_status(
            (
                "FIXED_TARGET_START_PENDING"
                if (
                    self.enabled
                    and USE_FIXED_PRACTICE_TARGET
                )
                else (
                    "ARMED_WAITING"
                    if self.enabled
                    else "DISABLED"
                )
            )
        )

        print()
        print("=" * 76)
        print(
            " DROK AUTO GRASP PROTOTYPE 1"
        )
        print("=" * 76)

        print(
            "TARGET MODE       : "
            + (
                "FIXED PRACTICE"
                if USE_FIXED_PRACTICE_TARGET
                else "CAMERA / YOLO XYZ"
            )
        )

        if USE_FIXED_PRACTICE_TARGET:
            print(
                "Fixed target [ARM_BASE_LINK m]: "
                f"({FIXED_GRASP_X_M:+.4f}, "
                f"{FIXED_GRASP_Y_M:+.4f}, "
                f"{FIXED_GRASP_Z_M:+.4f})"
            )

            print(
                f"IK mode trigger   : "
                f"{AUTO_ENABLE_TOPIC}=True"
            )

            print(
                "YOLO XYZ / TF     : IGNORED"
            )

        else:
            print(
                f"TARGET_CLASS      : "
                f"{TARGET_CLASS}"
            )

            print(
                f"YOLO class topic  : "
                f"{YOLO_CLASS_TOPIC}"
            )

            print(
                f"YOLO xyz topic    : "
                f"{YOLO_XYZ_TOPIC}"
            )

            print(
                f"TF                : "
                f"{DEFAULT_CAMERA_FRAME} "
                f"-> {ROBOT_TARGET_FRAME}"
            )

            print()
            print(
                "USER OFFSETS [cm]"
            )

            print(
                f"  forward : "
                f"{ROBOT_OFFSET_FORWARD_CM:+.2f}"
            )

            print(
                f"  right   : "
                f"{ROBOT_OFFSET_RIGHT_CM:+.2f}"
            )

            print(
                f"  up      : "
                f"{ROBOT_OFFSET_UP_CM:+.2f}"
            )

        print()
        print(
            "MuJoCo preview: DISABLED"
        )

        print(
            "Mission start: EXACT HOME_Q -> FULL OPEN -> IK CHECK -> GRASP"
        )

        print(
            f"Start HOME duration: "
            f"{START_HOME_SEC:.1f} s"
        )

        print(
            f"Auto start enabled: "
            f"{self.enabled}"
        )

        print("=" * 76)

        self._startup_fixed_timer = None

        if (
            self.enabled
            and USE_FIXED_PRACTICE_TARGET
        ):
            # Start after rclpy.spin() begins, not inside __init__.
            self._startup_fixed_timer = self.create_timer(
                0.5,
                self._start_fixed_once_after_launch,
            )

    # ----------------------------------------------------------------------
    # Status
    # ----------------------------------------------------------------------

    def publish_status(
        self,
        text: str,
    ) -> None:
        msg = String()
        msg.data = text

        self.status_pub.publish(
            msg
        )

        print(
            f"[STATUS] {text}"
        )

    def publish_done(
        self,
        value: bool,
    ) -> None:
        msg = Bool()
        msg.data = bool(value)

        self.done_pub.publish(
            msg
        )

    # ----------------------------------------------------------------------
    # Enable / chassis integration placeholder
    # ----------------------------------------------------------------------

    def _fixed_target_xyz(
        self,
    ) -> List[float]:
        """
        Fixed practice target + user offset.

        This makes ROBOT_OFFSET_FORWARD_CM / RIGHT_CM / UP_CM useful in
        BOTH fixed-practice mode and future camera mode.

        ARM_BASE_LINK:
          +X = forward
          +Y = left
          +Z = up

        User offset convention:
          FORWARD + -> X increases
          RIGHT   + -> Y decreases
          UP      + -> Z increases
        """
        return self.apply_user_offset(
            [
                float(FIXED_GRASP_X_M),
                float(FIXED_GRASP_Y_M),
                float(FIXED_GRASP_Z_M),
            ]
        )

    def _start_target_mission(
        self,
        target_xyz: Sequence[float],
        source_label: str,
    ) -> bool:
        if self.busy:
            self.publish_status(
                "BUSY_IGNORE_NEW_IK_MODE"
            )
            return False

        if self.mission_complete:
            self.mission_complete = False

        target = [
            float(target_xyz[0]),
            float(target_xyz[1]),
            float(target_xyz[2]),
        ]

        if not self.in_workspace(
            target
        ):
            self.publish_status(
                "TARGET_OUTSIDE_WORKSPACE"
            )

            self.get_logger().error(
                f"{source_label} target outside workspace: "
                + format_xyz(target)
            )
            return False

        print()
        print("=" * 76)
        print(
            f" {source_label} GRASP TARGET"
        )
        print("=" * 76)
        print(
            f"ARM_BASE_LINK target: {format_xyz(target)}"
        )
        print("=" * 76)

        self.samples.clear()
        self.busy = True

        self.publish_done(
            False
        )

        self.publish_status(
            "IK_CHECK:"
            + source_label
        )

        self.worker = threading.Thread(
            target=self.run_grasp_worker,
            args=(target,),
            daemon=True,
        )

        self.worker.start()
        return True

    def _start_fixed_once_after_launch(
        self,
    ) -> None:
        if self._startup_fixed_timer is not None:
            self._startup_fixed_timer.cancel()
            self._startup_fixed_timer = None

        if (
            not self.enabled
            or self.busy
            or not USE_FIXED_PRACTICE_TARGET
        ):
            return

        self._start_target_mission(
            self._fixed_target_xyz(),
            "FIXED_PRACTICE",
        )

    def enable_callback(
        self,
        msg: Bool,
    ) -> None:
        requested = bool(
            msg.data
        )

        if requested:
            if self.busy:
                self.publish_status(
                    "BUSY_IGNORE_ENABLE"
                )
                return

            self.enabled = True
            self.mission_complete = False
            self.samples.clear()

            self.publish_done(
                False
            )

            if USE_FIXED_PRACTICE_TARGET:
                # IK mode ON = immediately grasp the configured practice point.
                self.publish_status(
                    "IK_MODE_ON_FIXED_TARGET"
                )

                self._start_target_mission(
                    self._fixed_target_xyz(),
                    "FIXED_PRACTICE",
                )

            else:
                # Camera mode waits for a stable target from YOLO/TF.
                self.publish_status(
                    "ARMED_WAITING_CAMERA_TARGET"
                )

        else:
            # Never interrupt a physical grasp halfway through.
            if self.busy:
                self.publish_status(
                    "BUSY_IGNORE_DISABLE"
                )
                return

            self.enabled = False
            self.samples.clear()

            self.publish_status(
                "DISABLED"
            )

    # ----------------------------------------------------------------------
    # YOLO topic association
    # ----------------------------------------------------------------------

    def class_callback(
        self,
        msg: String,
    ) -> None:
        if USE_FIXED_PRACTICE_TARGET:
            return

        self.last_class = (
            msg.data
            .strip()
            .lower()
        )

        self.last_class_monotonic = (
            time.monotonic()
        )

    def xyz_callback(
        self,
        msg: Vector3Stamped,
    ) -> None:
        if USE_FIXED_PRACTICE_TARGET:
            return

        now = time.monotonic()

        if not self.enabled:
            return

        if self.busy:
            return

        if self.mission_complete:
            return

        if now < self.cooldown_until:
            return

        # Current YOLO code sends class and xyz on separate topics.
        # Gate XYZ by the most recent class.
        class_age = (
            now
            - self.last_class_monotonic
        )

        if (
            self.last_class != TARGET_CLASS
            or class_age
            > CLASS_XYZ_ASSOCIATION_MAX_SEC
        ):
            return

        try:
            base_xyz = (
                self.transform_xyz_to_robot(
                    msg
                )
            )
        except Exception as exc:
            self.get_logger().warning(
                f"TF reject: {exc}"
            )
            return

        corrected_xyz = (
            self.apply_user_offset(
                base_xyz
            )
        )

        if not self.in_workspace(
            corrected_xyz
        ):
            self.get_logger().warning(
                "Target outside prototype workspace: "
                + format_xyz(
                    corrected_xyz
                )
            )
            return

        self.add_sample(
            corrected_xyz,
            now,
        )

    # ----------------------------------------------------------------------
    # TF + offset
    # ----------------------------------------------------------------------

    def transform_xyz_to_robot(
        self,
        msg: Vector3Stamped,
    ) -> List[float]:
        source_frame = (
            msg.header.frame_id.strip()
            or DEFAULT_CAMERA_FRAME
        )

        if (
            source_frame
            == ROBOT_TARGET_FRAME
        ):
            return [
                float(msg.vector.x),
                float(msg.vector.y),
                float(msg.vector.z),
            ]

        tf_time = (
            Time()
            if USE_LATEST_TF
            else Time.from_msg(
                msg.header.stamp
            )
        )

        if self.tf_buffer is None:
            raise RuntimeError(
                "TF buffer is disabled in fixed-practice mode"
            )

        try:
            transform = (
                self.tf_buffer.lookup_transform(
                    ROBOT_TARGET_FRAME,
                    source_frame,
                    tf_time,
                    timeout=Duration(
                        seconds=TF_TIMEOUT_SEC
                    ),
                )
            )
        except TransformException as exc:
            raise RuntimeError(
                f"no TF {source_frame} -> "
                f"{ROBOT_TARGET_FRAME}: {exc}"
            )

        translation = (
            transform.transform.translation
        )

        rotation = (
            transform.transform.rotation
        )

        # IMPORTANT:
        # Although the incoming ROS type is Vector3Stamped, the supplied YOLO
        # node is using vector.x/y/z as an OBJECT POSITION. Therefore TF
        # translation MUST be applied, not only rotation.
        rotated = quaternion_rotate(
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
            [
                msg.vector.x,
                msg.vector.y,
                msg.vector.z,
            ],
        )

        return [
            rotated[0]
            + float(translation.x),
            rotated[1]
            + float(translation.y),
            rotated[2]
            + float(translation.z),
        ]

    @staticmethod
    def apply_user_offset(
        xyz: Sequence[float],
    ) -> List[float]:
        return [
            float(xyz[0])
            + ROBOT_OFFSET_FORWARD_CM
            / 100.0,

            float(xyz[1])
            - ROBOT_OFFSET_RIGHT_CM
            / 100.0,

            float(xyz[2])
            + ROBOT_OFFSET_UP_CM
            / 100.0,
        ]

    @staticmethod
    def in_workspace(
        xyz: Sequence[float],
    ) -> bool:
        x, y, z = xyz

        return (
            WORKSPACE_X_MIN_M
            <= x
            <= WORKSPACE_X_MAX_M

            and

            WORKSPACE_Y_MIN_M
            <= y
            <= WORKSPACE_Y_MAX_M

            and

            WORKSPACE_Z_MIN_M
            <= z
            <= WORKSPACE_Z_MAX_M
        )

    # ----------------------------------------------------------------------
    # Sample stabilisation
    # ----------------------------------------------------------------------

    def add_sample(
        self,
        xyz: Sequence[float],
        now: float,
    ) -> None:
        self.samples.append(
            (
                now,
                [
                    float(xyz[0]),
                    float(xyz[1]),
                    float(xyz[2]),
                ],
            )
        )

        self.samples = [
            item
            for item in self.samples
            if now - item[0]
            <= SAMPLE_WINDOW_SEC
        ]

        if len(self.samples) < REQUIRED_SAMPLES:
            print(
                f"\r[DETECTION] "
                f"{len(self.samples)}/"
                f"{REQUIRED_SAMPLES} "
                f"{format_xyz(xyz)}",
                end="",
                flush=True,
            )
            return

        points = [
            item[1]
            for item in self.samples
        ]

        target = median_xyz(
            points
        )

        max_deviation = max(
            euclidean_distance(
                sample,
                target,
            )
            for sample in points
        )

        if (
            max_deviation
            > MAX_MEDIAN_DEVIATION_M
        ):
            print()

            self.get_logger().warning(
                f"Detection unstable: "
                f"max deviation="
                f"{max_deviation*100.0:.1f} cm"
            )

            # Keep only recent half and continue collecting.
            keep = max(
                1,
                REQUIRED_SAMPLES // 2,
            )

            self.samples = (
                self.samples[-keep:]
            )

            return

        print()
        print()
        print("=" * 76)
        print(
            " STABLE OBJECT TARGET"
        )
        print("=" * 76)

        print(
            "ARM_BASE_LINK + user offset:"
        )

        print(
            " ",
            format_xyz(
                target
            ),
        )

        print(
            f"sample max deviation = "
            f"{max_deviation*100.0:.2f} cm"
        )

        print("=" * 76)

        self.samples.clear()

        self._start_target_mission(
            target,
            "CAMERA",
        )

    # ----------------------------------------------------------------------
    # Mission
    # ----------------------------------------------------------------------

    def run_grasp_worker(
        self,
        target_xyz: Sequence[float],
    ) -> None:
        real_node = None

        try:
            real_node = (
                self.core.RealFeedbackNode()
            )

            if not real_node.wait_for_feedback():
                raise RuntimeError(
                    "No real arm feedback"
                )

            self.publish_status(
                "START_HOME_AND_GRIPPER_OPEN"
            )

            if not move_real_arm_to_exact_home(
                self.core,
                real_node,
                START_HOME_SEC,
            ):
                self.publish_status(
                    "START_PREPARATION_FAILED"
                )

                self.cooldown_until = (
                    time.monotonic()
                    + FAILURE_COOLDOWN_SEC
                )

                return

            self.publish_status(
                "HOME_AND_GRIPPER_READY_IK_CHECK"
            )

            plan = build_topdown_plan(
                self.core,
                real_node,
                target_xyz,
            )

            if not plan["ok"]:
                self.publish_status(
                    "IK_UNREACHABLE:"
                    + str(
                        plan["failed"]
                    )
                )

                self.cooldown_until = (
                    time.monotonic()
                    + FAILURE_COOLDOWN_SEC
                )

                return

            self.publish_status(
                "IK_REACHABLE_EXECUTING"
            )

            ok = (
                execute_plan_and_return_home(
                    self.core,
                    real_node,
                    plan,
                )
            )

            if not ok:
                self.publish_status(
                    "EXECUTION_FAILED"
                )

                self.cooldown_until = (
                    time.monotonic()
                    + FAILURE_COOLDOWN_SEC
                )

                return

            # Arm auto mission is complete.
            self.mission_complete = True
            self.enabled = False

            self.publish_done(
                True
            )

            self.publish_status(
                "GRASP_COMPLETE_HOME_ARM_MODE_OFF"
            )

            print()
            print(
                "[AUTO MODE] Arm auto mission OFF."
            )

            print(
                "The chassis/autonomous-driving node can "
                "resume when it receives DONE=True."
            )

        except Exception as exc:
            self.get_logger().error(
                f"Auto grasp exception: {exc}"
            )

            self.publish_status(
                "ERROR:"
                + str(exc)
            )

            self.cooldown_until = (
                time.monotonic()
                + FAILURE_COOLDOWN_SEC
            )

        finally:
            if real_node is not None:
                try:
                    real_node.destroy_node()
                except Exception:
                    pass

            self.busy = False


def main() -> int:
    rclpy.init()

    node = None

    try:
        node = (
            AutoGraspPrototype1()
        )

        rclpy.spin(
            node
        )

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
    raise SystemExit(
        main()
    )
