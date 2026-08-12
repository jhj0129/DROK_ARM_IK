#!/usr/bin/env python3
"""
DROK interactive box IK grasp — v11
===================================

Main changes from v2
--------------------
1) IK target is referenced from the FIXED HOME TCP, not the current TCP.
2) After IK succeeds, the full grasp sequence is previewed in MuJoCo at 3x speed.
3) Real motion starts only after the preview finishes and the user types GRASP.
4) Arm real execution no longer trusts the old action result as "arrived".
   It sends RMD A4 commands directly using the SAME real_mapping.yaml conversion,
   computes a maxSpeed that can actually follow the requested Poly5 duration,
   and waits for /joint_states to physically reach each target before continuing.
5) Current gripper calibration:
      OPEN 14.60 cm = protocol -1640.890 deg
      CLOSE 9.70 cm = protocol -545.910 deg

No CAN interface state/bitrate changes.
No motor ROM/limit writes.
"""

from __future__ import annotations

import concurrent.futures
import importlib.util
import math
import os
import re
import socket
import select
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

# MuJoCo is optional for the CURRENT real/manual grasp pipeline.
# The automatic/manual pipeline never calls preview_plan_mujoco().
# Keep preview support available only when MuJoCo is installed.
try:
    import mujoco
    import mujoco.viewer
except ImportError:
    mujoco = None

import rclpy
import yaml
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


# ============================================================
# Paths — single standalone DROK_ARM_IK workspace
# ============================================================

HOME = Path.home()
SCRIPT_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = SCRIPT_DIR.parent

KINEMATICS_SOURCE_DIR = (
    WORKSPACE_ROOT
    / "src"
    / "drok_arm_kinematics"
)

GEOMETRY_PATH = (
    KINEMATICS_SOURCE_DIR
    / "config"
    / "robot_geometry.yaml"
)

URDF_PATH = (
    KINEMATICS_SOURCE_DIR
    / "config"
    / "drok_arm_kinematics_only.urdf"
)

BASELINE_IK_PATH = (
    SCRIPT_DIR
    / "baseline_nearest_ik_core.py"
)

IK_SOLVER_EXE = (
    WORKSPACE_ROOT
    / "install"
    / "drok_arm_kinematics"
    / "lib"
    / "drok_arm_kinematics"
    / "solve_ik_pose"
)

FK_EXE = (
    WORKSPACE_ROOT
    / "install"
    / "drok_arm_kinematics"
    / "lib"
    / "drok_arm_kinematics"
    / "test_fk"
)

REAL_WS = WORKSPACE_ROOT

REAL_MAPPING_CANDIDATES = [
    (
        REAL_WS
        / "src"
        / "drok_real_arm_bridge"
        / "config"
        / "real_mapping.yaml"
    ),
    (
        REAL_WS
        / "install"
        / "drok_real_arm_bridge"
        / "share"
        / "drok_real_arm_bridge"
        / "config"
        / "real_mapping.yaml"
    ),
]

# Legacy preview only. It is not used by the automatic/fixed real grasp.
MJCF_CANDIDATES = [
    (
        REAL_WS
        / "runtime"
        / "existing_mujoco_environment"
        / "drok_arm_complete_environment.xml"
    ),
]


# ============================================================
# Robot / coordinates
# ============================================================

JOINTS = [
    "JOINT1",
    "JOINT2",
    "JOINT3",
    "JOINT4",
    "JOINT5",
    "JOINT6",
]

JOINT_STATE_TOPIC = "/joint_states"

# Physically verified logical HOME.
HOME_Q = [
    -0.000001628,
    +0.297361544,
    +0.296742637,
    -0.000030712,
    +0.000061231,
    +0.000102331,
]

# User input is relative to HOME TCP.
DEFAULT_FORWARD_CM = 30.0
DEFAULT_LATERAL_CM = 0.0
DEFAULT_DOWN_CM = 20.0

# Two-stage grasp geometry.
#
# v9 orientation policy:
#
#   roll  = 0 deg  (remove side twist)
#   yaw   = 0 deg  (keep gripper left/right faces parallel to box faces)
#   pitch = AUTO   (allow the wrist to naturally tilt toward the box)
#
# APPROACH1:
#   HOME TCP -> NEAR (9 cm above box)
#   pitch changes smoothly from the HOME TCP pitch to the final approach pitch.
#
# APPROACH2:
#   NEAR -> GRASP
#   pitch is held at the approach-direction pitch.
#
# For the normal vertical final approach:
#   NEAR -> GRASP = world -Z
# and, with this project's TCP +X approach convention,
#   pitch = +90 deg.
#
# This fixes v8's unnatural behavior where pitch was also forced to zero.
NEAR_STANDOFF_M = 0.09
LIFT_HEIGHT_M = 0.05

# Roll/yaw stay aligned to ARM_BASE_LINK.
LOCK_ROLL_RAD = 0.0
LOCK_YAW_RAD = 0.0

# Requested relaxed FULL-pose convergence tolerances.
# The target pitch is still explicitly calculated at every waypoint.
IK_POSITION_TOLERANCE_M = 0.01
IK_ORIENTATION_TOLERANCE_DEG = 10.0

MIN_JOINT_LIMIT_MARGIN_RAD = 0.05

# Cartesian IK waypoint spacing.
APPROACH1_IK_STEP_M = 0.015   # 1.5 cm
APPROACH2_IK_STEP_M = 0.0075  # 7.5 mm
LIFT_IK_STEP_M = 0.010        # 1 cm

# If the geometrically ideal pitch (+/-90 deg for a vertical approach)
# drives a wrist joint too close to its limit, automatically back off the
# pitch while keeping Roll=0 and Yaw=0.
#
# Search order for a +90 deg request:
#   90, 85, 80, 75, 70, 65, 60 deg
#
# The first pitch whose APPROACH1 + APPROACH2 + LIFT paths all satisfy
# IK and the existing software joint-limit margin is selected.
PITCH_BACKOFF_STEP_DEG = 5.0
PITCH_MIN_ABS_DEG = 60.0


# ============================================================
# Trajectory / preview
# ============================================================

# At HOME TCP, first cancel roll/yaw mounting tilt while preserving HOME pitch.
REAL_CURRENT_TO_PREALIGN_SEC = 1.2

# Then move while pitch changes naturally.
REAL_APPROACH1_SEC = 6.0*2
REAL_APPROACH2_SEC = 3.0*2
REAL_GRASP_TO_LIFT_SEC = 3.0*2
REAL_GRIPPER_CLOSE_SEC = 3.0*2

PREVIEW_SPEEDUP = 3.0
PREVIEW_RATE_HZ = 60.0
REAL_COMMAND_RATE_HZ = 50.0

# Poly5 max normalized velocity is 1.875 / T.
POLY5_MAX_DS = 1.875

# Small headroom over the theoretical Poly5 peak speed.
ARM_SPEED_MARGIN = 1.15

# Conservative protocol maxSpeed cap for this debug executor.
ARM_MIN_PROTOCOL_SPEED_DPS = 20
ARM_MAX_PROTOCOL_SPEED_DPS = 5000

# Physical-arrival confirmation.
ARM_ARRIVAL_TOL_RAD = math.radians(1.0)
ARM_ARRIVAL_TIMEOUT_SEC = 8.0


# ============================================================
# Real gripper empirical calibration
# ============================================================

GRIPPER_CAN_IFACE = "can11"
GRIPPER_CAN_ID = 0x144
CAN_FRAME_FMT = "=IB3x8s"

GRIPPER_FEEDBACK_TOPIC = (
    "/motor_angles/can11_motor_0x144"
)

GRIPPER_OPEN_GAP_CM = 14.600000000
GRIPPER_OPEN_PROTOCOL_DEG = 105.110000000

GRIPPER_CLOSE_GAP_CM = 9.700000000
GRIPPER_CLOSE_PROTOCOL_DEG = 1172.960000000

# Current publisher exposes protocol / 6.
GRIPPER_TOPIC_DIVISOR = 6.0

GRIPPER_OPEN_TOPIC_DEG = (
    GRIPPER_OPEN_PROTOCOL_DEG
    / GRIPPER_TOPIC_DIVISOR
)

GRIPPER_CLOSE_TOPIC_DEG = (
    GRIPPER_CLOSE_PROTOCOL_DEG
    / GRIPPER_TOPIC_DIVISOR
)

GRIPPER_SPEED_DPS = 449
GRIPPER_WAIT_TIMEOUT_SEC = 4.5
GRIPPER_TOPIC_TOLERANCE_DEG = 0.35

# MuJoCo symmetric finger travel for 14.5 -> 9.0 cm gap.
MJ_GRIPPER_EACH_TRAVEL_M = 0.0275


# ============================================================
# Helpers
# ============================================================

RAD2DEG = 180.0 / math.pi


def poly5(s: float) -> float:
    s = max(0.0, min(1.0, s))
    return (
        10.0 * s**3
        - 15.0 * s**4
        + 6.0 * s**5
    )


def find_existing(paths: Sequence[Path], label: str) -> Path:
    for path in paths:
        if path.is_file():
            return path

    raise FileNotFoundError(
        f"{label} 파일을 찾지 못했습니다:\n"
        + "\n".join(
            f"  {path}"
            for path in paths
        )
    )


def format_xyz(xyz: Sequence[float]) -> str:
    return (
        "("
        + ", ".join(
            f"{value:+.4f}"
            for value in xyz
        )
        + ")"
    )


def format_q_deg(q: Sequence[float]) -> str:
    return (
        "["
        + ", ".join(
            f"{math.degrees(value):+8.2f}"
            for value in q
        )
        + "]"
    )


def read_float(label: str, default: float) -> float:
    while True:
        text = input(
            f"{label} [Enter={default:g}]: "
        ).strip()

        if text == "":
            return float(default)

        try:
            value = float(text)
        except ValueError:
            print("숫자로 입력하세요.")
            continue

        if not math.isfinite(value):
            print("유한한 숫자를 입력하세요.")
            continue

        return value


# ============================================================
# Existing project FK / IK
# ============================================================

def load_baseline_ik_module():
    if not BASELINE_IK_PATH.is_file():
        raise FileNotFoundError(
            f"기존 nearest IK 파일 없음:\n"
            f"{BASELINE_IK_PATH}"
        )

    spec = importlib.util.spec_from_file_location(
        "drok_baseline_nearest_ik",
        BASELINE_IK_PATH,
    )

    if (
        spec is None
        or spec.loader is None
    ):
        raise RuntimeError(
            "baseline_nearest_ik_dry_run.py "
            "로드 실패"
        )

    module = importlib.util.module_from_spec(
        spec
    )

    spec.loader.exec_module(module)

    return module


def run_project_fk_pose(
    q: Sequence[float],
) -> Tuple[
    Tuple[float, float, float],
    Tuple[float, float, float],
]:
    """
    Use the project's own test_fk executable and return:
      xyz = (x, y, z)
      rpy = (roll, pitch, yaw)
    """
    if not FK_EXE.is_file():
        raise FileNotFoundError(
            f"IK workspace FK executable not found:\n{FK_EXE}"
        )

    command = [
        str(FK_EXE),
        str(GEOMETRY_PATH),
        *[
            f"{value:.12f}"
            for value in q
        ],
    ]

    result = subprocess.run(
        command,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=8.0,
        check=False,
    )

    if result.returncode != 0:
        raise RuntimeError(
            "기존 DROK FK 실행 실패:\n"
            + result.stdout
        )

    values: Dict[str, float] = {}

    patterns = {
        "x": r"^x\s*=\s*([-+0-9.eE]+)\s*$",
        "y": r"^y\s*=\s*([-+0-9.eE]+)\s*$",
        "z": r"^z\s*=\s*([-+0-9.eE]+)\s*$",
        "roll": r"^roll\s*=\s*([-+0-9.eE]+)\s*$",
        "pitch": r"^pitch\s*=\s*([-+0-9.eE]+)\s*$",
        "yaw": r"^yaw\s*=\s*([-+0-9.eE]+)\s*$",
    }

    for key, pattern in patterns.items():
        match = re.search(
            pattern,
            result.stdout,
            flags=re.MULTILINE,
        )

        if match is not None:
            values[key] = float(
                match.group(1)
            )

    if not all(
        key in values
        for key in (
            "x",
            "y",
            "z",
            "roll",
            "pitch",
            "yaw",
        )
    ):
        raise RuntimeError(
            "test_fk 출력에서 TCP pose를 읽지 못했습니다.\n"
            + result.stdout
        )

    xyz = (
        values["x"],
        values["y"],
        values["z"],
    )

    rpy = (
        values["roll"],
        values["pitch"],
        values["yaw"],
    )

    return xyz, rpy


def run_project_fk(
    q: Sequence[float],
) -> Tuple[float, float, float]:
    xyz, _ = run_project_fk_pose(q)
    return xyz


def wrap_to_pi(
    angle_rad: float,
) -> float:
    return (
        angle_rad
        + math.pi
    ) % (
        2.0 * math.pi
    ) - math.pi


def canonicalize_zyx_rpy(
    rpy: Sequence[float],
) -> Tuple[float, float, float]:
    """
    Canonicalize an equivalent ZYX roll-pitch-yaw representation so that:

        pitch in [-90 deg, +90 deg]

    Example from the current DROK HOME FK:

        raw:
          [ +155.99, +179.96, +180.00 ] deg

        equivalent canonical:
          [  -24.01,   +0.04,    0.00 ] deg

    These represent the SAME rotation matrix.

    This prevents us from taking only the raw Euler 'pitch=179.96 deg'
    and accidentally constructing a totally different target orientation
    such as [roll=0, pitch=179.96, yaw=0].
    """
    roll = wrap_to_pi(
        float(rpy[0])
    )

    pitch = wrap_to_pi(
        float(rpy[1])
    )

    yaw = wrap_to_pi(
        float(rpy[2])
    )

    if pitch > (
        math.pi / 2.0
    ):
        roll = wrap_to_pi(
            roll + math.pi
        )

        pitch = (
            math.pi - pitch
        )

        yaw = wrap_to_pi(
            yaw + math.pi
        )

    elif pitch < (
        -math.pi / 2.0
    ):
        roll = wrap_to_pi(
            roll + math.pi
        )

        pitch = (
            -math.pi - pitch
        )

        yaw = wrap_to_pi(
            yaw + math.pi
        )

    # Clean tiny numerical values for easier diagnostics.
    eps = 1.0e-10

    if abs(roll) < eps:
        roll = 0.0

    if abs(pitch) < eps:
        pitch = 0.0

    if abs(yaw) < eps:
        yaw = 0.0

    return (
        roll,
        pitch,
        yaw,
    )


def shortest_angle_delta(
    start_rad: float,
    end_rad: float,
) -> float:
    """
    Shortest signed angular change from start to end.

    Used for pitch interpolation so Euler wrapping never creates
    an unnecessary ~360 degree motion.
    """
    return wrap_to_pi(
        float(end_rad)
        - float(start_rad)
    )


def approach_pitch_from_vector(
    start_xyz: Sequence[float],
    end_xyz: Sequence[float],
) -> float:
    """
    Compute TCP pitch so local +X points along the X-Z projection
    of the approach vector.

    R_y(pitch) * +X = [cos(pitch), 0, -sin(pitch)]

    Therefore:
      pitch = atan2(-dz, dx)

    Examples:
      +X direction ->   0 deg
      world -Z     -> +90 deg
    """
    dx = (
        float(end_xyz[0])
        - float(start_xyz[0])
    )

    dz = (
        float(end_xyz[2])
        - float(start_xyz[2])
    )

    if (
        abs(dx) < 1.0e-9
        and abs(dz) < 1.0e-9
    ):
        raise ValueError(
            "approach pitch를 계산할 수 없는 0-length XZ vector"
        )

    return math.atan2(
        -dz,
        dx,
    )


def make_roll_yaw_locked_rpy(
    pitch_rad: float,
) -> List[float]:
    return [
        LOCK_ROLL_RAD,
        float(pitch_rad),
        LOCK_YAW_RAD,
    ]


def joint_limit_margin(
    q: Sequence[float],
    limits,
):
    margins = []

    for joint, value in zip(
        JOINTS,
        q,
    ):
        lower, upper = limits[joint]

        margin = min(
            value - lower,
            upper - value,
        )

        margins.append(
            (
                joint,
                margin,
            )
        )

    return min(
        margins,
        key=lambda item: item[1],
    )


class FullPoseAttempt:
    def __init__(
        self,
        q: Optional[List[float]],
        solver_success: bool,
        position_error_m: Optional[float],
        orientation_error_rad: Optional[float],
        stdout: str,
        returncode: int,
    ) -> None:
        self.q = q
        self.solver_success = solver_success
        self.position_error_m = position_error_m
        self.orientation_error_rad = orientation_error_rad
        self.stdout = stdout
        self.returncode = returncode

    @property
    def relaxed_success(self) -> bool:
        if (
            self.q is None
            or self.position_error_m is None
            or self.orientation_error_rad is None
        ):
            return False

        return (
            self.position_error_m
            <= IK_POSITION_TOLERANCE_M
            and self.orientation_error_rad
            <= math.radians(
                IK_ORIENTATION_TOLERANCE_DEG
            )
        )


def _parse_solver_float(
    pattern: str,
    stdout: str,
) -> Optional[float]:
    match = re.search(
        pattern,
        stdout,
        flags=re.IGNORECASE,
    )

    if match is None:
        return None

    try:
        value = float(
            match.group(1)
        )
    except ValueError:
        return None

    if not math.isfinite(value):
        return None

    return value


def solve_one_seed_full_pose(
    position: Sequence[float],
    rpy: Sequence[float],
    seed: Sequence[float],
) -> FullPoseAttempt:
    """
    Run the existing C++ full-pose DLS solver exactly as-is.

    IMPORTANT:
    We do NOT patch the C++ tolerance anymore.

    The C++ solver still tries to converge to its original strict tolerance.
    After it finishes, Python reads:
      Position error
      Orientation error
      JOINT_RESULT

    and accepts the final solution when:
      position error <= 1 cm
      orientation error <= 10 deg

    This means the requested target remains exactly:
      RPY = (0, 0, 0)

    and 1 cm / 10 deg are only acceptance tolerances.
    """
    if not IK_SOLVER_EXE.is_file():
        return FullPoseAttempt(
            q=None,
            solver_success=False,
            position_error_m=None,
            orientation_error_rad=None,
            stdout=(
                "IK solver executable not found: "
                f"{IK_SOLVER_EXE}"
            ),
            returncode=127,
        )

    x, y, z = position
    roll, pitch, yaw = rpy

    command = [
        str(IK_SOLVER_EXE),
        str(GEOMETRY_PATH),
        f"{x:.12f}",
        f"{y:.12f}",
        f"{z:.12f}",
        f"{roll:.12f}",
        f"{pitch:.12f}",
        f"{yaw:.12f}",
        *[
            f"{value:.12f}"
            for value in seed
        ],
    ]

    environment = os.environ.copy()
    environment["DROK_IK_MODE"] = "full"

    try:
        result = subprocess.run(
            command,
            env=environment,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=15.0,
            check=False,
        )
    except subprocess.TimeoutExpired:
        return FullPoseAttempt(
            q=None,
            solver_success=False,
            position_error_m=None,
            orientation_error_rad=None,
            stdout="IK timeout",
            returncode=124,
        )

    success_match = re.search(
        r"Success\s*:\s*(true|false)",
        result.stdout,
        flags=re.IGNORECASE,
    )

    solver_success = (
        success_match is not None
        and success_match.group(1).lower()
        == "true"
    )

    position_error_m = _parse_solver_float(
        r"Position error\s*:\s*([-+0-9.eE]+)",
        result.stdout,
    )

    orientation_error_rad = _parse_solver_float(
        r"Orientation error\s*:\s*([-+0-9.eE]+)",
        result.stdout,
    )

    joint_match = re.search(
        r"JOINT_RESULT=([^\r\n]+)",
        result.stdout,
    )

    q: Optional[List[float]] = None

    if joint_match is not None:
        try:
            q = [
                float(value)
                for value
                in joint_match.group(1).split(",")
            ]
        except ValueError:
            q = None

    if (
        q is not None
        and (
            len(q) != 6
            or not all(
                math.isfinite(value)
                for value in q
            )
        )
    ):
        q = None

    return FullPoseAttempt(
        q=q,
        solver_success=solver_success,
        position_error_m=position_error_m,
        orientation_error_rad=orientation_error_rad,
        stdout=result.stdout,
        returncode=result.returncode,
    )


def solve_all_candidates_full_pose(
    baseline,
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
):
    seeds = baseline.make_seed_set(
        reference_q,
        limits,
    )

    attempts: List[
        FullPoseAttempt
    ] = []

    with concurrent.futures.ThreadPoolExecutor(
        max_workers=4
    ) as executor:
        futures = [
            executor.submit(
                solve_one_seed_full_pose,
                position,
                rpy,
                seed,
            )
            for seed in seeds
        ]

        for future in concurrent.futures.as_completed(
            futures
        ):
            try:
                attempts.append(
                    future.result()
                )
            except Exception:
                pass

    # Keep diagnostics even when nothing is accepted.
    valid_attempts = [
        attempt
        for attempt in attempts
        if attempt.relaxed_success
    ]

    candidates = []

    for attempt in valid_attempts:
        assert attempt.q is not None

        normalized = baseline.normalize_candidate(
            attempt.q,
            reference_q,
            limits,
        )

        if normalized is None:
            continue

        duplicate_index = None

        for index, item in enumerate(
            candidates
        ):
            if baseline.is_duplicate(
                [item["q"]],
                normalized,
            ):
                duplicate_index = index
                break

        normalized_position_error = (
            (
                attempt.position_error_m
                if attempt.position_error_m
                is not None
                else 1.0e9
            )
            / IK_POSITION_TOLERANCE_M
        )

        normalized_orientation_error = (
            (
                attempt.orientation_error_rad
                if attempt.orientation_error_rad
                is not None
                else 1.0e9
            )
            / math.radians(
                IK_ORIENTATION_TOLERANCE_DEG
            )
        )

        residual_score = (
            normalized_position_error
            + normalized_orientation_error
        )

        item = {
            "q": list(normalized),
            "position_error_m": (
                attempt.position_error_m
            ),
            "orientation_error_rad": (
                attempt.orientation_error_rad
            ),
            "solver_success": (
                attempt.solver_success
            ),
            "residual_score": (
                residual_score
            ),
        }

        if duplicate_index is None:
            candidates.append(item)
        else:
            if (
                item["residual_score"]
                <
                candidates[
                    duplicate_index
                ]["residual_score"]
            ):
                candidates[
                    duplicate_index
                ] = item

    # Preserve "nearest solution" as the primary selection criterion,
    # then prefer the smaller final residual among similarly near branches.
    candidates.sort(
        key=lambda item: (
            baseline.candidate_score(
                item["q"],
                reference_q,
            ),
            item["residual_score"],
        )
    )

    # Best numerical residual for diagnostics.
    diagnostic_attempts = [
        attempt
        for attempt in attempts
        if (
            attempt.q is not None
            and attempt.position_error_m
            is not None
            and attempt.orientation_error_rad
            is not None
        )
    ]

    diagnostic_attempts.sort(
        key=lambda attempt: (
            (
                attempt.position_error_m
                / IK_POSITION_TOLERANCE_M
            )
            +
            (
                attempt.orientation_error_rad
                / math.radians(
                    IK_ORIENTATION_TOLERANCE_DEG
                )
            )
        )
    )

    best_attempt = (
        diagnostic_attempts[0]
        if diagnostic_attempts
        else None
    )

    return candidates, best_attempt

def solve_waypoint(
    baseline,
    name: str,
    position: Sequence[float],
    target_rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
):
    print()
    print("=" * 76)
    print(f"[FULL-POSE IK] {name}")
    print("=" * 76)

    print(
        "target xyz :",
        format_xyz(position),
    )

    print(
        "target RPY [deg] :",
        "["
        + ", ".join(
            f"{math.degrees(value):+7.3f}"
            for value in target_rpy
        )
        + "]",
    )

    print(
        "reference q:",
        format_q_deg(reference_q),
    )

    print(
        "DROK_IK_MODE = full"
    )

    print(
        f"Python acceptance position error <= "
        f"{IK_POSITION_TOLERANCE_M*100.0:.2f} cm"
    )

    print(
        f"Python acceptance orientation error <= "
        f"{IK_ORIENTATION_TOLERANCE_DEG:.2f} deg"
    )

    candidates, best_attempt = (
        solve_all_candidates_full_pose(
            baseline,
            position,
            target_rpy,
            reference_q,
            limits,
        )
    )

    print(
        "accepted IK candidates:",
        len(candidates),
    )

    if not candidates:
        if best_attempt is not None:
            assert (
                best_attempt.position_error_m
                is not None
            )
            assert (
                best_attempt.orientation_error_rad
                is not None
            )

            print(
                "[BEST REJECTED RESIDUAL]"
            )

            print(
                f"  position error    = "
                f"{best_attempt.position_error_m*100.0:.3f} cm"
            )

            print(
                f"  orientation error = "
                f"{math.degrees(best_attempt.orientation_error_rad):.3f} deg"
            )

            print(
                f"  requested limit   = "
                f"{IK_POSITION_TOLERANCE_M*100.0:.3f} cm / "
                f"{IK_ORIENTATION_TOLERANCE_DEG:.3f} deg"
            )

        return None

    selected_item = candidates[0]
    selected = list(
        selected_item["q"]
    )

    minimum_joint, margin = (
        joint_limit_margin(
            selected,
            limits,
        )
    )

    print(
        "SELECTED q :",
        format_q_deg(selected),
    )

    print(
        f"selected position error    = "
        f"{selected_item['position_error_m']*100.0:.3f} cm"
    )

    print(
        f"selected orientation error = "
        f"{math.degrees(selected_item['orientation_error_rad']):.3f} deg"
    )

    print(
        "C++ strict solver success =",
        (
            "YES"
            if selected_item[
                "solver_success"
            ]
            else "NO (accepted by 1 cm / 10 deg rule)"
        ),
    )

    deltas = [
        selected_value
        - reference_value
        for selected_value, reference_value
        in zip(
            selected,
            reference_q,
        )
    ]

    print(
        "delta q    :",
        format_q_deg(deltas),
    )

    print(
        f"min limit margin: "
        f"{minimum_joint} "
        f"{math.degrees(margin):.2f} deg"
    )

    if (
        margin
        < MIN_JOINT_LIMIT_MARGIN_RAD
    ):
        print(
            "[BLOCK] joint-limit margin이 "
            f"{math.degrees(MIN_JOINT_LIMIT_MARGIN_RAD):.2f}° "
            "미만"
        )
        return None

    return selected


def solve_continuous_full_pose(
    baseline,
    position: Sequence[float],
    rpy: Sequence[float],
    reference_q: Sequence[float],
    limits,
) -> Optional[List[float]]:
    """
    Continuity-first full-pose solve with relaxed Python acceptance.

    1) Try the previous solution as the seed.
    2) Accept it when residual <= 1 cm / 10 deg.
    3) Otherwise fall back to deterministic multi-start.
    """
    attempt = solve_one_seed_full_pose(
        position,
        rpy,
        reference_q,
    )

    if (
        attempt.relaxed_success
        and attempt.q is not None
    ):
        normalized = baseline.normalize_candidate(
            attempt.q,
            reference_q,
            limits,
        )

        if normalized is not None:
            return list(normalized)

    candidates, _ = (
        solve_all_candidates_full_pose(
            baseline,
            position,
            rpy,
            reference_q,
            limits,
        )
    )

    if not candidates:
        return None

    return list(
        candidates[0]["q"]
    )


def make_pitch_search_candidates(
    requested_pitch_rad: float,
) -> List[float]:
    """
    Keep the requested sign and reduce only the magnitude.

    Example:
      +90 -> +90,+85,+80,...,+60
      -90 -> -90,-85,-80,...,-60

    For a requested magnitude already below PITCH_MIN_ABS_DEG,
    use only the requested value.
    """
    requested_deg = math.degrees(
        float(requested_pitch_rad)
    )

    magnitude = abs(
        requested_deg
    )

    if (
        magnitude
        <= PITCH_MIN_ABS_DEG
        + 1.0e-9
    ):
        return [
            float(requested_pitch_rad)
        ]

    sign = (
        1.0
        if requested_deg >= 0.0
        else -1.0
    )

    candidates: List[float] = []

    current = magnitude

    while (
        current
        >= PITCH_MIN_ABS_DEG
        - 1.0e-9
    ):
        candidates.append(
            math.radians(
                sign * current
            )
        )

        current -= (
            PITCH_BACKOFF_STEP_DEG
        )

    # Ensure the exact minimum is included.
    minimum_candidate = math.radians(
        sign * PITCH_MIN_ABS_DEG
    )

    if not any(
        abs(
            candidate
            - minimum_candidate
        ) < 1.0e-9
        for candidate in candidates
    ):
        candidates.append(
            minimum_candidate
        )

    return candidates


def solve_pitch_cartesian_path(
    baseline,
    name: str,
    start_xyz: Sequence[float],
    end_xyz: Sequence[float],
    start_q: Sequence[float],
    limits,
    step_m: float,
    start_pitch_rad: float,
    end_pitch_rad: float,
) -> Optional[List[List[float]]]:
    """
    Dense full-pose Cartesian IK path.

    At EVERY sample:
      roll = 0
      yaw  = 0
      pitch = smooth interpolation from start_pitch to end_pitch

    This preserves gripper face alignment while allowing the TCP to
    naturally tilt toward the object.
    """
    delta = [
        float(b) - float(a)
        for a, b in zip(
            start_xyz,
            end_xyz,
        )
    ]

    distance = math.sqrt(
        sum(value * value for value in delta)
    )

    segment_count = max(
        1,
        int(
            math.ceil(
                distance
                / max(step_m, 1.0e-6)
            )
        ),
    )

    print()
    print("=" * 76)
    print(f"[ROLL/YAW LOCK + AUTO PITCH PATH] {name}")
    print("=" * 76)

    print(
        "start xyz:",
        format_xyz(start_xyz),
    )

    print(
        "end   xyz:",
        format_xyz(end_xyz),
    )

    print(
        f"distance : {distance*100.0:.2f} cm"
    )

    print(
        f"IK points: {segment_count + 1}"
    )

    print(
        "roll/yaw target = 0 deg / 0 deg"
    )

    print(
        f"pitch target     = "
        f"{math.degrees(start_pitch_rad):+.2f} deg"
        f" -> "
        f"{math.degrees(end_pitch_rad):+.2f} deg"
    )

    path: List[List[float]] = [
        list(start_q)
    ]

    reference_q = list(
        start_q
    )

    for index in range(
        1,
        segment_count + 1
    ):
        s = (
            index
            / segment_count
        )

        blend = poly5(s)

        xyz = [
            float(a)
            + blend
            * (float(b) - float(a))
            for a, b in zip(
                start_xyz,
                end_xyz,
            )
        ]

        pitch_delta = (
            shortest_angle_delta(
                start_pitch_rad,
                end_pitch_rad,
            )
        )

        pitch = wrap_to_pi(
            float(start_pitch_rad)
            + blend
            * pitch_delta
        )

        target_rpy = (
            make_roll_yaw_locked_rpy(
                pitch
            )
        )

        q = solve_continuous_full_pose(
            baseline,
            xyz,
            target_rpy,
            reference_q,
            limits,
        )

        if q is None:
            print()
            print(
                f"[FAIL] {name}: "
                f"Cartesian sample {index}/{segment_count}"
            )

            print(
                "xyz =",
                format_xyz(xyz),
            )

            print(
                f"target RPY [deg] = "
                f"[0.00, {math.degrees(pitch):+.2f}, 0.00]"
            )

            _, best_attempt = (
                solve_all_candidates_full_pose(
                    baseline,
                    xyz,
                    target_rpy,
                    reference_q,
                    limits,
                )
            )

            if best_attempt is not None:
                if (
                    best_attempt.position_error_m
                    is not None
                    and best_attempt.orientation_error_rad
                    is not None
                ):
                    print(
                        "[BEST REJECTED RESIDUAL]"
                    )

                    print(
                        f"  position    = "
                        f"{best_attempt.position_error_m*100.0:.3f} cm"
                    )

                    print(
                        f"  orientation = "
                        f"{math.degrees(best_attempt.orientation_error_rad):.3f} deg"
                    )

                    print(
                        f"  limit       = "
                        f"{IK_POSITION_TOLERANCE_M*100.0:.3f} cm / "
                        f"{IK_ORIENTATION_TOLERANCE_DEG:.3f} deg"
                    )

            return None

        minimum_joint, margin = (
            joint_limit_margin(
                q,
                limits,
            )
        )

        if (
            margin
            < MIN_JOINT_LIMIT_MARGIN_RAD
        ):
            print()
            print(
                f"[BLOCK] {name}: "
                f"{minimum_joint} limit margin "
                f"{math.degrees(margin):.2f} deg"
            )
            return None

        path.append(
            list(q)
        )

        reference_q = list(q)

        print(
            "\r"
            f"[AUTO-PITCH IK] {name}: "
            f"{index:02d}/{segment_count:02d} | "
            f"xyz={format_xyz(xyz)} | "
            f"pitch={math.degrees(pitch):+6.2f} deg",
            end="",
            flush=True,
        )

    print()

    print(
        f"[AUTO-PITCH IK] {name}: SUCCESS"
    )

    return path


# ============================================================
# ROS feedback
# ============================================================

class RealFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__(
            "drok_interactive_box_ik_grasp_v11"
        )

        self.current_q: Optional[
            List[float]
        ] = None

        self.gripper_topic_deg: Optional[
            float
        ] = None

        self.create_subscription(
            JointState,
            JOINT_STATE_TOPIC,
            self._joint_cb,
            30,
        )

        self.create_subscription(
            Float64,
            GRIPPER_FEEDBACK_TOPIC,
            self._gripper_cb,
            30,
        )

    def _joint_cb(
        self,
        msg: JointState,
    ) -> None:
        values = dict(
            zip(
                msg.name,
                msg.position,
            )
        )

        if not all(
            joint in values
            for joint in JOINTS
        ):
            return

        q = [
            float(
                values[joint]
            )
            for joint in JOINTS
        ]

        if all(
            math.isfinite(value)
            for value in q
        ):
            self.current_q = q

    def _gripper_cb(
        self,
        msg: Float64,
    ) -> None:
        value = float(msg.data)

        if math.isfinite(value):
            self.gripper_topic_deg = value

    def wait_for_feedback(
        self,
        timeout_sec: float = 8.0,
    ) -> bool:
        deadline = (
            time.monotonic()
            + timeout_sec
        )

        while (
            rclpy.ok()
            and self.current_q is None
            and time.monotonic()
            < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.1,
            )

        return (
            self.current_q
            is not None
        )

    def refresh(
        self,
        seconds: float = 0.15,
    ) -> None:
        deadline = (
            time.monotonic()
            + seconds
        )

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )


# ============================================================
# Current real_mapping.yaml direct RMD executor
# ============================================================

class DirectArmRmdExecutor:
    """
    Uses the same command conversion as moveit_to_rmd_bridge.py,
    but computes enough maxSpeed for the requested Poly5 segment
    and confirms arrival from /joint_states.
    """

    def __init__(
        self,
        node: RealFeedbackNode,
    ) -> None:
        self.node = node

        mapping_path = find_existing(
            REAL_MAPPING_CANDIDATES,
            "real_mapping.yaml",
        )

        print(
            f"[REAL MAP] {mapping_path}"
        )

        with mapping_path.open(
            "r",
            encoding="utf-8",
        ) as file:
            document = yaml.safe_load(
                file
            )

        params = document[
            "moveit_to_rmd_bridge"
        ][
            "ros__parameters"
        ]

        self.model_home = [
            float(v)
            for v in params[
                "model_home_rad"
            ]
        ]

        ifaces = [
            str(v)
            for v in params[
                "primary_motor_ifaces"
            ]
        ]

        ids = [
            int(v)
            for v in params[
                "primary_motor_ids"
            ]
        ]

        self.primary_raw_home = [
            float(v)
            for v in params[
                "primary_raw_home_deg"
            ]
        ]

        self.primary_sign = [
            float(v)
            for v in params[
                "raw_per_model_sign"
            ]
        ]

        gears = [
            float(v)
            for v in params[
                "primary_gear_ratio"
            ]
        ]

        if not (
            len(self.model_home)
            == len(ifaces)
            == len(ids)
            == len(self.primary_raw_home)
            == len(self.primary_sign)
            == len(gears)
            == 6
        ):
            raise ValueError(
                "arm mapping array length error"
            )

        self.primary_keys = list(
            zip(ifaces, ids)
        )

        self.gear_ratio = {
            key: gear
            for key, gear
            in zip(
                self.primary_keys,
                gears,
            )
        }

        self.mirror_key = (
            str(
                params[
                    "joint2_mirror_iface"
                ]
            ),
            int(
                params[
                    "joint2_mirror_id"
                ]
            ),
        )

        self.mirror_raw_home = float(
            params[
                "joint2_mirror_raw_home_deg"
            ]
        )

        self.mirror_sign = float(
            params[
                "joint2_mirror_sign"
            ]
        )

        self.gear_ratio[
            self.mirror_key
        ] = float(
            params[
                "joint2_mirror_gear_ratio"
            ]
        )

        self.sockets: Dict[
            str,
            socket.socket,
        ] = {}

        for iface in sorted(
            {
                key[0]
                for key
                in self.gear_ratio
            }
        ):
            sock = socket.socket(
                socket.AF_CAN,
                socket.SOCK_RAW,
                socket.CAN_RAW,
            )

            sock.bind(
                (
                    iface,
                )
            )

            self.sockets[
                iface
            ] = sock

    def close(self) -> None:
        for sock in (
            self.sockets.values()
        ):
            sock.close()

    def q_to_raw_deg(
        self,
        q: Sequence[float],
    ):
        raw = {}

        for index, joint in enumerate(
            JOINTS
        ):
            delta_deg = (
                q[index]
                - self.model_home[index]
            ) * RAD2DEG

            key = self.primary_keys[
                index
            ]

            raw[key] = (
                self.primary_raw_home[
                    index
                ]
                + self.primary_sign[
                    index
                ]
                * delta_deg
            )

        q2_delta_deg = (
            q[1]
            - self.model_home[1]
        ) * RAD2DEG

        raw[
            self.mirror_key
        ] = (
            self.mirror_raw_home
            + self.mirror_sign
            * q2_delta_deg
        )

        return raw

    def raw_to_protocol_deg(
        self,
        key,
        raw_deg: float,
    ) -> float:
        return (
            raw_deg
            * self.gear_ratio[
                key
            ]
        )

    def send_raw_position(
        self,
        key,
        raw_deg: float,
        speed_dps: int,
    ) -> None:
        iface, motor_id = key

        protocol_deg = (
            self.raw_to_protocol_deg(
                key,
                raw_deg,
            )
        )

        counts = int(
            round(
                protocol_deg
                / 0.01
            )
        )

        speed_dps = max(
            0,
            min(
                65535,
                int(speed_dps),
            ),
        )

        payload = struct.pack(
            "<BBHi",
            0xA4,
            0x00,
            speed_dps,
            counts,
        )

        frame = struct.pack(
            CAN_FRAME_FMT,
            motor_id,
            8,
            payload,
        )

        self.sockets[
            iface
        ].send(frame)

    def send_q(
        self,
        q: Sequence[float],
        speed_by_key,
    ) -> None:
        raw = self.q_to_raw_deg(
            q
        )

        for key, raw_deg in raw.items():
            self.send_raw_position(
                key,
                raw_deg,
                speed_by_key[
                    key
                ],
            )

    def calculate_segment_speeds(
        self,
        start_q: Sequence[float],
        target_q: Sequence[float],
        duration_sec: float,
    ):
        raw_start = self.q_to_raw_deg(
            start_q
        )

        raw_target = self.q_to_raw_deg(
            target_q
        )

        speed_by_key = {}

        for key in raw_start:
            start_protocol = (
                self.raw_to_protocol_deg(
                    key,
                    raw_start[key],
                )
            )

            target_protocol = (
                self.raw_to_protocol_deg(
                    key,
                    raw_target[key],
                )
            )

            delta_protocol = abs(
                target_protocol
                - start_protocol
            )

            theoretical_peak = (
                POLY5_MAX_DS
                * delta_protocol
                / max(
                    duration_sec,
                    1.0e-6,
                )
            )

            speed = int(
                math.ceil(
                    theoretical_peak
                    * ARM_SPEED_MARGIN
                )
            )

            if delta_protocol < 0.01:
                speed = (
                    ARM_MIN_PROTOCOL_SPEED_DPS
                )

            speed = max(
                ARM_MIN_PROTOCOL_SPEED_DPS,
                min(
                    ARM_MAX_PROTOCOL_SPEED_DPS,
                    speed,
                ),
            )

            speed_by_key[
                key
            ] = speed

        return speed_by_key

    def wait_until_reached(
        self,
        target_q: Sequence[float],
        speed_by_key,
        timeout_sec: float,
    ) -> bool:
        deadline = (
            time.monotonic()
            + timeout_sec
        )

        last_resend = 0.0

        while (
            rclpy.ok()
            and time.monotonic()
            < deadline
        ):
            rclpy.spin_once(
                self.node,
                timeout_sec=0.03,
            )

            if (
                self.node.current_q
                is None
            ):
                continue

            errors = [
                target
                - actual
                for target, actual
                in zip(
                    target_q,
                    self.node.current_q,
                )
            ]

            max_error = max(
                abs(value)
                for value in errors
            )

            print(
                "\r[ARRIVAL] error deg = "
                + "["
                + ", ".join(
                    f"{math.degrees(v):+6.2f}"
                    for v in errors
                )
                + "]"
                + f" | max="
                f"{math.degrees(max_error):.2f}°",
                end="",
                flush=True,
            )

            if (
                max_error
                <= ARM_ARRIVAL_TOL_RAD
            ):
                print()
                print(
                    "[ARRIVAL] physical target reached."
                )
                return True

            now = time.monotonic()

            if (
                now - last_resend
                >= 0.25
            ):
                self.send_q(
                    target_q,
                    speed_by_key,
                )

                last_resend = now

        print()
        print(
            "[ERROR] physical target arrival timeout."
        )

        return False

    def calculate_path_speeds(
        self,
        q_path: Sequence[Sequence[float]],
        duration_sec: float,
    ):
        if len(q_path) < 2:
            raise ValueError(
                "q_path must contain at least two points"
            )

        dt = (
            duration_sec
            / max(1, len(q_path) - 1)
        )

        raw_path = [
            self.q_to_raw_deg(q)
            for q in q_path
        ]

        speed_by_key = {}

        keys = list(
            raw_path[0].keys()
        )

        for key in keys:
            maximum_protocol_velocity = 0.0

            previous_protocol = (
                self.raw_to_protocol_deg(
                    key,
                    raw_path[0][key],
                )
            )

            for raw in raw_path[1:]:
                current_protocol = (
                    self.raw_to_protocol_deg(
                        key,
                        raw[key],
                    )
                )

                velocity = abs(
                    current_protocol
                    - previous_protocol
                ) / max(dt, 1.0e-6)

                maximum_protocol_velocity = max(
                    maximum_protocol_velocity,
                    velocity,
                )

                previous_protocol = (
                    current_protocol
                )

            speed = int(
                math.ceil(
                    maximum_protocol_velocity
                    * ARM_SPEED_MARGIN
                )
            )

            speed = max(
                ARM_MIN_PROTOCOL_SPEED_DPS,
                min(
                    ARM_MAX_PROTOCOL_SPEED_DPS,
                    speed,
                ),
            )

            speed_by_key[
                key
            ] = speed

        return speed_by_key

    def move_locked_path(
        self,
        q_path: Sequence[Sequence[float]],
        duration_sec: float,
        label: str,
    ) -> bool:
        """
        Stream the already solved FULL-pose Cartesian IK path.

        Unlike endpoint-only joint interpolation, every command point came
        from RPY=(0,0,0) IK, so wrist compensation is retained throughout
        the approach.
        """
        if len(q_path) < 2:
            print(
                "[ERROR] locked path has fewer than 2 points"
            )
            return False

        speed_by_key = (
            self.calculate_path_speeds(
                q_path,
                duration_sec,
            )
        )

        print()
        print("=" * 76)
        print(
            f"[REAL ROLL/YAW LOCK + AUTO-PITCH PATH] {label}"
        )
        print("=" * 76)

        print(
            f"points : {len(q_path)}"
        )

        print(
            f"time   : {duration_sec:.2f} s"
        )

        print(
            "각 command point는 사전 계산된 full-pose IK 결과입니다."
        )

        print(
            "v11 path policy: roll=0°, yaw=0°, pitch=auto"
        )

        print(
            "RMD maxSpeed:"
        )

        for key in sorted(
            speed_by_key
        ):
            print(
                f"  {key[0]} "
                f"0x{key[1]:X}: "
                f"{speed_by_key[key]} dps"
            )

        print("=" * 76)

        start_time = time.monotonic()
        point_count = len(q_path)

        # Skip the first point: the robot is already at/near it.
        for index in range(
            1,
            point_count,
        ):
            ratio = (
                index
                / (point_count - 1)
            )

            expected = (
                start_time
                + duration_sec
                * ratio
            )

            wait = (
                expected
                - time.monotonic()
            )

            if wait > 0.0:
                time.sleep(wait)

            self.send_q(
                q_path[index],
                speed_by_key,
            )

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

        target_q = list(
            q_path[-1]
        )

        self.send_q(
            target_q,
            speed_by_key,
        )

        return self.wait_until_reached(
            target_q,
            speed_by_key,
            ARM_ARRIVAL_TIMEOUT_SEC,
        )


    def move_poly5(
        self,
        target_q: Sequence[float],
        duration_sec: float,
        label: str,
    ) -> bool:
        self.node.refresh()

        if (
            self.node.current_q
            is None
        ):
            print(
                "[ERROR] current /joint_states unavailable."
            )
            return False

        start_q = (
            self.node.current_q.copy()
        )

        speed_by_key = (
            self.calculate_segment_speeds(
                start_q,
                target_q,
                duration_sec,
            )
        )

        print()
        print("=" * 76)
        print(
            f"[REAL ARM] {label}"
        )
        print("=" * 76)

        print(
            "start q:",
            format_q_deg(start_q),
        )

        print(
            "goal  q:",
            format_q_deg(target_q),
        )

        print(
            f"Poly5 command time: "
            f"{duration_sec:.2f} s"
        )

        print(
            "RMD maxSpeed:"
        )

        for key in sorted(
            speed_by_key
        ):
            print(
                f"  {key[0]} "
                f"0x{key[1]:X}: "
                f"{speed_by_key[key]} dps"
            )

        print("=" * 76)

        sample_count = max(
            2,
            int(
                round(
                    duration_sec
                    * REAL_COMMAND_RATE_HZ
                )
            ),
        )

        start_time = (
            time.monotonic()
        )

        for index in range(
            sample_count + 1
        ):
            ratio = (
                index
                / sample_count
            )

            blend = poly5(
                ratio
            )

            q_cmd = [
                q0
                + blend
                * (q1 - q0)
                for q0, q1
                in zip(
                    start_q,
                    target_q,
                )
            ]

            self.send_q(
                q_cmd,
                speed_by_key,
            )

            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )

            expected = (
                start_time
                + duration_sec
                * ratio
            )

            wait = (
                expected
                - time.monotonic()
            )

            if wait > 0.0:
                time.sleep(wait)

        # Exact final target.
        self.send_q(
            target_q,
            speed_by_key,
        )

        return self.wait_until_reached(
            target_q,
            speed_by_key,
            ARM_ARRIVAL_TIMEOUT_SEC,
        )


# ============================================================
# Gripper direct controller
# ============================================================

def send_gripper_protocol(
    node: RealFeedbackNode,
    target_protocol_deg: float,
) -> bool:
    counts = int(
        round(
            target_protocol_deg
            * 100.0
        )
    )

    payload = struct.pack(
        "<BBHi",
        0xA4,
        0x00,
        GRIPPER_SPEED_DPS,
        counts,
    )

    frame = struct.pack(
        CAN_FRAME_FMT,
        GRIPPER_CAN_ID,
        8,
        payload,
    )

    sock = socket.socket(
        socket.PF_CAN,
        socket.SOCK_RAW,
        socket.CAN_RAW,
    )

    try:
        sock.bind(
            (
                GRIPPER_CAN_IFACE,
            )
        )

        sock.send(frame)

    finally:
        sock.close()

    target_topic_deg = (
        target_protocol_deg
        / GRIPPER_TOPIC_DIVISOR
    )

    deadline = (
        time.monotonic()
        + GRIPPER_WAIT_TIMEOUT_SEC
    )

    while (
        rclpy.ok()
        and time.monotonic()
        < deadline
    ):
        rclpy.spin_once(
            node,
            timeout_sec=0.05,
        )

        if (
            node.gripper_topic_deg
            is None
        ):
            continue

        error = (
            target_topic_deg
            - node.gripper_topic_deg
        )

        print(
            "\r[GRIPPER] feedback="
            f"{node.gripper_topic_deg:+9.3f} "
            f"| target="
            f"{target_topic_deg:+9.3f} "
            f"| err={error:+7.3f}",
            end="",
            flush=True,
        )

        if (
            abs(error)
            <=
            GRIPPER_TOPIC_TOLERANCE_DEG
        ):
            print()
            return True

    print()
    print(
        "[ERROR] gripper arrival timeout."
    )

    return False


# ============================================================
# MuJoCo 3x preview
# ============================================================

LEGACY_STATIC_OBJECT_NAMES = {
    "pickup_cube",
}

LEGACY_STATIC_GEOM_NAMES = {
    "pickup_cube_geom",
}


def remove_legacy_static_objects(
    root: ET.Element,
) -> int:
    """
    Remove only the old fixed pickup object from the preview copy.
    Pedestals/floor/robot geometry are untouched.

    The source MJCF is not modified here.
    """
    removed = 0

    for parent in root.iter():
        for child in list(parent):
            name = child.attrib.get("name", "")

            remove = False

            if (
                child.tag == "body"
                and name in LEGACY_STATIC_OBJECT_NAMES
            ):
                remove = True

            if (
                child.tag == "geom"
                and name in LEGACY_STATIC_GEOM_NAMES
            ):
                remove = True

            if remove:
                parent.remove(child)
                removed += 1

    return removed


def build_debug_preview_xml(
    source_mjcf: Path,
    plan,
) -> Path:
    tree = ET.parse(
        source_mjcf
    )

    root = tree.getroot()

    # Every IK preview creates its own object at the NEW grasp position.
    # Therefore the old fixed pickup_cube must never remain in the preview.
    removed_legacy = remove_legacy_static_objects(
        root
    )

    base = root.find(
        ".//body[@name='ARM_BASE_LINK']"
    )

    if base is None:
        raise RuntimeError(
            "MuJoCo XML에서 ARM_BASE_LINK를 찾지 못했습니다."
        )

    # Remove old debug items if a previous generated file was reused.
    for child in list(base):
        name = child.attrib.get(
            "name",
            "",
        )

        if name.startswith(
            "IK_DEBUG_"
        ):
            base.remove(child)

    if removed_legacy > 0:
        print(
            f"[PREVIEW] 기존 고정 pickup_cube "
            f"{removed_legacy}개 제거"
        )

    grasp = plan[
        "grasp_xyz"
    ]

    near = plan[
        "near_xyz"
    ]

    lift = plan[
        "lift_xyz"
    ]

    # Visualization-only box:
    # 8 cm (X) x 9 cm (Y grasp width) x 8 cm (Z)
    box_body = ET.SubElement(
        base,
        "body",
        {
            "name": "IK_DEBUG_BOX",
            "pos": (
                f"{grasp[0]:.9f} "
                f"{grasp[1]:.9f} "
                f"{grasp[2]:.9f}"
            ),
        },
    )

    ET.SubElement(
        box_body,
        "geom",
        {
            "name": "IK_DEBUG_BOX_GEOM",
            "type": "box",
            "size": "0.04 0.045 0.04",
            "rgba": "0.9 0.25 0.15 0.45",
            "contype": "0",
            "conaffinity": "0",
        },
    )

    for name, xyz, rgba in [
        (
            "IK_DEBUG_NEAR_ALIGN",
            near,
            "1 0.7 0 0.9",
        ),
        (
            "IK_DEBUG_GRASP",
            grasp,
            "1 0 0 0.9",
        ),
        (
            "IK_DEBUG_LIFT",
            lift,
            "0 1 0 0.9",
        ),
    ]:
        ET.SubElement(
            base,
            "site",
            {
                "name": name,
                "type": "sphere",
                "size": "0.012",
                "pos": (
                    f"{xyz[0]:.9f} "
                    f"{xyz[1]:.9f} "
                    f"{xyz[2]:.9f}"
                ),
                "rgba": rgba,
            },
        )

    output = (
        source_mjcf.parent
        / "drok_arm_ik_debug_preview.xml"
    )

    try:
        ET.indent(
            tree,
            space="  ",
        )
    except AttributeError:
        pass

    tree.write(
        output,
        encoding="utf-8",
        xml_declaration=True,
    )

    return output


def mj_joint_qpos_address(
    model: mujoco.MjModel,
    name: str,
) -> int:
    joint_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_JOINT,
        name,
    )

    if joint_id < 0:
        raise RuntimeError(
            f"MuJoCo joint 없음: {name}"
        )

    return int(
        model.jnt_qposadr[
            joint_id
        ]
    )


def preview_plan_mujoco(
    plan,
) -> None:
    if mujoco is None:
        raise RuntimeError(
            "MuJoCo preview is optional and is not installed. "
            "Current automatic/manual real grasp does not need it."
        )
    source_mjcf = find_existing(
        MJCF_CANDIDATES,
        "MuJoCo MJCF",
    )

    preview_xml = (
        build_debug_preview_xml(
            source_mjcf,
            plan,
        )
    )

    print()
    print("=" * 76)
    print(
        " MUJOCO DEBUG PREVIEW — 3x SPEED / REPEAT"
    )
    print("=" * 76)

    print(
        f"MJCF: {preview_xml}"
    )

    print(
        "Orange sphere = NEAR (box 위 9 cm)"
    )

    print(
        "Red sphere    = GRASP / box center"
    )

    print(
        "Green sphere  = LIFT"
    )

    print()
    print(
        "APPROACH orientation policy:"
    )

    print(
        "  roll  = 0°"
    )

    print(
        "  yaw   = 0°"
    )

    print(
        f"  pitch = AUTO "
        f"({math.degrees(plan['home_pitch_rad']):+.1f}° "
        f"-> {math.degrees(plan['approach_pitch_rad']):+.1f}°)"
    )

    print(
        f"  geometric request = "
        f"{math.degrees(plan['requested_approach_pitch_rad']):+.1f}°"
    )

    print(
        f"  joint-safe result = "
        f"{math.degrees(plan['approach_pitch_rad']):+.1f}°"
    )

    print(
        "Roll/Yaw 비틀림은 막고, Pitch는 박스를 향하도록 허용합니다."
    )

    if abs(
        math.degrees(
            plan["approach_pitch_rad"]
        ) - 90.0
    ) < 1.0:
        print(
            "현재 기본 경로에서는 +90° = top-down 접근입니다."
        )

    print(
        "Preview는 Enter 또는 viewer 종료 전까지 계속 반복합니다."
    )

    print(
        "REAL MOTOR COMMAND = NONE"
    )

    print("=" * 76)

    model = (
        mujoco.MjModel.from_xml_path(
            str(
                preview_xml
            )
        )
    )

    data = mujoco.MjData(
        model
    )

    addresses = {
        name: (
            mj_joint_qpos_address(
                model,
                name,
            )
        )
        for name in (
            JOINTS
            + [
                "JOINT7",
                "GRIPPER_RIGHT_JOINT",
            ]
        )
    }

    def stop_requested() -> bool:
        try:
            readable, _, _ = select.select(
                [sys.stdin],
                [],
                [],
                0.0,
            )
        except (ValueError, OSError):
            return False

        if not readable:
            return False

        sys.stdin.readline()

        print()
        print(
            "[PREVIEW] Enter -> 반복재생 종료"
        )

        return True

    def set_state(
        arm_q: Sequence[float],
        finger_travel_m: float,
    ) -> None:
        for name, value in zip(
            JOINTS,
            arm_q,
        ):
            data.qpos[
                addresses[name]
            ] = value

        data.qpos[
            addresses["JOINT7"]
        ] = finger_travel_m

        data.qpos[
            addresses[
                "GRIPPER_RIGHT_JOINT"
            ]
        ] = -finger_travel_m

        data.qvel[:] = 0.0

        mujoco.mj_forward(
            model,
            data,
        )

    def hold(
        viewer,
        arm_q: Sequence[float],
        finger_m: float,
        seconds: float,
    ) -> bool:
        deadline = (
            time.monotonic()
            + seconds
        )

        while (
            viewer.is_running()
            and time.monotonic()
            < deadline
        ):
            if stop_requested():
                return False

            set_state(
                arm_q,
                finger_m,
            )

            viewer.sync()
            time.sleep(0.02)

        return viewer.is_running()

    def animate_joint_segment(
        viewer,
        q0: Sequence[float],
        q1: Sequence[float],
        real_duration: float,
        label: str,
        finger_m: float,
    ) -> bool:
        preview_duration = (
            real_duration
            / PREVIEW_SPEEDUP
        )

        count = max(
            2,
            int(
                round(
                    preview_duration
                    * PREVIEW_RATE_HZ
                )
            ),
        )

        print(
            f"[PREVIEW] {label}: "
            f"real {real_duration:.2f}s "
            f"-> preview {preview_duration:.2f}s"
        )

        for index in range(
            count + 1
        ):
            if (
                not viewer.is_running()
                or stop_requested()
            ):
                return False

            ratio = (
                index / count
            )

            blend = poly5(
                ratio
            )

            q = [
                a
                + blend
                * (b - a)
                for a, b
                in zip(q0, q1)
            ]

            set_state(
                q,
                finger_m,
            )

            viewer.sync()

            time.sleep(
                preview_duration
                / count
            )

        return True

    def animate_locked_path(
        viewer,
        q_path: Sequence[Sequence[float]],
        real_duration: float,
        label: str,
        finger_m: float,
    ) -> bool:
        preview_duration = (
            real_duration
            / PREVIEW_SPEEDUP
        )

        count = len(
            q_path
        )

        if count < 2:
            return True

        dt = (
            preview_duration
            / (count - 1)
        )

        print(
            f"[PREVIEW LOCK] {label}: "
            f"{count} full-pose IK points | "
            f"real {real_duration:.2f}s "
            f"-> preview {preview_duration:.2f}s"
        )

        for index, q in enumerate(
            q_path
        ):
            if (
                not viewer.is_running()
                or stop_requested()
            ):
                return False

            set_state(
                q,
                finger_m,
            )

            viewer.sync()

            if index < count - 1:
                time.sleep(dt)

        return True

    def animate_gripper(
        viewer,
        arm_q: Sequence[float],
    ) -> bool:
        preview_duration = (
            REAL_GRIPPER_CLOSE_SEC
            / PREVIEW_SPEEDUP
        )

        count = max(
            2,
            int(
                round(
                    preview_duration
                    * PREVIEW_RATE_HZ
                )
            ),
        )

        print(
            f"[PREVIEW] GRIPPER 14.5 -> 9.0 cm "
            f"({preview_duration:.2f}s)"
        )

        for index in range(
            count + 1
        ):
            if (
                not viewer.is_running()
                or stop_requested()
            ):
                return False

            ratio = (
                index / count
            )

            finger = (
                poly5(ratio)
                * MJ_GRIPPER_EACH_TRAVEL_M
            )

            set_state(
                arm_q,
                finger,
            )

            viewer.sync()

            time.sleep(
                preview_duration
                / count
            )

        return True

    set_state(
        plan["start_q"],
        0.0,
    )

    with mujoco.viewer.launch_passive(
        model,
        data,
    ) as viewer:
        viewer.cam.distance = 1.6
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [
            0.35,
            0.0,
            0.20,
        ]

        loop_index = 0
        stop = False

        while (
            viewer.is_running()
            and not stop
        ):
            loop_index += 1

            print()
            print(
                f"[PREVIEW LOOP {loop_index}]"
            )

            set_state(
                plan["start_q"],
                0.0,
            )

            viewer.sync()

            if not hold(
                viewer,
                plan["start_q"],
                0.0,
                0.25,
            ):
                break

            # First correct the existing ~-24 deg wrist mounting tilt
            # without treating that tilt as the desired orientation.
            if not animate_joint_segment(
                viewer,
                plan["start_q"],
                plan["q_prealign"],
                REAL_CURRENT_TO_PREALIGN_SEC,
                "CURRENT -> PREALIGN (roll/yaw=0, HOME pitch 유지)",
                0.0,
            ):
                break

            if not animate_locked_path(
                viewer,
                plan["approach1_path"],
                REAL_APPROACH1_SEC,
                "APPROACH1: PREALIGN -> NEAR / pitch auto",
                0.0,
            ):
                break

            if not animate_locked_path(
                viewer,
                plan["approach2_path"],
                REAL_APPROACH2_SEC,
                "APPROACH2: NEAR -> GRASP",
                0.0,
            ):
                break

            if not animate_gripper(
                viewer,
                plan["q_grasp"],
            ):
                break

            if not animate_locked_path(
                viewer,
                plan["lift_path"],
                REAL_GRASP_TO_LIFT_SEC,
                "LIFT",
                MJ_GRIPPER_EACH_TRAVEL_M,
            ):
                break

            if not hold(
                viewer,
                plan["q_lift"],
                MJ_GRIPPER_EACH_TRAVEL_M,
                0.5,
            ):
                break

        # Viewer close or Enter both mean preview finished.
        stop = True

    print()
    print(
        "[PREVIEW] 반복 preview 종료 -> 실제 실행 확인 단계"
    )


# ============================================================
# Plan
# ============================================================

def calculate_plan(
    node: RealFeedbackNode,
):
    baseline = (
        load_baseline_ik_module()
    )

    limits = (
        baseline.load_joint_limits(
            URDF_PATH
        )
    )

    node.refresh()

    if node.current_q is None:
        raise RuntimeError(
            "현재 /joint_states 없음"
        )

    start_q = (
        node.current_q.copy()
    )

    # Fixed HOME reference for object coordinates.
    home_tcp, home_rpy_raw = (
        run_project_fk_pose(
            HOME_Q
        )
    )

    home_rpy = (
        canonicalize_zyx_rpy(
            home_rpy_raw
        )
    )

    home_pitch_rad = float(
        home_rpy[1]
    )

    print()
    print("=" * 76)
    print(
        " ROLL/YAW LOCK + AUTO PITCH REFERENCE"
    )
    print("=" * 76)

    print(
        "CURRENT q [deg]:",
        format_q_deg(start_q),
    )

    print(
        "HOME TCP [m]   :",
        format_xyz(home_tcp),
    )

    print(
        "HOME TCP raw RPY [deg]:",
        "["
        + ", ".join(
            f"{math.degrees(value):+7.2f}"
            for value in home_rpy_raw
        )
        + "]",
    )

    print(
        "HOME TCP canonical RPY [deg]:",
        "["
        + ", ".join(
            f"{math.degrees(value):+7.2f}"
            for value in home_rpy
        )
        + "]",
    )

    print(
        "  위 두 RPY는 같은 회전행렬입니다."
    )

    print()
    print(
        "v11:"
    )

    print(
        "  Roll=0°, Yaw=0° 유지"
    )

    print(
        "  Pitch는 접근 방향을 기준으로 계산"
    )

    print(
        "  단, +90°가 wrist joint limit margin을 침범하면"
    )

    print(
        "  90 -> 85 -> 80 -> ... -> 60° 순서로 자동 후퇴"
    )

    print(
        f"  기존 software joint-limit margin "
        f"{math.degrees(MIN_JOINT_LIMIT_MARGIN_RAD):.2f}°는 그대로 유지"
    )

    print("=" * 76)
    print()

    forward_cm = read_float(
        "HOME TCP에서 전방 오프셋 [cm]",
        DEFAULT_FORWARD_CM,
    )

    lateral_cm = read_float(
        "좌우 오프셋 [cm] "
        "(왼쪽 + / 오른쪽 -)",
        DEFAULT_LATERAL_CM,
    )

    down_cm = read_float(
        "아래 오프셋 [cm] "
        "(아래쪽을 +값으로 입력)",
        DEFAULT_DOWN_CM,
    )

    grasp = [
        home_tcp[0]
        + forward_cm / 100.0,
        home_tcp[1]
        + lateral_cm / 100.0,
        home_tcp[2]
        - down_cm / 100.0,
    ]

    near = [
        grasp[0],
        grasp[1],
        grasp[2]
        + NEAR_STANDOFF_M,
    ]

    lift = [
        grasp[0],
        grasp[1],
        grasp[2]
        + LIFT_HEIGHT_M,
    ]

    requested_approach_pitch_rad = (
        approach_pitch_from_vector(
            near,
            grasp,
        )
    )

    prealign_rpy = (
        make_roll_yaw_locked_rpy(
            home_pitch_rad
        )
    )

    print()
    print("=" * 76)
    print(
        " TWO-STAGE AUTO-PITCH TARGET"
    )
    print("=" * 76)

    print(
        "PREALIGN @ HOME:",
        format_xyz(home_tcp),
    )

    print(
        "  target RPY [deg] = "
        f"[0.00, {math.degrees(home_pitch_rad):+.2f}, 0.00]"
    )

    print(
        "APPROACH1 end / NEAR:",
        format_xyz(near),
        f"  (box 위 {NEAR_STANDOFF_M*100:.0f} cm)",
    )

    print(
        "APPROACH2 end / GRASP:",
        format_xyz(grasp),
    )

    print(
        "LIFT:",
        format_xyz(lift),
    )

    print()
    print(
        "Geometric approach pitch:"
    )

    print(
        f"  NEAR -> GRASP vector = "
        f"{math.degrees(requested_approach_pitch_rad):+.2f}°"
    )

    if abs(
        abs(
            math.degrees(
                requested_approach_pitch_rad
            )
        ) - 90.0
    ) < 1.0:
        print(
            "  현재 TCP +X=approach 정의에서는 "
            "|Pitch|=90°가 top-down 접근입니다."
        )

    print()
    print(
        "Pitch feasibility search:"
    )

    print(
        "  "
        + " -> ".join(
            f"{math.degrees(value):+.0f}°"
            for value in make_pitch_search_candidates(
                requested_approach_pitch_rad
            )
        )
    )

    print("=" * 76)

    choice = input(
        "FULL-POSE IK 계산하려면 [Enter], 취소는 q: "
    ).strip().lower()

    if choice == "q":
        return None

    # --------------------------------------------------------
    # 0) PREALIGN at HOME.
    # --------------------------------------------------------
    q_prealign = solve_waypoint(
        baseline,
        "PREALIGN @ HOME TCP",
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

    # --------------------------------------------------------
    # Search the steepest feasible approach pitch.
    # Do NOT relax the joint-limit margin.
    # --------------------------------------------------------
    selected_pitch_rad = None
    selected_approach1_path = None
    selected_approach2_path = None
    selected_lift_path = None

    for attempt_index, candidate_pitch_rad in enumerate(
        make_pitch_search_candidates(
            requested_approach_pitch_rad
        ),
        start=1,
    ):
        print()
        print("#" * 76)

        print(
            f" PITCH SEARCH {attempt_index}: "
            f"{math.degrees(candidate_pitch_rad):+.2f}°"
        )

        print("#" * 76)

        # APPROACH1
        approach1_path = (
            solve_pitch_cartesian_path(
                baseline,
                "APPROACH1",
                home_tcp,
                near,
                q_prealign,
                limits,
                APPROACH1_IK_STEP_M,
                home_pitch_rad,
                candidate_pitch_rad,
            )
        )

        if approach1_path is None:
            print(
                f"[PITCH REJECT] "
                f"{math.degrees(candidate_pitch_rad):+.2f}° "
                f"-> APPROACH1 실패"
            )
            continue

        q_near_candidate = list(
            approach1_path[-1]
        )

        # APPROACH2
        approach2_path = (
            solve_pitch_cartesian_path(
                baseline,
                "APPROACH2",
                near,
                grasp,
                q_near_candidate,
                limits,
                APPROACH2_IK_STEP_M,
                candidate_pitch_rad,
                candidate_pitch_rad,
            )
        )

        if approach2_path is None:
            print(
                f"[PITCH REJECT] "
                f"{math.degrees(candidate_pitch_rad):+.2f}° "
                f"-> APPROACH2 실패"
            )
            continue

        q_grasp_candidate = list(
            approach2_path[-1]
        )

        # LIFT
        lift_path = (
            solve_pitch_cartesian_path(
                baseline,
                "LIFT",
                grasp,
                lift,
                q_grasp_candidate,
                limits,
                LIFT_IK_STEP_M,
                candidate_pitch_rad,
                candidate_pitch_rad,
            )
        )

        if lift_path is None:
            print(
                f"[PITCH REJECT] "
                f"{math.degrees(candidate_pitch_rad):+.2f}° "
                f"-> LIFT 실패"
            )
            continue

        selected_pitch_rad = float(
            candidate_pitch_rad
        )

        selected_approach1_path = (
            approach1_path
        )

        selected_approach2_path = (
            approach2_path
        )

        selected_lift_path = (
            lift_path
        )

        print()
        print("=" * 76)

        print(
            f"[PITCH SELECTED] "
            f"{math.degrees(selected_pitch_rad):+.2f}°"
        )

        if (
            abs(
                selected_pitch_rad
                - requested_approach_pitch_rad
            ) > math.radians(0.1)
        ):
            print(
                f"  geometric request "
                f"{math.degrees(requested_approach_pitch_rad):+.2f}°에서"
            )

            print(
                f"  wrist joint-limit margin을 지키기 위해 "
                f"{math.degrees(selected_pitch_rad):+.2f}°로 완화했습니다."
            )

        print("=" * 76)

        break

    if (
        selected_pitch_rad is None
        or selected_approach1_path is None
        or selected_approach2_path is None
        or selected_lift_path is None
    ):
        return {
            "ok": False,
            "failed": "PITCH_FEASIBILITY_SEARCH",
        }

    approach_pitch_rad = (
        selected_pitch_rad
    )

    approach1_path = (
        selected_approach1_path
    )

    approach2_path = (
        selected_approach2_path
    )

    lift_path = (
        selected_lift_path
    )

    q_near = list(
        approach1_path[-1]
    )

    q_grasp = list(
        approach2_path[-1]
    )

    q_lift = list(
        lift_path[-1]
    )

    final_rpy = (
        make_roll_yaw_locked_rpy(
            approach_pitch_rad
        )
    )

    return {
        "ok": True,
        "start_q": start_q,
        "home_tcp": home_tcp,
        "home_rpy_raw": list(home_rpy_raw),
        "home_rpy": list(home_rpy),
        "home_pitch_rad": home_pitch_rad,
        "requested_approach_pitch_rad": (
            requested_approach_pitch_rad
        ),
        "approach_pitch_rad": approach_pitch_rad,
        "prealign_rpy": prealign_rpy,
        "final_rpy": final_rpy,
        "near_xyz": near,
        "grasp_xyz": grasp,
        "lift_xyz": lift,
        "q_prealign": q_prealign,
        "q_near": q_near,
        "q_grasp": q_grasp,
        "q_lift": q_lift,
        "approach1_path": approach1_path,
        "approach2_path": approach2_path,
        "lift_path": lift_path,
    }


# ============================================================
# Real execution
# ============================================================

def execute_real(
    node: RealFeedbackNode,
    plan,
) -> bool:
    print()
    print("=" * 76)
    print(
        " MUJOCO PREVIEW PASSED — REAL EXECUTION READY"
    )
    print("=" * 76)

    print(
        "PREALIGN:",
        format_q_deg(
            plan["q_prealign"]
        ),
    )

    print(
        "NEAR    :",
        format_q_deg(
            plan["q_near"]
        ),
    )

    print(
        "GRASP   :",
        format_q_deg(
            plan["q_grasp"]
        ),
    )

    print(
        "LIFT    :",
        format_q_deg(
            plan["q_lift"]
        ),
    )

    print()
    print(
        "Orientation policy:"
    )

    print(
        "  Roll = 0°"
    )

    print(
        "  Yaw  = 0°"
    )

    print(
        f"  Pitch APPROACH1: "
        f"{math.degrees(plan['home_pitch_rad']):+.2f}° "
        f"-> {math.degrees(plan['approach_pitch_rad']):+.2f}°"
    )

    print(
        f"  Pitch APPROACH2/LIFT: "
        f"{math.degrees(plan['approach_pitch_rad']):+.2f}°"
    )

    print()
    print(
        "REAL sequence:"
    )

    print(
        "  1. gripper OPEN 14.5 cm"
    )

    print(
        "  2. HOME TCP PREALIGN: Roll/Yaw=0, HOME Pitch 유지"
    )

    print(
        "  3. APPROACH1: Pitch를 자연스럽게 증가시키며 NEAR까지 이동"
    )

    print(
        "  4. 실제 NEAR 도착 후 APPROACH2 IK 재계산"
    )

    print(
        "  5. APPROACH2: 최종 Pitch 유지하며 박스로 접근"
    )

    print(
        "  6. gripper CLOSE 9.0 cm"
    )

    print(
        "  7. LIFT: 동일 Pitch 유지"
    )

    print(
        "  8. HOLD"
    )

    print("=" * 76)

    answer = input(
        "실제 실행하려면 정확히 GRASP 입력: "
    ).strip()

    if answer != "GRASP":
        print(
            "실행 취소. 실제 모터 명령 없음."
        )
        return False

    executor = (
        DirectArmRmdExecutor(
            node
        )
    )

    try:
        print()
        print(
            f"[GRIPPER] OPEN "
            f"{GRIPPER_OPEN_GAP_CM:.2f} cm"
        )

        if not send_gripper_protocol(
            node,
            GRIPPER_OPEN_PROTOCOL_DEG,
        ):
            return False

        if not executor.move_poly5(
            plan["q_prealign"],
            REAL_CURRENT_TO_PREALIGN_SEC,
            "CURRENT -> PREALIGN",
        ):
            print(
                "[BLOCK] PREALIGN 도착 실패."
            )
            return False

        if not executor.move_locked_path(
            plan["approach1_path"],
            REAL_APPROACH1_SEC,
            "APPROACH1: PREALIGN -> NEAR / AUTO PITCH",
        ):
            print(
                "[BLOCK] NEAR 도착 실패."
            )
            return False

        # ----------------------------------------------------
        # REAL SECOND IK at measured NEAR.
        # ----------------------------------------------------
        node.refresh()

        if node.current_q is None:
            print(
                "[BLOCK] 실제 NEAR q를 읽지 못했습니다."
            )
            return False

        actual_near_q = (
            node.current_q.copy()
        )

        print()
        print("=" * 76)
        print(
            " REAL SECOND IK @ NEAR"
        )
        print("=" * 76)

        print(
            "actual q:",
            format_q_deg(
                actual_near_q
            ),
        )

        print(
            "APPROACH2 target:"
        )

        print(
            f"  RPY [deg] = "
            f"[0.00, "
            f"{math.degrees(plan['approach_pitch_rad']):+.2f}, "
            f"0.00]"
        )

        baseline = (
            load_baseline_ik_module()
        )

        limits = (
            baseline.load_joint_limits(
                URDF_PATH
            )
        )

        real_approach2_path = (
            solve_pitch_cartesian_path(
                baseline,
                "REAL APPROACH2",
                plan["near_xyz"],
                plan["grasp_xyz"],
                actual_near_q,
                limits,
                APPROACH2_IK_STEP_M,
                plan["approach_pitch_rad"],
                plan["approach_pitch_rad"],
            )
        )

        if real_approach2_path is None:
            print(
                "[BLOCK] 실제 NEAR에서 APPROACH2 IK 실패. "
                "박스로 내려가지 않습니다."
            )
            return False

        real_q_grasp = list(
            real_approach2_path[-1]
        )

        real_lift_path = (
            solve_pitch_cartesian_path(
                baseline,
                "REAL LIFT",
                plan["grasp_xyz"],
                plan["lift_xyz"],
                real_q_grasp,
                limits,
                LIFT_IK_STEP_M,
                plan["approach_pitch_rad"],
                plan["approach_pitch_rad"],
            )
        )

        if real_lift_path is None:
            print(
                "[BLOCK] 실제 LIFT IK 실패."
            )
            return False

        if not executor.move_locked_path(
            real_approach2_path,
            REAL_APPROACH2_SEC,
            "APPROACH2: NEAR -> GRASP / PITCH HOLD",
        ):
            print(
                "[BLOCK] GRASP 실제 도착 실패. "
                "그리퍼를 닫지 않습니다."
            )
            return False

        print()
        print(
            f"[GRIPPER] CLOSE "
            f"{GRIPPER_CLOSE_GAP_CM:.2f} cm"
        )

        if not send_gripper_protocol(
            node,
            GRIPPER_CLOSE_PROTOCOL_DEG,
        ):
            return False

        if not executor.move_locked_path(
            real_lift_path,
            REAL_GRASP_TO_LIFT_SEC,
            "LIFT: GRASP -> HOLD / PITCH HOLD",
        ):
            print(
                "[BLOCK] LIFT 실제 도착 실패."
            )
            return False

        print()
        print("=" * 76)
        print(
            " REAL GRASP COMPLETE — HOLD"
        )
        print("=" * 76)

        print(
            "Roll/Yaw는 0° 근처를 유지하고 "
            "Pitch는 접근 방향에 맞춰 사용했습니다."
        )

        print(
            "그리퍼는 9.0 cm 상태로 유지합니다."
        )

        print("=" * 76)

        return True

    finally:
        executor.close()


# ============================================================
# Main
# ============================================================


def main() -> int:
    print("=" * 76)
    print(
        " DROK INTERACTIVE BOX IK GRASP v11"
    )
    print("=" * 76)

    print(
        "v11 orientation policy:"
    )

    print(
        "  - HOME FK Euler를 principal ZYX branch로 canonicalize"
    )

    print(
        "    예: [155.99,179.96,180]° -> [-24.01,0.04,0]°"
    )

    print(
        "  - Roll = 0° 유지"
    )

    print(
        "  - Yaw  = 0° 유지"
    )

    print(
        "  - Pitch는 잠그지 않음"
    )

    print(
        "  - 최종 Pitch는 NEAR -> GRASP 접근 벡터로 자동 계산"
    )

    print(
        "  - 요청 Pitch가 wrist joint margin을 침범하면 "
        "5°씩 자동 backoff"
    )

    print(
        f"  - 최소 탐색 Pitch magnitude = "
        f"{PITCH_MIN_ABS_DEG:.0f}°"
    )

    print(
        "  - APPROACH1에서 HOME Pitch -> 최종 Pitch로 부드럽게 변화"
    )

    print(
        "  - APPROACH2/LIFT에서는 최종 Pitch 유지"
    )

    print(
        "  - 위치 허용오차 1 cm / 자세 허용오차 10°"
    )

    print(
        "  - 실제 NEAR 도착 후 APPROACH2 IK 재계산"
    )

    print(
        "  - MuJoCo 3배속 preview 반복"
    )

    print()
    print(
        "CAN interface / bitrate 변경: 없음"
    )

    print(
        "Motor ROM / limit 변경: 없음"
    )

    print("=" * 76)

    required = [
        GEOMETRY_PATH,
        URDF_PATH,
        BASELINE_IK_PATH,
        IK_SOLVER_EXE,
        FK_EXE,
    ]

    missing = [
        path
        for path in required
        if not path.is_file()
    ]

    if missing:
        print(
            "[ERROR] 필수 기존 파일 없음:"
        )

        for path in missing:
            print(
                " ",
                path,
            )

        return 1

    rclpy.init()

    node = RealFeedbackNode()

    try:
        if not node.wait_for_feedback():
            print(
                "[ERROR] /joint_states JOINT1~6 피드백 없음"
            )
            return 2

        try:
            plan = calculate_plan(
                node
            )
        except Exception as exc:
            print(
                "[ERROR]",
                exc,
            )
            return 3

        if plan is None:
            print(
                "취소되었습니다."
            )
            return 0

        if not plan["ok"]:
            print()
            print("=" * 76)
            print(
                " IK FAILED — REAL MOTION BLOCKED"
            )
            print(
                f"failed stage: "
                f"{plan['failed']}"
            )
            print(
                "실제 모터 명령 없음."
            )
            print("=" * 76)
            return 4

        print()
        print("=" * 76)
        print(
            " FULL-POSE ROLL/YAW LOCK + AUTO-PITCH IK SUCCESS"
        )
        print("=" * 76)

        print(
            "PREALIGN:",
            format_q_deg(
                plan["q_prealign"]
            ),
        )

        print(
            "NEAR    :",
            format_q_deg(
                plan["q_near"]
            ),
        )

        print(
            "GRASP   :",
            format_q_deg(
                plan["q_grasp"]
            ),
        )

        print(
            "LIFT    :",
            format_q_deg(
                plan["q_lift"]
            ),
        )

        print()
        print(
            f"Pitch profile: "
            f"{math.degrees(plan['home_pitch_rad']):+.2f}° "
            f"-> "
            f"{math.degrees(plan['approach_pitch_rad']):+.2f}°"
        )

        print(
            f"Geometric requested pitch: "
            f"{math.degrees(plan['requested_approach_pitch_rad']):+.2f}°"
        )

        print(
            f"Joint-safe selected pitch : "
            f"{math.degrees(plan['approach_pitch_rad']):+.2f}°"
        )

        print(
            "Roll/Yaw target: 0° / 0°"
        )

        print()
        print(
            f"APPROACH1 points: "
            f"{len(plan['approach1_path'])}"
        )

        print(
            f"APPROACH2 points: "
            f"{len(plan['approach2_path'])}"
        )

        print(
            f"LIFT points     : "
            f"{len(plan['lift_path'])}"
        )

        print("=" * 76)

        input(
            "실제 로봇은 아직 안 움직입니다. "
            "MuJoCo 반복 preview 시작 [Enter]: "
        )

        try:
            preview_plan_mujoco(
                plan
            )
        except Exception as exc:
            print()
            print(
                "[ERROR] MuJoCo preview 실패:"
            )
            print(
                exc
            )
            print(
                "Preview 실패 상태에서는 실제 실행을 막습니다."
            )
            return 5

        execute_real(
            node,
            plan,
        )

        return 0

    except KeyboardInterrupt:
        print()
        print(
            "Interrupted."
        )
        return 130

    finally:
        # Avoid the v8 Ctrl-C double-shutdown exception.
        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    sys.exit(main())
