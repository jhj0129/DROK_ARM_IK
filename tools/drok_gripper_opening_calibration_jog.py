#!/usr/bin/env python3
"""
DROK gripper calibration jog v4
===============================

Portable + safe-save version.

Key points
----------
- Finds interactive_box_ik_grasp_v11.py without hardcoding the username.
- Optional overrides:
    --v11 /absolute/path/to/interactive_box_ik_grasp_v11.py
    DROK_V11_PATH=/absolute/path/to/interactive_box_ik_grasp_v11.py
    DROK_GRASP_WS=/absolute/path/to/DROK_grasp_ws
- +1 / -1 are RELATIVE to the CURRENT feedback, so they still work if the
  motor's multi-turn feedback is on a different numerical branch.
- save open / save grasp use AST line locations instead of a greedy regex.
  This prevents two Python assignments from being concatenated.
- The patched Python text is compiled before an atomic replace.
- A timestamped backup is created before every successful save.
- No CAN interface state/bitrate changes.
- No motor ROM/limit writes.
"""

from __future__ import annotations

import argparse
import ast
import importlib.util
import math
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy


V11_FILENAME = "interactive_box_ik_grasp_v11.py"

# Calibration jog speed only.
TUNE_GRIPPER_SPEED_DPS = 120

# One command cannot jump farther than this in topic-angle degrees.
# +1/-1 are therefore always allowed; accidental huge "set" jumps are blocked.
MAX_SINGLE_JOG_TOPIC_DEG = 5.0

# If a save differs hugely from the currently saved calibration, require
# an explicit SAVE confirmation. This is expected when the motor's multi-turn
# angle has moved to another numerical branch.
LARGE_SAVE_DELTA_TOPIC_DEG = 30.0


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--v11",
        type=Path,
        default=None,
        help="Explicit path to interactive_box_ik_grasp_v11.py",
    )
    return parser.parse_args()


def _candidate_v11_paths(cli_path: Optional[Path]):
    candidates = []

    if cli_path is not None:
        candidates.append(cli_path)

    env_v11 = os.environ.get("DROK_V11_PATH")
    if env_v11:
        candidates.append(Path(env_v11))

    # Best portable case: this calibration script is installed next to v11.
    script_dir = Path(__file__).resolve().parent
    candidates.append(script_dir / V11_FILENAME)

    env_ws = os.environ.get("DROK_GRASP_WS")
    if env_ws:
        candidates.append(
            Path(env_ws) / "tools" / V11_FILENAME
        )

    # Common legacy/default location.
    candidates.append(
        Path.home()
        / "DROK_grasp_ws"
        / "tools"
        / V11_FILENAME
    )

    # Also search the current directory and its parents.
    cwd = Path.cwd().resolve()
    for base in [cwd, *cwd.parents]:
        candidates.append(
            base / "tools" / V11_FILENAME
        )
        candidates.append(
            base / V11_FILENAME
        )

    # Deduplicate while preserving order.
    seen = set()
    result = []

    for item in candidates:
        p = item.expanduser().resolve()
        if p in seen:
            continue
        seen.add(p)
        result.append(p)

    return result


def discover_v11(cli_path: Optional[Path]) -> Path:
    tried = _candidate_v11_paths(cli_path)

    for path in tried:
        if path.is_file():
            return path

    message = [
        "interactive_box_ik_grasp_v11.py를 찾지 못했습니다.",
        "찾아본 경로:",
    ]

    message.extend(
        f"  {path}"
        for path in tried
    )

    message.extend([
        "",
        "다른 컴퓨터에서는 다음 중 하나를 사용하세요:",
        "  python3 drok_gripper_opening_calibration_jog.py --v11 /path/to/interactive_box_ik_grasp_v11.py",
        "또는:",
        "  export DROK_V11_PATH=/path/to/interactive_box_ik_grasp_v11.py",
    ])

    raise FileNotFoundError(
        "\n".join(message)
    )


def load_v11(v11_path: Path):
    # Validate the file already on disk before importing it.
    source = v11_path.read_text(
        encoding="utf-8"
    )
    compile(
        source,
        str(v11_path),
        "exec",
    )

    spec = importlib.util.spec_from_file_location(
        "drok_v11_gripper_core",
        v11_path,
    )

    if spec is None or spec.loader is None:
        raise RuntimeError(
            f"Failed to import: {v11_path}"
        )

    module = importlib.util.module_from_spec(
        spec
    )
    spec.loader.exec_module(module)
    return module


def _find_top_level_assignments(
    source: str,
) -> Dict[str, ast.Assign]:
    tree = ast.parse(source)

    found: Dict[str, ast.Assign] = {}

    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue

        for target in node.targets:
            if isinstance(target, ast.Name):
                found[target.id] = node

    return found


def patch_assignments_ast(
    source: str,
    replacements: Dict[str, float],
) -> str:
    """
    Replace complete top-level assignment statements by AST line ranges.

    This safely handles both:
        NAME = 123.4

    and:
        NAME = (
            123.4
        )

    without ever consuming the newline of the next assignment.
    """
    assignments = _find_top_level_assignments(
        source
    )

    lines = source.splitlines(
        keepends=True
    )

    edits = []

    for name, value in replacements.items():
        node = assignments.get(name)

        if node is None:
            raise RuntimeError(
                f"{name} assignment not found"
            )

        if not hasattr(node, "end_lineno"):
            raise RuntimeError(
                "Python AST end_lineno unavailable"
            )

        start = int(node.lineno) - 1
        end = int(node.end_lineno)

        newline = "\n"

        if (
            end - 1 < len(lines)
            and lines[end - 1].endswith("\r\n")
        ):
            newline = "\r\n"

        replacement = (
            f"{name} = {float(value):.9f}"
            f"{newline}"
        )

        edits.append(
            (
                start,
                end,
                replacement,
            )
        )

    # Apply from bottom to top so earlier line numbers do not shift.
    for start, end, replacement in sorted(
        edits,
        key=lambda item: item[0],
        reverse=True,
    ):
        lines[start:end] = [
            replacement
        ]

    updated = "".join(lines)

    # Hard validation before any disk write.
    compile(
        updated,
        "<patched_v11>",
        "exec",
    )

    return updated


def atomic_save_v11(
    v11_path: Path,
    updated: str,
) -> Path:
    timestamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    backup = v11_path.with_name(
        v11_path.name
        + f".bak_gripper_{timestamp}"
    )

    shutil.copy2(
        v11_path,
        backup,
    )

    fd, temp_name = tempfile.mkstemp(
        prefix=v11_path.name + ".tmp.",
        dir=str(v11_path.parent),
        text=True,
    )

    temp_path = Path(temp_name)

    try:
        with os.fdopen(
            fd,
            "w",
            encoding="utf-8",
        ) as file:
            file.write(updated)
            file.flush()
            os.fsync(file.fileno())

        # Validate the actual temp file once more.
        temp_source = temp_path.read_text(
            encoding="utf-8"
        )

        compile(
            temp_source,
            str(v11_path),
            "exec",
        )

        os.replace(
            temp_path,
            v11_path,
        )

    except Exception:
        try:
            temp_path.unlink(
                missing_ok=True
            )
        except Exception:
            pass
        raise

    return backup


def current_saved_values(core):
    return {
        "open_topic": (
            float(
                core.GRIPPER_OPEN_PROTOCOL_DEG
            )
            / float(
                core.GRIPPER_TOPIC_DIVISOR
            )
        ),
        "close_topic": (
            float(
                core.GRIPPER_CLOSE_PROTOCOL_DEG
            )
            / float(
                core.GRIPPER_TOPIC_DIVISOR
            )
        ),
        "open_protocol": float(
            core.GRIPPER_OPEN_PROTOCOL_DEG
        ),
        "close_protocol": float(
            core.GRIPPER_CLOSE_PROTOCOL_DEG
        ),
        "open_gap": float(
            core.GRIPPER_OPEN_GAP_CM
        ),
        "close_gap": float(
            core.GRIPPER_CLOSE_GAP_CM
        ),
    }


def estimate_gap(core, topic_deg: float) -> Optional[float]:
    values = current_saved_values(
        core
    )

    a = values["open_topic"]
    b = values["close_topic"]

    lower = min(a, b)
    upper = max(a, b)

    # Do not print absurd extrapolated gap values when feedback is on another
    # multi-turn numerical branch.
    if not (
        lower <= topic_deg <= upper
    ):
        return None

    if abs(b - a) < 1.0e-9:
        return None

    ratio = (
        (topic_deg - a)
        / (b - a)
    )

    return (
        values["open_gap"]
        + ratio
        * (
            values["close_gap"]
            - values["open_gap"]
        )
    )


def refresh_feedback(node) -> Optional[float]:
    node.refresh()

    value = node.gripper_topic_deg

    if value is None:
        return None

    return float(value)


def show_status(core, node) -> Optional[float]:
    topic_deg = refresh_feedback(
        node
    )

    if topic_deg is None:
        print(
            "[WARN] gripper feedback 없음"
        )
        return None

    protocol_deg = (
        topic_deg
        * core.GRIPPER_TOPIC_DIVISOR
    )

    gap = estimate_gap(
        core,
        topic_deg,
    )

    print()
    print(
        f"feedback/topic angle : "
        f"{topic_deg:+.6f} deg"
    )

    print(
        f"protocol angle       : "
        f"{protocol_deg:+.6f} deg"
    )

    if gap is None:
        print(
            "estimated gap        : N/A "
            "(현재 feedback이 저장된 OPEN~GRASP 수치 범위 밖)"
        )
    else:
        print(
            f"estimated gap        : "
            f"{gap:.4f} cm"
        )

    return topic_deg


def show_saved(core, v11_path: Path):
    values = current_saved_values(
        core
    )

    print()
    print("=" * 76)
    print(" CURRENT SAVED GRIPPER CALIBRATION")
    print("=" * 76)

    print(
        f"OPEN : gap={values['open_gap']:.4f} cm | "
        f"topic={values['open_topic']:+.6f} deg | "
        f"protocol={values['open_protocol']:+.6f} deg"
    )

    print(
        f"GRASP: gap={values['close_gap']:.4f} cm | "
        f"topic={values['close_topic']:+.6f} deg | "
        f"protocol={values['close_protocol']:+.6f} deg"
    )

    print(
        f"v11  : {v11_path}"
    )

    print("=" * 76)


def require_large_save_confirmation(
    current_topic: float,
    saved_topic: float,
    label: str,
) -> bool:
    delta = abs(
        current_topic
        - saved_topic
    )

    if delta <= LARGE_SAVE_DELTA_TOPIC_DEG:
        return True

    print()
    print(
        f"[WARNING] 현재 {label} 후보가 기존 저장값과 "
        f"{delta:.3f} topic-deg 차이납니다."
    )

    print(
        "멀티턴 feedback의 숫자 branch가 바뀐 경우일 수 있습니다."
    )

    print(
        "실제 그리퍼 위치를 눈으로 확인한 경우에만 저장하세요."
    )

    answer = input(
        "정말 저장하려면 정확히 SAVE 입력: "
    ).strip()

    return answer == "SAVE"


def save_current(
    core,
    node,
    v11_path: Path,
    kind: str,
    measured_gap_cm: Optional[float],
) -> bool:
    topic_deg = refresh_feedback(
        node
    )

    if topic_deg is None:
        print(
            "[SAVE ERROR] gripper feedback 없음"
        )
        return False

    protocol_deg = (
        topic_deg
        * core.GRIPPER_TOPIC_DIVISOR
    )

    values = current_saved_values(
        core
    )

    kind = kind.lower()

    if kind == "open":
        protocol_name = (
            "GRIPPER_OPEN_PROTOCOL_DEG"
        )
        gap_name = (
            "GRIPPER_OPEN_GAP_CM"
        )
        saved_topic = values[
            "open_topic"
        ]
        label = "FULL OPEN"

    elif kind in {
        "grasp",
        "close",
    }:
        protocol_name = (
            "GRIPPER_CLOSE_PROTOCOL_DEG"
        )
        gap_name = (
            "GRIPPER_CLOSE_GAP_CM"
        )
        saved_topic = values[
            "close_topic"
        ]
        label = "GRASP/CLOSE"

    else:
        print(
            "[INPUT] save open 또는 save grasp"
        )
        return False

    if not require_large_save_confirmation(
        topic_deg,
        saved_topic,
        label,
    ):
        print(
            "[CANCEL] 저장하지 않았습니다."
        )
        return False

    replacements = {
        protocol_name: protocol_deg,
    }

    if measured_gap_cm is not None:
        if (
            not math.isfinite(
                measured_gap_cm
            )
            or measured_gap_cm <= 0.0
        ):
            print(
                "[SAVE ERROR] gap_cm must be > 0"
            )
            return False

        replacements[
            gap_name
        ] = measured_gap_cm

    original = v11_path.read_text(
        encoding="utf-8"
    )

    updated = patch_assignments_ast(
        original,
        replacements,
    )

    backup = atomic_save_v11(
        v11_path,
        updated,
    )

    # Keep this running process consistent with the newly written file.
    if kind == "open":
        core.GRIPPER_OPEN_PROTOCOL_DEG = (
            protocol_deg
        )
        core.GRIPPER_OPEN_TOPIC_DEG = (
            topic_deg
        )

        if measured_gap_cm is not None:
            core.GRIPPER_OPEN_GAP_CM = (
                float(
                    measured_gap_cm
                )
            )

    else:
        core.GRIPPER_CLOSE_PROTOCOL_DEG = (
            protocol_deg
        )
        core.GRIPPER_CLOSE_TOPIC_DEG = (
            topic_deg
        )

        if measured_gap_cm is not None:
            core.GRIPPER_CLOSE_GAP_CM = (
                float(
                    measured_gap_cm
                )
            )

    print()
    print("=" * 76)
    print(
        f"[SAVED + APPLIED] {label}"
    )
    print(
        f"topic    : "
        f"{topic_deg:+.6f} deg"
    )
    print(
        f"protocol : "
        f"{protocol_deg:+.6f} deg"
    )

    if measured_gap_cm is not None:
        print(
            f"gap      : "
            f"{measured_gap_cm:.4f} cm"
        )

    print(
        f"v11 file : {v11_path}"
    )
    print(
        f"backup   : {backup}"
    )
    print("=" * 76)

    return True


def send_relative_jog(
    core,
    node,
    delta_topic_deg: float,
) -> Optional[float]:
    if (
        not math.isfinite(
            delta_topic_deg
        )
    ):
        print(
            "[INPUT] finite number required"
        )
        return None

    if abs(
        delta_topic_deg
    ) > MAX_SINGLE_JOG_TOPIC_DEG:
        print(
            f"[BLOCK] 한 번의 jog는 "
            f"±{MAX_SINGLE_JOG_TOPIC_DEG:g} topic-deg 이하만 허용합니다."
        )
        return None

    current = refresh_feedback(
        node
    )

    if current is None:
        print(
            "[ERROR] current gripper feedback 없음"
        )
        return None

    target_topic = (
        current
        + delta_topic_deg
    )

    target_protocol = (
        target_topic
        * core.GRIPPER_TOPIC_DIVISOR
    )

    print()
    print(
        f"CURRENT topic : "
        f"{current:+.6f} deg"
    )
    print(
        f"JOG           : "
        f"{delta_topic_deg:+.6f} deg"
    )
    print(
        f"TARGET topic  : "
        f"{target_topic:+.6f} deg"
    )
    print(
        f"TARGET protocol: "
        f"{target_protocol:+.6f} deg"
    )

    ok = core.send_gripper_protocol(
        node,
        target_protocol,
    )

    if not ok:
        print(
            "[ERROR] jog target arrival failed"
        )
        return None

    return show_status(
        core,
        node
    )


def send_nearby_set(
    core,
    node,
    target_topic: float,
) -> Optional[float]:
    current = refresh_feedback(
        node
    )

    if current is None:
        print(
            "[ERROR] current gripper feedback 없음"
        )
        return None

    delta = (
        target_topic
        - current
    )

    if abs(delta) > MAX_SINGLE_JOG_TOPIC_DEG:
        print()
        print(
            f"[BLOCK] set target is {delta:+.3f} topic-deg away."
        )
        print(
            f"안전을 위해 set도 현재값에서 "
            f"±{MAX_SINGLE_JOG_TOPIC_DEG:g}° 이내만 허용합니다."
        )
        print(
            "+1/-1 같은 상대 jog로 조금씩 이동하세요."
        )
        return None

    return send_relative_jog(
        core,
        node,
        delta,
    )


def main() -> int:
    args = parse_args()

    v11_path = discover_v11(
        args.v11
    )

    print("=" * 76)
    print(" DROK GRIPPER CALIBRATION JOG v4")
    print("=" * 76)
    print(
        f"v11 detected: {v11_path}"
    )
    print(
        "CAN interface/bitrate 변경 없음"
    )
    print(
        "Motor ROM/limit write 없음"
    )
    print("=" * 76)

    core = load_v11(
        v11_path
    )

    # Only the calibration run uses the slower runtime speed.
    core.GRIPPER_SPEED_DPS = int(
        TUNE_GRIPPER_SPEED_DPS
    )

    show_saved(
        core,
        v11_path,
    )

    print()
    print(
        "-1 = CURRENT feedback 기준 1° 더 OPEN 방향"
    )
    print(
        "+1 = CURRENT feedback 기준 1° 더 CLOSE 방향"
    )
    print()
    print(
        "commands:"
    )
    print(
        "  -1 / +1 / -0.5 / +2"
    )
    print(
        "  s"
    )
    print(
        "  save open"
    )
    print(
        "  save open 14.5"
    )
    print(
        "  save grasp"
    )
    print(
        "  save grasp 9.0"
    )
    print(
        "  saved"
    )
    print(
        "  set <nearby_topic_deg>"
    )
    print(
        "  q"
    )

    rclpy.init()

    node = None

    try:
        node = core.RealFeedbackNode()

        if not node.wait_for_feedback():
            print(
                "[ERROR] arm/gripper feedback 없음"
            )
            return 2

        current = show_status(
            core,
            node,
        )

        if current is None:
            return 3

        while True:
            print()
            command = input(
                "gripper> "
            ).strip().lower()

            if command in {
                "q",
                "quit",
                "exit",
            }:
                return 0

            if command in {
                "s",
                "status",
            }:
                show_status(
                    core,
                    node,
                )
                continue

            if command in {
                "saved",
                "cal",
                "calibration",
            }:
                show_saved(
                    core,
                    v11_path,
                )
                continue

            if command.startswith(
                "save "
            ):
                parts = command.split()

                if len(parts) not in {
                    2,
                    3,
                }:
                    print(
                        "[INPUT] save open [gap_cm] "
                        "또는 save grasp [gap_cm]"
                    )
                    continue

                gap = None

                if len(parts) == 3:
                    try:
                        gap = float(
                            parts[2]
                        )
                    except ValueError:
                        print(
                            "[INPUT] gap_cm 숫자 필요"
                        )
                        continue

                save_current(
                    core,
                    node,
                    v11_path,
                    parts[1],
                    gap,
                )
                continue

            if command.startswith(
                "set "
            ):
                try:
                    target = float(
                        command.split(
                            None,
                            1,
                        )[1]
                    )
                except ValueError:
                    print(
                        "[INPUT] set <nearby_topic_deg>"
                    )
                    continue

                send_nearby_set(
                    core,
                    node,
                    target,
                )
                continue

            try:
                delta = float(
                    command
                )
            except ValueError:
                print(
                    "[INPUT] -1 / +1 / s / save open / "
                    "save grasp / saved / q"
                )
                continue

            send_relative_jog(
                core,
                node,
                delta,
            )

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
