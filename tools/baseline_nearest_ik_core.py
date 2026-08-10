#!/usr/bin/env python3
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

JOINT_NAMES = [
    "JOINT1", "JOINT2", "JOINT3",
    "JOINT4", "JOINT5", "JOINT6",
]


def load_joint_limits(
    urdf_path: Path,
) -> Dict[str, Tuple[float, float]]:
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF 파일이 없습니다: {urdf_path}"
        )

    root = ET.parse(urdf_path).getroot()
    limits: Dict[str, Tuple[float, float]] = {}

    for joint_element in root.findall("joint"):
        name = joint_element.attrib.get("name", "")
        if name not in JOINT_NAMES:
            continue

        limit_element = joint_element.find("limit")
        if limit_element is None:
            raise RuntimeError(
                f"{name}에 limit 태그가 없습니다."
            )

        limits[name] = (
            float(limit_element.attrib["lower"]),
            float(limit_element.attrib["upper"]),
        )

    missing = [
        joint for joint in JOINT_NAMES
        if joint not in limits
    ]
    if missing:
        raise RuntimeError(
            "URDF에서 joint limit을 찾지 못했습니다: "
            + ", ".join(missing)
        )
    return limits


def clip_to_limits(
    q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> List[float]:
    result = []
    for joint, value in zip(JOINT_NAMES, q):
        lower, upper = limits[joint]
        result.append(
            min(max(float(value), lower), upper)
        )
    return result


def halton_value(index: int, base: int) -> float:
    result = 0.0
    fraction = 1.0 / base
    current = index
    while current > 0:
        result += fraction * (current % base)
        current //= base
        fraction /= base
    return result


def make_seed_set(
    reference_q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> List[List[float]]:
    seeds: List[List[float]] = []

    def append_unique(seed: Sequence[float]) -> None:
        clipped = clip_to_limits(seed, limits)
        for existing in seeds:
            error = max(
                abs(a - b)
                for a, b in zip(existing, clipped)
            )
            if error < 1.0e-8:
                return
        seeds.append(clipped)

    append_unique(reference_q)
    append_unique([0.0] * 6)

    for joint_index in range(6):
        for offset_deg in [-15.0, 15.0]:
            seed = list(reference_q)
            seed[joint_index] += math.radians(offset_deg)
            append_unique(seed)

    for joint_index in [3, 4, 5]:
        for offset_deg in [-45.0, 45.0]:
            seed = list(reference_q)
            seed[joint_index] += math.radians(offset_deg)
            append_unique(seed)

    primes = [2, 3, 5, 7, 11, 13]
    for sample_index in range(1, 13):
        seed = []
        for joint_index, joint in enumerate(JOINT_NAMES):
            lower, upper = limits[joint]
            ratio = halton_value(
                sample_index,
                primes[joint_index],
            )
            seed.append(
                lower + ratio * (upper - lower)
            )
        append_unique(seed)

    return seeds


def nearest_equivalent_angle(
    value: float,
    reference: float,
    lower: float,
    upper: float,
) -> Optional[float]:
    candidates = []
    for winding in range(-3, 4):
        candidate = value + winding * 2.0 * math.pi
        if lower - 1.0e-8 <= candidate <= upper + 1.0e-8:
            candidates.append(candidate)

    if not candidates:
        return None

    return min(
        candidates,
        key=lambda candidate: abs(candidate - reference),
    )


def normalize_candidate(
    candidate: Sequence[float],
    reference_q: Sequence[float],
    limits: Dict[str, Tuple[float, float]],
) -> Optional[List[float]]:
    normalized = []

    for joint, value, reference in zip(
        JOINT_NAMES,
        candidate,
        reference_q,
    ):
        lower, upper = limits[joint]
        equivalent = nearest_equivalent_angle(
            float(value),
            float(reference),
            lower,
            upper,
        )
        if equivalent is None:
            return None
        normalized.append(equivalent)

    return normalized


def is_duplicate(
    solutions: Sequence[Sequence[float]],
    candidate: Sequence[float],
    tolerance: float = 1.0e-4,
) -> bool:
    return any(
        max(
            abs(a - b)
            for a, b in zip(solution, candidate)
        ) < tolerance
        for solution in solutions
    )


def candidate_score(
    candidate: Sequence[float],
    reference_q: Sequence[float],
) -> Tuple[float, float, float]:
    deltas = [
        abs(candidate_value - reference_value)
        for candidate_value, reference_value
        in zip(candidate, reference_q)
    ]

    return (
        max(deltas),
        sum(deltas),
        sum(delta * delta for delta in deltas),
    )
