from __future__ import annotations

import math


MODULE_NAMES = ("A", "B", "C", "D")
CORNER_NAMES = ("FL", "FR", "RL", "RR")


def radians_per_second_to_rpm10(value: float) -> float:
    """Convert planner/Isaac rad/s to the Kilin bridge's RPM-times-ten unit."""

    return float(value) * 60.0 * 10.0 / (2.0 * math.pi)


def validate_four(values, name: str) -> tuple[float, float, float, float]:
    result = tuple(float(value) for value in values)
    if len(result) != 4 or not all(math.isfinite(value) for value in result):
        raise ValueError(f"{name} must contain four finite values")
    return result  # type: ignore[return-value]
