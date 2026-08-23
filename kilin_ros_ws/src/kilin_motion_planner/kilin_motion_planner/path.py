from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PathHorizon:
    time_s: FloatArray
    x_m: FloatArray
    y_m: FloatArray
    yaw_rad: FloatArray
    speed_m_s: FloatArray

    def __post_init__(self) -> None:
        arrays = (self.time_s, self.x_m, self.y_m, self.yaw_rad, self.speed_m_s)
        lengths = {np.asarray(item).size for item in arrays}
        if len(lengths) != 1:
            raise ValueError("All path-horizon arrays must have equal length")
        if np.asarray(self.time_s).ndim != 1:
            raise ValueError("Path-horizon arrays must be one-dimensional")
        if np.any(np.diff(self.time_s) <= 0.0):
            raise ValueError("time_s must be strictly increasing")

    @property
    def steps(self) -> int:
        return int(self.time_s.size)


def make_straight_horizon(
    *,
    start_x_m: float,
    start_y_m: float,
    yaw_rad: float,
    speed_m_s: float,
    steps: int,
    dt_s: float,
    knot_spacing_m: float | None = None,
) -> PathHorizon:
    """Create a straight reference horizon.

    When ``knot_spacing_m`` is provided, reference poses are separated in
    distance rather than by the current speed command.  ``speed_m_s`` remains
    the commanded rolling speed carried with the path for the first control
    action.
    """

    time = np.arange(steps, dtype=float) * dt_s
    if knot_spacing_m is None:
        distance = speed_m_s * time
    else:
        if knot_spacing_m <= 0.0:
            raise ValueError("knot_spacing_m must be positive")
        distance = np.arange(steps, dtype=float) * knot_spacing_m
    return PathHorizon(
        time_s=time,
        x_m=start_x_m + distance * np.cos(yaw_rad),
        y_m=start_y_m + distance * np.sin(yaw_rad),
        yaw_rad=np.full(steps, yaw_rad, dtype=float),
        speed_m_s=np.full(steps, speed_m_s, dtype=float),
    )
