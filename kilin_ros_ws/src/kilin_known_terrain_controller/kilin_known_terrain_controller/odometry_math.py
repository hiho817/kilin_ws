from __future__ import annotations

import numpy as np


def wrap_angle(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def relative_planar_pose(
    pose_xy_yaw: tuple[float, float, float],
    origin_xy_yaw: tuple[float, float, float],
    anchor_xy_yaw: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    """Express a planar pose relative to origin, then place it at anchor."""
    x, y, yaw = pose_xy_yaw
    origin_x, origin_y, origin_yaw = origin_xy_yaw
    anchor_x, anchor_y, anchor_yaw = anchor_xy_yaw
    delta_x = x - origin_x
    delta_y = y - origin_y
    cosine = float(np.cos(origin_yaw))
    sine = float(np.sin(origin_yaw))
    local_x = cosine * delta_x + sine * delta_y
    local_y = -sine * delta_x + cosine * delta_y
    anchor_cosine = float(np.cos(anchor_yaw))
    anchor_sine = float(np.sin(anchor_yaw))
    return (
        anchor_x + anchor_cosine * local_x - anchor_sine * local_y,
        anchor_y + anchor_sine * local_x + anchor_cosine * local_y,
        wrap_angle(anchor_yaw + wrap_angle(yaw - origin_yaw)),
    )
