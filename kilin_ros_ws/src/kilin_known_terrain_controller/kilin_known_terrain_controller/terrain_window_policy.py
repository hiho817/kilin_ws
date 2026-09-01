"""Conservative live-terrain conditioning for the first planning cycle.

The LiDAR front ROI deliberately does not observe the ground beneath or just
behind the robot.  A controller started from a verified flat stance can seed
only that initial support patch from the flat terrain observed immediately in
front.  This is not a general unknown-terrain fill policy.
"""

from __future__ import annotations

import numpy as np


def _relative_coordinates(x_coordinates_m, y_coordinates_m, origin_x_m, origin_y_m, yaw_rad):
    """Return map grid coordinates expressed in a fixed initial body frame."""

    x, y = np.meshgrid(
        np.asarray(x_coordinates_m, dtype=float),
        np.asarray(y_coordinates_m, dtype=float),
    )
    dx, dy = x - float(origin_x_m), y - float(origin_y_m)
    cosine, sine = np.cos(float(yaw_rad)), np.sin(float(yaw_rad))
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def infer_flat_height_ahead(
    x_coordinates_m,
    y_coordinates_m,
    heights_m,
    valid_mask,
    *,
    origin_x_m: float,
    origin_y_m: float,
    yaw_rad: float,
    minimum_forward_m: float,
    maximum_forward_m: float,
    half_width_m: float,
    maximum_height_span_m: float,
    minimum_inlier_fraction: float = 1.0,
):
    """Infer a flat initial support height from a bounded observed strip.

    ``None`` means the strip is unavailable or is not sufficiently flat.  The
    caller must then retain the normal unknown-terrain safety fallback.
    """

    heights = np.asarray(heights_m, dtype=float)
    valid = np.asarray(valid_mask, dtype=bool)
    forward, lateral = _relative_coordinates(
        x_coordinates_m, y_coordinates_m, origin_x_m, origin_y_m, yaw_rad
    )
    selected = (
        valid
        & (forward >= float(minimum_forward_m))
        & (forward <= float(maximum_forward_m))
        & (np.abs(lateral) <= float(half_width_m))
    )
    samples = heights[selected]
    if samples.size == 0:
        return None
    if not 0.0 < minimum_inlier_fraction <= 1.0:
        raise ValueError("minimum_inlier_fraction must be in (0, 1]")
    # A single misregistered return must not make a genuinely flat initial
    # support strip unusable.  Select the largest sorted height cluster whose
    # span respects the flatness limit, then require that it explains most of
    # the measured nodes.  A ramp occupying a material part of the strip does
    # not meet this dominant-cluster requirement and remains unsafe.
    sorted_samples = np.sort(samples)
    left = 0
    best_left, best_right = 0, 0
    for right in range(sorted_samples.size):
        while sorted_samples[right] - sorted_samples[left] > float(maximum_height_span_m):
            left += 1
        if right - left > best_right - best_left:
            best_left, best_right = left, right
    cluster = sorted_samples[best_left : best_right + 1]
    if cluster.size / samples.size < minimum_inlier_fraction:
        return None
    return float(np.median(cluster))


def seed_initial_flat_support(
    x_coordinates_m,
    y_coordinates_m,
    heights_m,
    valid_mask,
    *,
    origin_x_m: float,
    origin_y_m: float,
    yaw_rad: float,
    flat_height_m: float,
    rear_m: float,
    forward_m: float,
    half_width_m: float,
):
    """Fill unknown nodes only inside a fixed, verified initial support patch."""

    heights = np.asarray(heights_m, dtype=float).copy()
    valid = np.asarray(valid_mask, dtype=bool).copy()
    longitudinal, lateral = _relative_coordinates(
        x_coordinates_m, y_coordinates_m, origin_x_m, origin_y_m, yaw_rad
    )
    support = (
        (longitudinal >= -float(rear_m))
        & (longitudinal <= float(forward_m))
        & (np.abs(lateral) <= float(half_width_m))
    )
    fill = support & ~valid
    heights[fill] = float(flat_height_m)
    valid[fill] = True
    return heights, valid, int(np.count_nonzero(fill))


def fill_isolated_unknown_nodes(heights_m, valid_mask, *, maximum_height_span_m: float):
    """Fill a single unknown node only when all four cardinal neighbours agree.

    This tolerates a sparse LiDAR dropout without bridging an unobserved area,
    a window edge, or a terrain discontinuity.  The operation is intentionally
    one pass, so a larger hole cannot grow into a fabricated surface.
    """

    heights = np.asarray(heights_m, dtype=float).copy()
    valid = np.asarray(valid_mask, dtype=bool).copy()
    source_heights, source_valid = heights.copy(), valid.copy()
    rows, columns = valid.shape
    filled = 0
    for row in range(1, rows - 1):
        for column in range(1, columns - 1):
            if source_valid[row, column]:
                continue
            neighbours = (
                (row - 1, column),
                (row + 1, column),
                (row, column - 1),
                (row, column + 1),
            )
            values = np.asarray(
                [source_heights[r, c] for r, c in neighbours if source_valid[r, c]],
                dtype=float,
            )
            # Requiring all four sides avoids growing a contiguous gap from
            # either edge.  This remains sufficient for an actual one-node
            # LiDAR dropout at the planner grid resolution.
            if values.size != 4:
                continue
            if float(np.max(values) - np.min(values)) > float(maximum_height_span_m):
                continue
            heights[row, column] = float(np.mean(values))
            valid[row, column] = True
            filled += 1
    return heights, valid, filled
