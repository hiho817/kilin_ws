"""Frame-independent elevation-window extraction utilities."""

from __future__ import annotations

import numpy as np


def transform_points(points_m, rotation, translation):
    """Apply a rigid transform to an N-by-3 point array."""
    return np.asarray(points_m, dtype=float) @ np.asarray(rotation, dtype=float).T + np.asarray(translation, dtype=float)


def front_roi(points_in_hip_m, minimum_x_m, maximum_x_m, half_width_m, minimum_z_m, maximum_z_m):
    """Keep only the body-fixed forward terrain region."""
    points = np.asarray(points_in_hip_m, dtype=float)
    if points.size == 0:
        return np.empty((0, 3), dtype=float)
    keep = ((points[:, 0] >= minimum_x_m) & (points[:, 0] <= maximum_x_m) &
            (np.abs(points[:, 1]) <= half_width_m) &
            (points[:, 2] >= minimum_z_m) & (points[:, 2] <= maximum_z_m))
    return points[keep]


def retained_window_roi(points_in_hip_m, rear_m, forward_m, half_width_m, margin_m=0.0):
    """Keep previously observed points that can still contribute to the local window.

    Unlike :func:`front_roi`, this accepts a small region behind the robot.  It
    is used only after a point was admitted by ``front_roi`` when it was first
    observed, so it preserves causal forward observations without admitting
    arbitrary rear or body points.
    """
    points = np.asarray(points_in_hip_m, dtype=float)
    if points.size == 0:
        return np.empty((0, 3), dtype=float)
    keep = ((points[:, 0] >= -rear_m - margin_m) &
            (points[:, 0] <= forward_m + margin_m) &
            (np.abs(points[:, 1]) <= half_width_m + margin_m))
    return points[keep]


def voxel_downsample(points_m, voxel_m):
    """Keep one representative per fixed-frame voxel for bounded map memory."""
    points = np.asarray(points_m, dtype=float)
    if points.size == 0 or voxel_m <= 0.0:
        return points.reshape((-1, 3))
    points = points.reshape((-1, 3))
    keys = np.floor(points / voxel_m).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(indices)]


def grid_coordinates(
    center_x_m,
    center_y_m,
    forward_m,
    rear_m,
    half_width_m,
    resolution_m,
    *,
    align_to_resolution=False,
):
    """Return map-frame grid axes centered laterally on the robot.

    A continuously translated grid makes the same fixed-frame point fall into
    different elevation cells whenever odometry jitters by a few millimetres.
    ``align_to_resolution`` anchors the rolling window to the fixed map lattice
    instead: its bounds move only after the robot crosses one grid resolution.
    """
    lower_x, upper_x = center_x_m - rear_m, center_x_m + forward_m
    lower_y, upper_y = center_y_m - half_width_m, center_y_m + half_width_m
    if align_to_resolution:
        lower_x = np.floor(lower_x / resolution_m) * resolution_m
        upper_x = np.ceil(upper_x / resolution_m) * resolution_m
        lower_y = np.floor(lower_y / resolution_m) * resolution_m
        upper_y = np.ceil(upper_y / resolution_m) * resolution_m
    xs = np.arange(lower_x, upper_x + resolution_m * 0.1, resolution_m)
    ys = np.arange(lower_y, upper_y + resolution_m * 0.1, resolution_m)
    return xs, ys


def elevation_from_points(
    points_m,
    xs,
    ys,
    resolution_m,
    minimum_points=2,
    percentile=50.0,
    maximum_vertical_span_m=None,
):
    """Bin map-frame points into a robust 2.5-D terrain surface.

    A cell spanning distinct vertical surfaces (for example floor and ceiling)
    is marked unknown when ``maximum_vertical_span_m`` is set.  Unknown is the
    safe result: it prevents an overhead return becoming a ground height.
    """
    width, height = len(xs), len(ys)
    elevation = np.zeros((height, width), dtype=np.float32)
    valid = np.zeros((height, width), dtype=bool)
    counts = np.zeros((height, width), dtype=np.uint16)
    points = np.asarray(points_m, dtype=float)
    if points.size == 0:
        return elevation, valid, counts
    points = points.reshape((-1, 3))
    points = points[np.isfinite(points).all(axis=1)]
    if not len(points):
        return elevation, valid, counts
    ix = np.floor((points[:, 0] - xs[0]) / resolution_m).astype(int)
    iy = np.floor((points[:, 1] - ys[0]) / resolution_m).astype(int)
    inside = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    ix, iy, z = ix[inside], iy[inside], points[inside, 2]
    if not len(z):
        return elevation, valid, counts
    flat = iy * width + ix
    order = np.argsort(flat, kind="stable")
    flat, z = flat[order], z[order]
    starts = np.r_[0, np.flatnonzero(np.diff(flat)) + 1]
    ends = np.r_[starts[1:], len(flat)]
    for start, end in zip(starts, ends):
        count = end - start
        cell = int(flat[start])
        row, column = divmod(cell, width)
        counts[row, column] = min(count, np.iinfo(np.uint16).max)
        if count >= minimum_points:
            if (maximum_vertical_span_m is not None and
                    np.ptp(z[start:end]) > maximum_vertical_span_m):
                continue
            elevation[row, column] = np.percentile(z[start:end], percentile)
            valid[row, column] = True
    return elevation, valid, counts


def update_temporal_cell_memory(
    memory,
    xs,
    ys,
    elevation,
    valid,
    *,
    resolution_m,
    observed_ns,
    maximum_height_deviation_m,
    replacement_observations,
):
    """Update bounded per-cell terrain memory from measured elevation cells.

    ``memory`` is deliberately a plain dictionary so the ROS node owns its
    lifetime.  A one-off conflicting height cannot overwrite a previously
    measured surface; it needs repeated, mutually consistent observations.
    """
    if replacement_observations < 1:
        raise ValueError("replacement_observations must be positive")
    for row, column in zip(*np.nonzero(valid)):
        # Keep the grid-coordinate origin in the key.  Integer rounding of a
        # half-cell offset would collapse adjacent nodes (for example -0.05 m
        # and +0.05 m at a 0.10 m resolution) into the same cache entry.
        key = (round(float(xs[column]), 6), round(float(ys[row]), 6))
        height = float(elevation[row, column])
        entry = memory.get(key)
        if entry is None:
            memory[key] = {
                "height_m": height,
                "observed_ns": int(observed_ns),
                "candidate_height_m": None,
                "candidate_count": 0,
            }
            continue
        if abs(height - entry["height_m"]) <= maximum_height_deviation_m:
            entry["height_m"] = height
            entry["observed_ns"] = int(observed_ns)
            entry["candidate_height_m"] = None
            entry["candidate_count"] = 0
            continue
        candidate = entry["candidate_height_m"]
        if candidate is not None and abs(height - candidate) <= maximum_height_deviation_m:
            entry["candidate_height_m"] = height
            entry["candidate_count"] += 1
        else:
            entry["candidate_height_m"] = height
            entry["candidate_count"] = 1
        if entry["candidate_count"] >= replacement_observations:
            entry["height_m"] = entry["candidate_height_m"]
            entry["observed_ns"] = int(observed_ns)
            entry["candidate_height_m"] = None
            entry["candidate_count"] = 0
    return memory


def fill_from_temporal_cell_memory(
    memory,
    xs,
    ys,
    elevation,
    valid,
    *,
    resolution_m,
    now_ns,
    hold_ns,
    maximum_age_ns,
):
    """Fill only unknown cells from recently measured, non-stale memory."""
    elevation = np.asarray(elevation, dtype=np.float32).copy()
    valid = np.asarray(valid, dtype=bool).copy()
    stale = [key for key, entry in memory.items() if now_ns - entry["observed_ns"] > maximum_age_ns]
    for key in stale:
        del memory[key]
    filled = 0
    for row, column in zip(*np.nonzero(~valid)):
        key = (round(float(xs[column]), 6), round(float(ys[row]), 6))
        entry = memory.get(key)
        if entry is None or now_ns - entry["observed_ns"] > hold_ns:
            continue
        elevation[row, column] = entry["height_m"]
        valid[row, column] = True
        filled += 1
    return elevation, valid, filled


def prune_cell_memory_to_bounds(
    memory,
    *,
    minimum_x_m,
    maximum_x_m,
    minimum_y_m,
    maximum_y_m,
):
    """Discard cached elevation cells outside the current local-map bounds.

    The cache stores map-frame terrain cells rather than raw point clouds.  A
    spatial bound is therefore as important as its time bound: it prevents a
    long traversal from gradually turning the local estimator into a global
    map and keeps each publish operation proportional to the planning window.
    """
    outside = [
        key for key in memory
        if (key[0] < minimum_x_m or key[0] > maximum_x_m or
            key[1] < minimum_y_m or key[1] > maximum_y_m)
    ]
    for key in outside:
        del memory[key]
    return memory
