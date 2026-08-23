from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]
BoolArray = NDArray[np.bool_]
IntArray = NDArray[np.int64]


class TerrainDataUnavailable(ValueError):
    """Raised when a requested live-terrain sample is unknown or out of bounds."""


@dataclass(frozen=True)
class ElevationSample:
    """Bilinearly interpolated samples from an :class:`ElevationWindow`."""

    height_m: FloatArray
    height_std_m: FloatArray
    gradient_x: FloatArray
    gradient_y: FloatArray
    valid: BoolArray


def regular_grid_coordinates(
    minimum_m: float,
    maximum_m: float,
    resolution_m: float,
) -> FloatArray:
    """Return regular grid-node coordinates including both requested limits."""

    if not np.isfinite([minimum_m, maximum_m, resolution_m]).all():
        raise ValueError("Grid limits and resolution must be finite")
    if maximum_m <= minimum_m:
        raise ValueError("maximum_m must be greater than minimum_m")
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive")

    intervals = max(1, int(np.ceil((maximum_m - minimum_m) / resolution_m)))
    return np.linspace(minimum_m, maximum_m, intervals + 1, dtype=float)


@dataclass(frozen=True)
class ElevationWindow:
    """Rolling 2.5D terrain window with explicit uncertainty and validity.

    The coordinate arrays describe grid nodes, not cell edges. A bilinear
    query is valid only when the four surrounding nodes are valid. Unknown
    nodes remain NaN and are never replaced by a flat-ground assumption.
    """

    x_coordinates_m: FloatArray
    y_coordinates_m: FloatArray
    heights_m: FloatArray
    valid_mask: BoolArray
    height_std_m: FloatArray | None = None
    points_per_node: IntArray | None = None
    timestamp_s: float = 0.0
    frame_id: str = "map"

    def __post_init__(self) -> None:
        x = np.asarray(self.x_coordinates_m, dtype=float)
        y = np.asarray(self.y_coordinates_m, dtype=float)
        z = np.asarray(self.heights_m, dtype=float)
        valid = np.asarray(self.valid_mask, dtype=bool)
        shape = (y.size, x.size)

        if x.ndim != 1 or y.ndim != 1 or x.size < 2 or y.size < 2:
            raise ValueError("Grid coordinate arrays must be one-dimensional and nontrivial")
        if not np.isfinite(x).all() or not np.isfinite(y).all():
            raise ValueError("Grid coordinates must be finite")
        if np.any(np.diff(x) <= 0.0) or np.any(np.diff(y) <= 0.0):
            raise ValueError("Grid coordinates must be strictly increasing")
        if z.shape != shape or valid.shape != shape:
            raise ValueError("heights_m and valid_mask must have shape (len(y), len(x))")
        if np.any(valid & ~np.isfinite(z)):
            raise ValueError("Every valid terrain node must have a finite height")
        if not np.isfinite(self.timestamp_s):
            raise ValueError("timestamp_s must be finite")
        if not self.frame_id:
            raise ValueError("frame_id must not be empty")

        standard_deviation = (
            np.zeros(shape, dtype=float)
            if self.height_std_m is None
            else np.asarray(self.height_std_m, dtype=float)
        )
        if standard_deviation.shape != shape:
            raise ValueError("height_std_m must have shape (len(y), len(x))")
        if np.any(valid & (~np.isfinite(standard_deviation) | (standard_deviation < 0.0))):
            raise ValueError("Valid height uncertainties must be finite and nonnegative")

        counts = (
            np.zeros(shape, dtype=np.int64)
            if self.points_per_node is None
            else np.asarray(self.points_per_node, dtype=np.int64)
        )
        if counts.shape != shape or np.any(counts < 0):
            raise ValueError("points_per_node must have grid shape and nonnegative values")

        # Canonicalize every invalid value so accidental direct array use also
        # exposes unknown terrain instead of silently reading a stale number.
        z = np.where(valid, z, np.nan)
        standard_deviation = np.where(valid, standard_deviation, np.nan)

        object.__setattr__(self, "x_coordinates_m", x)
        object.__setattr__(self, "y_coordinates_m", y)
        object.__setattr__(self, "heights_m", z)
        object.__setattr__(self, "valid_mask", valid)
        object.__setattr__(self, "height_std_m", standard_deviation)
        object.__setattr__(self, "points_per_node", counts)

    @classmethod
    def from_ground_points(
        cls,
        points_xyz_m: ArrayLike,
        *,
        x_coordinates_m: ArrayLike,
        y_coordinates_m: ArrayLike,
        min_points_per_node: int = 1,
        timestamp_s: float = 0.0,
        frame_id: str = "map",
    ) -> "ElevationWindow":
        """Bin prefiltered ground points and estimate node height by the median.

        The input is deliberately named ``ground_points``: obstacle removal
        and transformation into ``frame_id`` belong to the sensor front end.
        Within each nearest grid-node bin, height is the median and uncertainty
        is the scaled median absolute deviation (1.4826 * MAD).
        """

        points = np.asarray(points_xyz_m, dtype=float)
        x = np.asarray(x_coordinates_m, dtype=float)
        y = np.asarray(y_coordinates_m, dtype=float)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("points_xyz_m must have shape (N, 3)")
        if min_points_per_node < 1:
            raise ValueError("min_points_per_node must be at least 1")
        if x.ndim != 1 or y.ndim != 1 or x.size < 2 or y.size < 2:
            raise ValueError("Grid coordinate arrays must be one-dimensional and nontrivial")
        if np.any(np.diff(x) <= 0.0) or np.any(np.diff(y) <= 0.0):
            raise ValueError("Grid coordinates must be strictly increasing")

        finite_points = points[np.isfinite(points).all(axis=1)]
        shape = (y.size, x.size)
        heights = np.full(shape, np.nan, dtype=float)
        spread = np.full(shape, np.nan, dtype=float)
        counts = np.zeros(shape, dtype=np.int64)
        if finite_points.size == 0:
            return cls(
                x,
                y,
                heights,
                np.zeros(shape, dtype=bool),
                spread,
                counts,
                timestamp_s,
                frame_id,
            )

        x_edges = _node_bin_edges(x)
        y_edges = _node_bin_edges(y)
        ix = np.searchsorted(x_edges, finite_points[:, 0], side="right") - 1
        iy = np.searchsorted(y_edges, finite_points[:, 1], side="right") - 1
        inside = (ix >= 0) & (ix < x.size) & (iy >= 0) & (iy < y.size)
        ix = ix[inside]
        iy = iy[inside]
        z = finite_points[inside, 2]

        if z.size:
            flat_index = iy * x.size + ix
            order = np.argsort(flat_index, kind="stable")
            sorted_index = flat_index[order]
            sorted_z = z[order]
            unique, starts, group_counts = np.unique(
                sorted_index,
                return_index=True,
                return_counts=True,
            )
            rows = unique // x.size
            columns = unique % x.size
            counts[rows, columns] = group_counts
            maximum_count = int(np.max(group_counts))

            # Most elevation-map bins contain only a handful of returns. A
            # padded ragged matrix evaluates all medians/MADs in compiled
            # NumPy code and avoids one Python ``median`` call per grid node.
            if unique.size * maximum_count <= 5_000_000:
                group_number = np.repeat(np.arange(unique.size), group_counts)
                rank_in_group = np.arange(sorted_z.size) - np.repeat(
                    starts,
                    group_counts,
                )
                grouped_height = np.full(
                    (unique.size, maximum_count),
                    np.nan,
                    dtype=float,
                )
                grouped_height[group_number, rank_in_group] = sorted_z
                medians = np.nanmedian(grouped_height, axis=1)
                deviations = np.abs(grouped_height - medians[:, None])
                scaled_mad = 1.4826 * np.nanmedian(deviations, axis=1)
                accepted = group_counts >= min_points_per_node
                heights[rows[accepted], columns[accepted]] = medians[accepted]
                spread[rows[accepted], columns[accepted]] = scaled_mad[accepted]
            else:
                # Memory-safe fallback for an unusually dense single bin.
                for flat, start, count in zip(unique, starts, group_counts):
                    if count < min_points_per_node:
                        continue
                    row, column = divmod(int(flat), x.size)
                    values = sorted_z[start : start + count]
                    median = float(np.median(values))
                    heights[row, column] = median
                    spread[row, column] = 1.4826 * float(
                        np.median(np.abs(values - median))
                    )

        valid = counts >= min_points_per_node
        return cls(
            x_coordinates_m=x,
            y_coordinates_m=y,
            heights_m=heights,
            valid_mask=valid,
            height_std_m=spread,
            points_per_node=counts,
            timestamp_s=timestamp_s,
            frame_id=frame_id,
        )

    @property
    def valid_fraction(self) -> float:
        return float(np.mean(self.valid_mask))

    @property
    def bounds_m(self) -> tuple[float, float, float, float]:
        return (
            float(self.x_coordinates_m[0]),
            float(self.x_coordinates_m[-1]),
            float(self.y_coordinates_m[0]),
            float(self.y_coordinates_m[-1]),
        )

    def sample(self, x_m: ArrayLike, y_m: ArrayLike) -> ElevationSample:
        """Sample height, uncertainty, and validity without fabricating values."""

        x_query, y_query = np.broadcast_arrays(
            np.asarray(x_m, dtype=float),
            np.asarray(y_m, dtype=float),
        )
        original_shape = x_query.shape
        xq = x_query.ravel()
        yq = y_query.ravel()
        finite = np.isfinite(xq) & np.isfinite(yq)
        inside = (
            finite
            & (xq >= self.x_coordinates_m[0])
            & (xq <= self.x_coordinates_m[-1])
            & (yq >= self.y_coordinates_m[0])
            & (yq <= self.y_coordinates_m[-1])
        )

        # Indices are clipped only to make array access safe. The separate
        # ``inside`` flag keeps out-of-window samples invalid.
        safe_x = np.clip(xq, self.x_coordinates_m[0], self.x_coordinates_m[-1])
        safe_y = np.clip(yq, self.y_coordinates_m[0], self.y_coordinates_m[-1])
        ix = np.searchsorted(self.x_coordinates_m, safe_x, side="right") - 1
        iy = np.searchsorted(self.y_coordinates_m, safe_y, side="right") - 1
        ix = np.clip(ix, 0, self.x_coordinates_m.size - 2)
        iy = np.clip(iy, 0, self.y_coordinates_m.size - 2)

        x0 = self.x_coordinates_m[ix]
        x1 = self.x_coordinates_m[ix + 1]
        y0 = self.y_coordinates_m[iy]
        y1 = self.y_coordinates_m[iy + 1]
        tx = (safe_x - x0) / (x1 - x0)
        ty = (safe_y - y0) / (y1 - y0)

        node_valid = (
            self.valid_mask[iy, ix]
            & self.valid_mask[iy, ix + 1]
            & self.valid_mask[iy + 1, ix]
            & self.valid_mask[iy + 1, ix + 1]
        )
        valid = inside & node_valid
        height = _bilinear(
            self.heights_m,
            ix,
            iy,
            tx,
            ty,
        )
        standard_deviation = _bilinear(
            self.height_std_m,
            ix,
            iy,
            tx,
            ty,
        )
        gradient_x = (
            (1.0 - ty)
            * (self.heights_m[iy, ix + 1] - self.heights_m[iy, ix])
            + ty
            * (
                self.heights_m[iy + 1, ix + 1]
                - self.heights_m[iy + 1, ix]
            )
        ) / (x1 - x0)
        gradient_y = (
            (1.0 - tx)
            * (self.heights_m[iy + 1, ix] - self.heights_m[iy, ix])
            + tx
            * (
                self.heights_m[iy + 1, ix + 1]
                - self.heights_m[iy, ix + 1]
            )
        ) / (y1 - y0)
        height = np.where(valid, height, np.nan).reshape(original_shape)
        standard_deviation = np.where(
            valid,
            standard_deviation,
            np.nan,
        ).reshape(original_shape)
        gradient_x = np.where(valid, gradient_x, np.nan).reshape(original_shape)
        gradient_y = np.where(valid, gradient_y, np.nan).reshape(original_shape)
        return ElevationSample(
            height_m=height,
            height_std_m=standard_deviation,
            gradient_x=gradient_x,
            gradient_y=gradient_y,
            valid=valid.reshape(original_shape),
        )

    def valid_at(self, x_m: ArrayLike, y_m: ArrayLike) -> BoolArray:
        return self.sample(x_m, y_m).valid

    def height(self, x_m: ArrayLike, y_m: ArrayLike) -> FloatArray:
        sample = self.sample(x_m, y_m)
        if not np.all(sample.valid):
            invalid_count = int(sample.valid.size - np.count_nonzero(sample.valid))
            raise TerrainDataUnavailable(
                f"{invalid_count} of {sample.valid.size} requested terrain "
                "samples are unknown or outside the rolling window"
            )
        return sample.height_m

    def gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray]:
        sample = self.sample(x_m, y_m)
        if not np.all(sample.valid):
            invalid_count = int(sample.valid.size - np.count_nonzero(sample.valid))
            raise TerrainDataUnavailable(
                f"{invalid_count} of {sample.valid.size} requested terrain "
                "gradient samples are unknown or outside the rolling window"
            )
        return sample.gradient_x, sample.gradient_y


def _node_bin_edges(coordinates: FloatArray) -> FloatArray:
    midpoints = 0.5 * (coordinates[:-1] + coordinates[1:])
    first = coordinates[0] - 0.5 * (coordinates[1] - coordinates[0])
    last = coordinates[-1] + 0.5 * (coordinates[-1] - coordinates[-2])
    return np.concatenate(([first], midpoints, [last]))


def _bilinear(
    values: FloatArray,
    ix: IntArray,
    iy: IntArray,
    tx: FloatArray,
    ty: FloatArray,
) -> FloatArray:
    return (
        (1.0 - tx) * (1.0 - ty) * values[iy, ix]
        + tx * (1.0 - ty) * values[iy, ix + 1]
        + (1.0 - tx) * ty * values[iy + 1, ix]
        + tx * ty * values[iy + 1, ix + 1]
    )
