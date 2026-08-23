from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


class Terrain(Protocol):
    def height(self, x_m: ArrayLike, y_m: ArrayLike) -> FloatArray:
        """Return terrain height for broadcast-compatible x and y inputs."""

    def gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray]:
        """Return partial derivatives dh/dx and dh/dy."""


@dataclass(frozen=True)
class FlatTerrain:
    height_m: float = 0.0

    def height(self, x_m: ArrayLike, y_m: ArrayLike) -> FloatArray:
        x, y = np.broadcast_arrays(
            np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
        )
        return np.full_like(x + y, self.height_m, dtype=float)

    def gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray]:
        x, y = np.broadcast_arrays(
            np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
        )
        zeros = np.zeros_like(x + y, dtype=float)
        return zeros, zeros.copy()


@dataclass(frozen=True)
class SineTerrain:
    """Smooth analytic terrain for initial simulation and unit tests."""

    longitudinal_amplitude_m: float = 0.025
    longitudinal_wavelength_m: float = 0.50
    lateral_amplitude_m: float = 0.015
    lateral_wavelength_m: float = 0.70
    phase_rad: float = 0.0
    slope_x: float = 0.0
    slope_y: float = 0.0
    offset_m: float = 0.0

    def height(self, x_m: ArrayLike, y_m: ArrayLike) -> FloatArray:
        x, y = np.broadcast_arrays(
            np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
        )
        kx = 2.0 * np.pi / self.longitudinal_wavelength_m
        ky = 2.0 * np.pi / self.lateral_wavelength_m
        return (
            self.offset_m
            + self.slope_x * x
            + self.slope_y * y
            + self.longitudinal_amplitude_m * np.sin(kx * x + self.phase_rad)
            + self.lateral_amplitude_m * np.sin(ky * y - self.phase_rad)
        )

    def gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray]:
        x, y = np.broadcast_arrays(
            np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
        )
        kx = 2.0 * np.pi / self.longitudinal_wavelength_m
        ky = 2.0 * np.pi / self.lateral_wavelength_m
        return (
            self.slope_x
            + self.longitudinal_amplitude_m
            * kx
            * np.cos(kx * x + self.phase_rad),
            self.slope_y
            + self.lateral_amplitude_m
            * ky
            * np.cos(ky * y - self.phase_rad),
        )


@dataclass(frozen=True)
class GridTerrain:
    """Bilinear 2.5D height map.

    Points outside the map are clamped to the nearest edge. A production map
    adapter should instead carry an explicit unknown-space confidence mask.
    """

    x_coordinates_m: FloatArray
    y_coordinates_m: FloatArray
    heights_m: FloatArray

    def __post_init__(self) -> None:
        x = np.asarray(self.x_coordinates_m, dtype=float)
        y = np.asarray(self.y_coordinates_m, dtype=float)
        z = np.asarray(self.heights_m, dtype=float)
        if x.ndim != 1 or y.ndim != 1:
            raise ValueError("Grid coordinates must be one-dimensional")
        if z.shape != (y.size, x.size):
            raise ValueError("heights_m shape must be (len(y), len(x))")
        if np.any(np.diff(x) <= 0.0) or np.any(np.diff(y) <= 0.0):
            raise ValueError("Grid coordinates must be strictly increasing")
        object.__setattr__(self, "x_coordinates_m", x)
        object.__setattr__(self, "y_coordinates_m", y)
        object.__setattr__(self, "heights_m", z)

    def height(self, x_m: ArrayLike, y_m: ArrayLike) -> FloatArray:
        result, _, _ = self._height_and_gradient(x_m, y_m)
        return result

    def gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray]:
        _, gradient_x, gradient_y = self._height_and_gradient(x_m, y_m)
        return gradient_x, gradient_y

    def _height_and_gradient(
        self,
        x_m: ArrayLike,
        y_m: ArrayLike,
    ) -> tuple[FloatArray, FloatArray, FloatArray]:
        x_query, y_query = np.broadcast_arrays(
            np.asarray(x_m, dtype=float), np.asarray(y_m, dtype=float)
        )
        original_shape = x_query.shape
        x_inside = (
            (x_query.ravel() >= self.x_coordinates_m[0])
            & (x_query.ravel() <= self.x_coordinates_m[-1])
        )
        y_inside = (
            (y_query.ravel() >= self.y_coordinates_m[0])
            & (y_query.ravel() <= self.y_coordinates_m[-1])
        )
        xq = np.clip(x_query.ravel(), self.x_coordinates_m[0], self.x_coordinates_m[-1])
        yq = np.clip(y_query.ravel(), self.y_coordinates_m[0], self.y_coordinates_m[-1])

        ix = np.searchsorted(self.x_coordinates_m, xq, side="right") - 1
        iy = np.searchsorted(self.y_coordinates_m, yq, side="right") - 1
        ix = np.clip(ix, 0, len(self.x_coordinates_m) - 2)
        iy = np.clip(iy, 0, len(self.y_coordinates_m) - 2)

        x0, x1 = self.x_coordinates_m[ix], self.x_coordinates_m[ix + 1]
        y0, y1 = self.y_coordinates_m[iy], self.y_coordinates_m[iy + 1]
        tx = (xq - x0) / (x1 - x0)
        ty = (yq - y0) / (y1 - y0)

        z00 = self.heights_m[iy, ix]
        z10 = self.heights_m[iy, ix + 1]
        z01 = self.heights_m[iy + 1, ix]
        z11 = self.heights_m[iy + 1, ix + 1]
        result = (
            (1.0 - tx) * (1.0 - ty) * z00
            + tx * (1.0 - ty) * z10
            + (1.0 - tx) * ty * z01
            + tx * ty * z11
        )
        gradient_x = (
            (1.0 - ty) * (z10 - z00) + ty * (z11 - z01)
        ) / (x1 - x0)
        gradient_y = (
            (1.0 - tx) * (z01 - z00) + tx * (z11 - z10)
        ) / (y1 - y0)
        gradient_x = np.where(x_inside, gradient_x, 0.0)
        gradient_y = np.where(y_inside, gradient_y, 0.0)
        return (
            result.reshape(original_shape),
            gradient_x.reshape(original_shape),
            gradient_y.reshape(original_shape),
        )
