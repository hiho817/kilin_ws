from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import ArrayLike, NDArray


FloatArray = NDArray[np.float64]


def rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> FloatArray:
    """Return R_world_body = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""

    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


@dataclass(frozen=True)
class KilinGeometry:
    """Verified URDF geometry plus a guessed rectangular body envelope."""

    corner_names: tuple[str, ...] = ("FL", "FR", "RL", "RR")
    hip_origins_body_m: FloatArray = field(
        default_factory=lambda: np.array(
            [
                [0.24448, 0.13106, -0.026929],
                [0.24448, -0.12894, -0.026929],
                [-0.23552, 0.13106, -0.026929],
                [-0.23552, -0.12894, -0.026929],
            ],
            dtype=float,
        )
    )
    side_sign: FloatArray = field(
        default_factory=lambda: np.array([1.0, -1.0, 1.0, -1.0])
    )
    steering_lateral_offset_m: float = 0.118
    hip_to_wheel_axis_m: float = 0.260
    wheel_radius_m: float = 0.0585

    # User dimensions plus one explicit width guess.
    body_x_min_m: float = -0.405723
    body_x_max_m: float = 0.414677
    body_half_width_m: float = 0.190077
    # Lower body surface is 115 mm below the verified -0.026929 m hip plane.
    body_bottom_z_m: float = -0.141929
    body_top_z_m: float = 0.267464

    # SRBD placeholders. They do not affect the Version 1 kinematic optimizer.
    placeholder_total_mass_kg: float = 33.8410641

    def __post_init__(self) -> None:
        if self.hip_origins_body_m.shape != (4, 3):
            raise ValueError("hip_origins_body_m must have shape (4, 3)")
        if self.side_sign.shape != (4,):
            raise ValueError("side_sign must have shape (4,)")

    def wheel_axes_body(self, hip_angles_rad: ArrayLike) -> FloatArray:
        """Wheel-axis coordinates in `base_link` for FL, FR, RL, RR.

        Suspension displacement is fixed at zero. Steering does not translate
        the wheel axis in the supplied URDF.
        """

        q = np.asarray(hip_angles_rad, dtype=float)
        if q.shape != (4,):
            raise ValueError("hip_angles_rad must have shape (4,)")

        axes = self.hip_origins_body_m.copy()
        axes[:, 0] -= self.hip_to_wheel_axis_m * np.sin(q)
        axes[:, 1] += self.side_sign * self.steering_lateral_offset_m
        axes[:, 2] -= self.hip_to_wheel_axis_m * np.cos(q)
        return axes

    def wheel_axes_world(
        self,
        base_position_world_m: ArrayLike,
        body_rpy_rad: ArrayLike,
        hip_angles_rad: ArrayLike,
    ) -> FloatArray:
        position = np.asarray(base_position_world_m, dtype=float)
        rpy = np.asarray(body_rpy_rad, dtype=float)
        if position.shape != (3,) or rpy.shape != (3,):
            raise ValueError("base position and body RPY must have shape (3,)")

        rotation = rotation_matrix_rpy(*rpy)
        wheel_body = self.wheel_axes_body(hip_angles_rad)
        return position + (rotation @ wheel_body.T).T

    def steering_axes_body(self, hip_angles_rad: ArrayLike) -> FloatArray:
        """Physical steering-axis directions in `base_link`."""

        q = np.asarray(hip_angles_rad, dtype=float)
        if q.shape != (4,):
            raise ValueError("hip_angles_rad must have shape (4,)")
        return np.column_stack((np.sin(q), np.zeros(4), np.cos(q)))

    def body_bottom_corners_world(
        self,
        base_position_world_m: ArrayLike,
        body_rpy_rad: ArrayLike,
    ) -> FloatArray:
        position = np.asarray(base_position_world_m, dtype=float)
        rpy = np.asarray(body_rpy_rad, dtype=float)
        rotation = rotation_matrix_rpy(*rpy)

        corners_body = self.body_bottom_corners_body()
        return position + (rotation @ corners_body.T).T

    def body_bottom_corners_body(self) -> FloatArray:
        """Return the conservative rectangular body envelope in ``base_link``."""

        return np.array(
            [
                [self.body_x_max_m, self.body_half_width_m, self.body_bottom_z_m],
                [self.body_x_max_m, -self.body_half_width_m, self.body_bottom_z_m],
                [self.body_x_min_m, self.body_half_width_m, self.body_bottom_z_m],
                [self.body_x_min_m, -self.body_half_width_m, self.body_bottom_z_m],
            ],
            dtype=float,
        )

    def body_bottom_footprint_body(
        self,
        x_samples: int = 5,
        y_samples: int = 3,
    ) -> FloatArray:
        """Return targeted lower-body clearance samples, including corners."""

        if x_samples < 2 or y_samples < 2:
            raise ValueError("x_samples and y_samples must both be at least 2")
        x_m, y_m = np.meshgrid(
            np.linspace(self.body_x_min_m, self.body_x_max_m, x_samples),
            np.linspace(-self.body_half_width_m, self.body_half_width_m, y_samples),
        )
        return np.column_stack(
            (
                x_m.ravel(),
                y_m.ravel(),
                np.full(x_m.size, self.body_bottom_z_m),
            )
        )

    def nominal_base_height_above_flat_ground(
        self, nominal_hip_rad: ArrayLike
    ) -> float:
        axes = self.wheel_axes_body(nominal_hip_rad)
        return float(self.wheel_radius_m - np.mean(axes[:, 2]))
