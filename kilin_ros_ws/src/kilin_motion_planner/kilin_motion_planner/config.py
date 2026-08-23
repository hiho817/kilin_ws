from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class PlannerWeights:
    """Quadratic objective weights.

    Roll and pitch dominate. The remaining terms resolve the redundant
    contact-constrained posture and produce smooth, trackable references.
    """

    roll: float = 600.0
    pitch: float = 600.0
    body_height: float = 40.0
    hip_nominal: float = 2.0
    hip_velocity: float = 0.15
    hip_acceleration: float = 0.015
    attitude_rate: float = 8.0


@dataclass(frozen=True)
class PlannerConfig:
    """Planner settings and deliberately explicit engineering guesses."""

    horizon_steps: int = 14
    dt_s: float = 0.15
    # Spatial separation of consecutive reference poses.  This deliberately
    # stays independent of the live wheel-speed command.
    horizon_knot_spacing_m: float = 0.05

    nominal_hip_rad: np.ndarray = field(
        default_factory=lambda: np.deg2rad(
            np.array([-45.0, -45.0, 45.0, 45.0], dtype=float)
        )
    )
    hip_lower_rad: np.ndarray = field(
        default_factory=lambda: np.deg2rad(
            np.array([-75.0, -75.0, 15.0, 15.0], dtype=float)
        )
    )
    hip_upper_rad: np.ndarray = field(
        default_factory=lambda: np.deg2rad(
            np.array([-15.0, -15.0, 75.0, 75.0], dtype=float)
        )
    )

    hip_speed_limit_rad_s: float = 0.8 * np.pi
    hip_acceleration_limit_rad_s2: float = 8.0
    roll_limit_rad: float = np.deg2rad(20.0)
    pitch_limit_rad: float = np.deg2rad(20.0)

    minimum_body_clearance_m: float = 0.030
    body_z_margin_m: float = 0.30

    contact_tolerance_m: float = 5.0e-4
    clearance_tolerance_m: float = 1.0e-4
    rate_tolerance_rad_s: float = 1.0e-4

    solver_max_iterations: int = 400
    solver_ftol: float = 1.0e-9
    use_analytic_derivatives: bool = False

    weights: PlannerWeights = field(default_factory=PlannerWeights)

    def __post_init__(self) -> None:
        arrays = (
            self.nominal_hip_rad,
            self.hip_lower_rad,
            self.hip_upper_rad,
        )
        if any(np.asarray(item).shape != (4,) for item in arrays):
            raise ValueError("All hip configuration arrays must have shape (4,)")
        if np.any(self.hip_lower_rad >= self.hip_upper_rad):
            raise ValueError("Each hip lower limit must be below its upper limit")
        if self.horizon_steps < 2:
            raise ValueError("horizon_steps must be at least 2")
        if self.dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if self.horizon_knot_spacing_m <= 0.0:
            raise ValueError("horizon_knot_spacing_m must be positive")
