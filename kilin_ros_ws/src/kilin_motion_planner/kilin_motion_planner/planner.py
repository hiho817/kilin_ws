from __future__ import annotations

from dataclasses import dataclass
from time import perf_counter
from typing import TYPE_CHECKING

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .config import PlannerConfig
from .model import KilinGeometry
from .path import PathHorizon

if TYPE_CHECKING:
    from .terrain import Terrain


FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PlannerDiagnostics:
    setup_time_s: float
    evaluation_counts: dict[str, int]
    evaluation_time_s: dict[str, float]
    solver_unattributed_time_s: float


@dataclass(frozen=True)
class PlanResult:
    solver_success: bool
    feasible: bool
    message: str
    objective: float
    iterations: int
    solve_time_s: float

    time_s: FloatArray
    path_x_m: FloatArray
    path_y_m: FloatArray
    path_yaw_rad: FloatArray
    base_z_m: FloatArray
    body_roll_rad: FloatArray
    body_pitch_rad: FloatArray
    hip_angles_rad: FloatArray

    wheel_axes_world_m: FloatArray
    contact_residuals_m: FloatArray
    body_footprint_clearances_m: FloatArray
    max_hip_speed_rad_s: float
    max_hip_acceleration_rad_s2: float

    fallback_hip_command_rad: FloatArray
    diagnostics: PlannerDiagnostics

    @property
    def first_hip_command_rad(self) -> FloatArray:
        if self.feasible:
            return self.hip_angles_rad[0].copy()
        return self.fallback_hip_command_rad.copy()

    @property
    def recommended_speed_scale(self) -> float:
        return 1.0 if self.feasible else 0.0

    @property
    def max_contact_error_m(self) -> float:
        return float(np.max(np.abs(self.contact_residuals_m)))

    @property
    def minimum_body_clearance_m(self) -> float:
        return float(np.min(self.body_footprint_clearances_m))

    @property
    def body_corner_clearances_m(self) -> FloatArray:
        """Compatibility alias; clearance is now sampled across the footprint."""

        return self.body_footprint_clearances_m


@dataclass(frozen=True)
class _GeometricEvaluation:
    contact_residuals_m: FloatArray
    contact_jacobian: FloatArray
    body_footprint_clearances_m: FloatArray
    body_footprint_clearance_jacobian: FloatArray


class RecedingHorizonPlanner:
    """Contact-constrained quasi-static terrain-preview planner.

    Decision variables at each horizon sample are:
        [base_z, roll, pitch, FL_hip, FR_hip, RL_hip, RR_hip]
    """

    state_width = 7

    def __init__(
        self,
        *,
        geometry: KilinGeometry | None = None,
        config: PlannerConfig | None = None,
    ) -> None:
        self.geometry = geometry or KilinGeometry()
        self.config = config or PlannerConfig()
        self._last_solution: FloatArray | None = None

    def warm_start_snapshot(self) -> FloatArray | None:
        """Copy the solution currently committed for the next horizon."""

        return (
            None
            if self._last_solution is None
            else self._last_solution.copy()
        )

    def restore_warm_start(self, snapshot: ArrayLike | None) -> None:
        """Restore a snapshot when a computed plan was not actually executed."""

        if snapshot is None:
            self._last_solution = None
            return
        states = np.asarray(snapshot, dtype=float)
        expected_shape = (self.config.horizon_steps, self.state_width)
        if states.shape != expected_shape or not np.isfinite(states).all():
            raise ValueError(
                f"warm-start snapshot must have finite shape {expected_shape}"
            )
        self._last_solution = states.copy()

    def prepare(self) -> None:
        """Load the numerical solver before the live control deadline begins."""

        try:
            from scipy.optimize import minimize as _  # noqa: F401
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "SciPy with a compatible NumPy is required. On Ubuntu/ROS, install "
                "python3-numpy and python3-scipy; remove or downgrade any user-site "
                "NumPy that shadows the system version."
            ) from exc

    def _unpack(self, decision: ArrayLike, steps: int) -> tuple[FloatArray, ...]:
        states = np.asarray(decision, dtype=float).reshape(steps, self.state_width)
        return states[:, 0], states[:, 1], states[:, 2], states[:, 3:7]

    def _wheel_axes_world(
        self,
        path: PathHorizon,
        base_z: FloatArray,
        roll: FloatArray,
        pitch: FloatArray,
        hips: FloatArray,
    ) -> FloatArray:
        wheel_body = self._wheel_axes_body_batch(hips)
        rotation = _rotation_matrices_rpy(roll, pitch, path.yaw_rad)
        position = np.column_stack((path.x_m, path.y_m, base_z))
        return position[:, None, :] + np.einsum(
            "kij,klj->kli",
            rotation,
            wheel_body,
        )

    def _wheel_axes_body_batch(self, hips: FloatArray) -> FloatArray:
        if hips.ndim != 2 or hips.shape[1] != 4:
            raise ValueError("hips must have shape (steps, 4)")
        axes = np.broadcast_to(
            self.geometry.hip_origins_body_m,
            (hips.shape[0], 4, 3),
        ).copy()
        axes[:, :, 0] -= self.geometry.hip_to_wheel_axis_m * np.sin(hips)
        axes[:, :, 1] += (
            self.geometry.side_sign * self.geometry.steering_lateral_offset_m
        )
        axes[:, :, 2] -= self.geometry.hip_to_wheel_axis_m * np.cos(hips)
        return axes

    def _contact_residuals(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        return self._geometric_evaluation(
            decision, path, terrain
        ).contact_residuals_m

    def _contact_jacobian(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        return self._geometric_evaluation(
            decision, path, terrain
        ).contact_jacobian

    def _geometric_evaluation(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> _GeometricEvaluation:
        """Evaluate shared wheel and lower-body footprint geometry once."""
        base_z, roll, pitch, hips = self._unpack(decision, path.steps)
        wheel_body = self._wheel_axes_body_batch(hips)
        rotation, derivative_roll, derivative_pitch = (
            _rotation_matrices_with_derivatives(roll, pitch, path.yaw_rad)
        )
        position = np.column_stack((path.x_m, path.y_m, base_z))
        wheels = position[:, None, :] + np.einsum(
            "kij,klj->kli",
            rotation,
            wheel_body,
        )
        footprint_body = self.geometry.body_bottom_footprint_body()
        footprint = position[:, None, :] + np.einsum(
            "kij,lj->kli",
            rotation,
            footprint_body,
        )
        sample_x = np.concatenate((wheels[:, :, 0], footprint[:, :, 0]), axis=1)
        sample_y = np.concatenate((wheels[:, :, 1], footprint[:, :, 1]), axis=1)
        ground = np.asarray(terrain.height(sample_x, sample_y), dtype=float)
        gradient_x, gradient_y = _terrain_gradient(
            terrain,
            sample_x,
            sample_y,
        )
        surface_gradient_world = np.stack(
            (-gradient_x, -gradient_y, np.ones_like(gradient_x)),
            axis=2,
        )
        contact = (
            wheels[:, :, 2]
            - ground[:, :4]
            - self.geometry.wheel_radius_m
        )
        clearance = footprint[:, :, 2] - ground[:, 4:]
        contact_gradient_world = surface_gradient_world[:, :4, :]
        clearance_gradient_world = surface_gradient_world[:, 4:, :]

        wheel_derivative_roll = np.einsum(
            "kij,klj->kli",
            derivative_roll,
            wheel_body,
        )
        wheel_derivative_pitch = np.einsum(
            "kij,klj->kli",
            derivative_pitch,
            wheel_body,
        )
        derivative_q_body = np.zeros_like(wheel_body)
        derivative_q_body[:, :, 0] = (
            -self.geometry.hip_to_wheel_axis_m * np.cos(hips)
        )
        derivative_q_body[:, :, 2] = (
            self.geometry.hip_to_wheel_axis_m * np.sin(hips)
        )
        wheel_derivative_q = np.einsum(
            "kij,klj->kli",
            rotation,
            derivative_q_body,
        )

        derivative_roll_value = np.einsum(
            "kli,kli->kl",
            contact_gradient_world,
            wheel_derivative_roll,
        )
        derivative_pitch_value = np.einsum(
            "kli,kli->kl",
            contact_gradient_world,
            wheel_derivative_pitch,
        )
        derivative_q_value = np.einsum(
            "kli,kli->kl",
            contact_gradient_world,
            wheel_derivative_q,
        )

        contact_jacobian = np.zeros(
            (path.steps * 4, path.steps * self.state_width),
            dtype=float,
        )
        knot_indices = np.arange(path.steps)
        knot_columns = knot_indices * self.state_width
        for wheel_index in range(4):
            rows = knot_indices * 4 + wheel_index
            contact_jacobian[rows, knot_columns] = 1.0
            contact_jacobian[rows, knot_columns + 1] = derivative_roll_value[:, wheel_index]
            contact_jacobian[rows, knot_columns + 2] = derivative_pitch_value[:, wheel_index]
            contact_jacobian[rows, knot_columns + 3 + wheel_index] = (
                derivative_q_value[:, wheel_index]
            )

        footprint_derivative_roll = np.einsum(
            "kij,lj->kli",
            derivative_roll,
            footprint_body,
        )
        footprint_derivative_pitch = np.einsum(
            "kij,lj->kli",
            derivative_pitch,
            footprint_body,
        )
        footprint_roll_value = np.einsum(
            "kli,kli->kl",
            clearance_gradient_world,
            footprint_derivative_roll,
        )
        footprint_pitch_value = np.einsum(
            "kli,kli->kl",
            clearance_gradient_world,
            footprint_derivative_pitch,
        )
        footprint_count = footprint_body.shape[0]
        clearance_jacobian = np.zeros(
            (path.steps * footprint_count, path.steps * self.state_width),
            dtype=float,
        )
        for footprint_index in range(footprint_count):
            rows = knot_indices * footprint_count + footprint_index
            clearance_jacobian[rows, knot_columns] = 1.0
            clearance_jacobian[rows, knot_columns + 1] = (
                footprint_roll_value[:, footprint_index]
            )
            clearance_jacobian[rows, knot_columns + 2] = (
                footprint_pitch_value[:, footprint_index]
            )
        return _GeometricEvaluation(
            contact_residuals_m=contact,
            contact_jacobian=contact_jacobian,
            body_footprint_clearances_m=clearance,
            body_footprint_clearance_jacobian=clearance_jacobian,
        )

    def _body_footprint_clearances(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        return self._geometric_evaluation(
            decision, path, terrain
        ).body_footprint_clearances_m

    def _body_footprint_clearance_jacobian(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        return self._geometric_evaluation(
            decision, path, terrain
        ).body_footprint_clearance_jacobian

    def _height_reference(
        self,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        nominal = self.config.nominal_hip_rad
        wheel_body = self.geometry.wheel_axes_body(nominal)
        rotation = _rotation_matrices_rpy(
            np.zeros(path.steps),
            np.zeros(path.steps),
            path.yaw_rad,
        )
        position = np.column_stack((path.x_m, path.y_m, np.zeros(path.steps)))
        wheel_world = position[:, None, :] + np.einsum(
            "kij,lj->kli",
            rotation,
            wheel_body,
        )
        ground = terrain.height(wheel_world[:, :, 0], wheel_world[:, :, 1])
        return np.mean(
            ground + self.geometry.wheel_radius_m - wheel_body[None, :, 2],
            axis=1,
        )

    def _fresh_initial_guess(
        self,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        height_reference = self._height_reference(path, terrain)
        states = np.zeros((path.steps, self.state_width), dtype=float)
        states[:, 0] = height_reference
        states[:, 3:7] = self.config.nominal_hip_rad

        # Improve contact feasibility while preserving a level-body seed. The
        # ground sample is iterated because hip angle also changes wheel x.
        hip_origin_z = self.geometry.hip_origins_body_m[:, 2]
        length = self.geometry.hip_to_wheel_axis_m
        signs = np.sign(self.config.nominal_hip_rad)

        q = np.broadcast_to(
            self.config.nominal_hip_rad,
            (path.steps, 4),
        ).copy()
        rotation = _rotation_matrices_rpy(
            np.zeros(path.steps),
            np.zeros(path.steps),
            path.yaw_rad,
        )
        position = np.column_stack((path.x_m, path.y_m, height_reference))
        for _ in range(6):
            wheel_body = self._wheel_axes_body_batch(q)
            wheel_world = position[:, None, :] + np.einsum(
                "kij,klj->kli",
                rotation,
                wheel_body,
            )
            ground = terrain.height(wheel_world[:, :, 0], wheel_world[:, :, 1])
            cosine = (
                height_reference[:, None]
                + hip_origin_z[None, :]
                - ground
                - self.geometry.wheel_radius_m
            ) / length
            magnitude = np.arccos(np.clip(cosine, -1.0, 1.0))
            q = np.clip(
                signs[None, :] * magnitude,
                self.config.hip_lower_rad[None, :],
                self.config.hip_upper_rad[None, :],
            )
        states[:, 3:7] = q
        return states

    def _initial_guess(
        self,
        path: PathHorizon,
        terrain: "Terrain",
    ) -> FloatArray:
        fresh = self._fresh_initial_guess(path, terrain)
        if self._last_solution is None or self._last_solution.shape != fresh.shape:
            return fresh

        shifted = np.vstack((self._last_solution[1:], self._last_solution[-1]))
        # Preserve warm-start posture but move the height with the new terrain.
        shifted[:, 0] += fresh[:, 0] - np.mean(shifted[:, 0])
        return 0.75 * shifted + 0.25 * fresh

    def _objective(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        height_reference: FloatArray,
        previous_hip_rad: FloatArray | None,
    ) -> float:
        base_z, roll, pitch, hips = self._unpack(decision, path.steps)
        weights = self.config.weights
        dt = self.config.dt_s

        cost = (
            weights.roll * np.sum(roll**2)
            + weights.pitch * np.sum(pitch**2)
            + weights.body_height * np.sum((base_z - height_reference) ** 2)
            + weights.hip_nominal
            * np.sum((hips - self.config.nominal_hip_rad) ** 2)
        )

        hip_differences = np.diff(hips, axis=0)
        if previous_hip_rad is not None:
            hip_differences = np.vstack((hips[0] - previous_hip_rad, hip_differences))
        if hip_differences.size:
            cost += weights.hip_velocity * np.sum((hip_differences / dt) ** 2)

        if path.steps >= 3:
            hip_acceleration = np.diff(hips, n=2, axis=0) / (dt**2)
            cost += weights.hip_acceleration * np.sum(hip_acceleration**2)

        attitude = np.column_stack((roll, pitch))
        if path.steps >= 2:
            attitude_rate = np.diff(attitude, axis=0) / dt
            cost += weights.attitude_rate * np.sum(attitude_rate**2)
        return float(cost)

    def _objective_gradient(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        height_reference: FloatArray,
        previous_hip_rad: FloatArray | None,
    ) -> FloatArray:
        base_z, roll, pitch, hips = self._unpack(decision, path.steps)
        weights = self.config.weights
        dt = self.config.dt_s
        gradient = np.zeros((path.steps, self.state_width), dtype=float)
        gradient[:, 0] = 2.0 * weights.body_height * (
            base_z - height_reference
        )
        gradient[:, 1] = 2.0 * weights.roll * roll
        gradient[:, 2] = 2.0 * weights.pitch * pitch
        gradient[:, 3:7] = 2.0 * weights.hip_nominal * (
            hips - self.config.nominal_hip_rad
        )

        hip_difference = _first_difference_matrix(
            path.steps,
            include_initial=previous_hip_rad is not None,
        )
        if hip_difference.shape[0]:
            differences = hip_difference @ hips
            if previous_hip_rad is not None:
                differences[0] -= previous_hip_rad
            gradient[:, 3:7] += (
                2.0
                * weights.hip_velocity
                / (dt**2)
                * (hip_difference.T @ differences)
            )

        if path.steps >= 3:
            second_difference = _second_difference_matrix(path.steps)
            gradient[:, 3:7] += (
                2.0
                * weights.hip_acceleration
                / (dt**4)
                * (
                    second_difference.T
                    @ (second_difference @ hips)
                )
            )

        if path.steps >= 2:
            attitude_difference = _first_difference_matrix(
                path.steps,
                include_initial=False,
            )
            attitude = np.column_stack((roll, pitch))
            attitude_gradient = (
                2.0
                * weights.attitude_rate
                / (dt**2)
                * (
                    attitude_difference.T
                    @ (attitude_difference @ attitude)
                )
            )
            gradient[:, 1:3] += attitude_gradient
        return gradient.ravel()

    def _rate_margins(
        self,
        decision: ArrayLike,
        path: PathHorizon,
        previous_hip_rad: FloatArray | None,
    ) -> FloatArray:
        _, _, _, hips = self._unpack(decision, path.steps)
        differences = np.diff(hips, axis=0)
        if previous_hip_rad is not None:
            differences = np.vstack((hips[0] - previous_hip_rad, differences))
        rates = differences / self.config.dt_s
        limit = self.config.hip_speed_limit_rad_s
        return np.concatenate(((limit - rates).ravel(), (limit + rates).ravel()))

    def _rate_margin_jacobian(
        self,
        path: PathHorizon,
        previous_hip_rad: FloatArray | None,
    ) -> FloatArray:
        difference = _first_difference_matrix(
            path.steps,
            include_initial=previous_hip_rad is not None,
        ) / self.config.dt_s
        rate_jacobian = np.zeros(
            (difference.shape[0] * 4, path.steps * self.state_width),
            dtype=float,
        )
        for difference_row in range(difference.shape[0]):
            for wheel_index in range(4):
                row = difference_row * 4 + wheel_index
                for knot in np.flatnonzero(difference[difference_row]):
                    rate_jacobian[row, knot * self.state_width + 3 + wheel_index] = (
                        difference[difference_row, knot]
                    )
        return np.vstack((-rate_jacobian, rate_jacobian))

    def _acceleration_margins(
        self,
        decision: ArrayLike,
        path: PathHorizon,
    ) -> FloatArray:
        _, _, _, hips = self._unpack(decision, path.steps)
        if path.steps < 3:
            return np.array([1.0])
        acceleration = np.diff(hips, n=2, axis=0) / (self.config.dt_s**2)
        limit = self.config.hip_acceleration_limit_rad_s2
        return np.concatenate(
            ((limit - acceleration).ravel(), (limit + acceleration).ravel())
        )

    def _acceleration_margin_jacobian(self, path: PathHorizon) -> FloatArray:
        if path.steps < 3:
            return np.zeros((1, path.steps * self.state_width), dtype=float)
        difference = _second_difference_matrix(path.steps) / (self.config.dt_s**2)
        acceleration_jacobian = np.zeros(
            (difference.shape[0] * 4, path.steps * self.state_width),
            dtype=float,
        )
        for difference_row in range(difference.shape[0]):
            for wheel_index in range(4):
                row = difference_row * 4 + wheel_index
                for knot in np.flatnonzero(difference[difference_row]):
                    acceleration_jacobian[
                        row,
                        knot * self.state_width + 3 + wheel_index,
                    ] = difference[difference_row, knot]
        return np.vstack((-acceleration_jacobian, acceleration_jacobian))

    def _bounds(
        self,
        height_reference: FloatArray,
    ) -> list[tuple[float, float]]:
        result: list[tuple[float, float]] = []
        for height in height_reference:
            result.extend(
                [
                    (
                        float(height - self.config.body_z_margin_m),
                        float(height + self.config.body_z_margin_m),
                    ),
                    (-self.config.roll_limit_rad, self.config.roll_limit_rad),
                    (-self.config.pitch_limit_rad, self.config.pitch_limit_rad),
                ]
            )
            result.extend(
                zip(
                    self.config.hip_lower_rad.tolist(),
                    self.config.hip_upper_rad.tolist(),
                )
            )
        return result

    def plan(
        self,
        *,
        path: PathHorizon,
        terrain: "Terrain",
        previous_hip_rad: ArrayLike | None = None,
    ) -> PlanResult:
        if path.steps != self.config.horizon_steps:
            raise ValueError(
                f"Path has {path.steps} samples; planner expects "
                f"{self.config.horizon_steps}"
            )

        try:
            from scipy.optimize import minimize
        except ImportError as exc:  # pragma: no cover - environment dependent
            raise RuntimeError(
                "SciPy is required. Install this package with `python -m pip "
                "install -e planning`."
            ) from exc

        plan_started = perf_counter()
        previous = (
            None if previous_hip_rad is None else np.asarray(previous_hip_rad, dtype=float)
        )
        if previous is not None and previous.shape != (4,):
            raise ValueError("previous_hip_rad must have shape (4,)")

        height_reference = self._height_reference(path, terrain)
        initial = self._initial_guess(path, terrain)
        fallback = (
            self.config.nominal_hip_rad.copy() if previous is None else previous.copy()
        )

        evaluation_counts = {
            "objective": 0,
            "objective_gradient": 0,
            "contact": 0,
            "contact_jacobian": 0,
            "clearance": 0,
            "clearance_jacobian": 0,
            "hip_rate": 0,
            "hip_rate_jacobian": 0,
            "hip_acceleration": 0,
            "hip_acceleration_jacobian": 0,
        }
        evaluation_time = {name: 0.0 for name in evaluation_counts}

        cached_decision: FloatArray | None = None
        cached_geometry: _GeometricEvaluation | None = None

        def geometry_evaluation(decision: ArrayLike) -> _GeometricEvaluation:
            nonlocal cached_decision, cached_geometry
            current = np.asarray(decision, dtype=float)
            if (
                cached_decision is None
                or cached_geometry is None
                or not np.array_equal(current, cached_decision)
            ):
                cached_decision = current.copy()
                cached_geometry = self._geometric_evaluation(
                    current, path, terrain
                )
            return cached_geometry

        rate_jacobian = self._rate_margin_jacobian(path, previous)
        acceleration_jacobian = self._acceleration_margin_jacobian(path)

        def timed_evaluation(name: str, function, decision):
            evaluation_counts[name] += 1
            started = perf_counter()
            try:
                return function(decision)
            finally:
                evaluation_time[name] += perf_counter() - started

        contact_constraint = {
            "type": "eq",
            "fun": lambda x: timed_evaluation(
                    "contact",
                    lambda value: geometry_evaluation(
                        value
                    ).contact_residuals_m.ravel(),
                    x,
                ),
        }
        clearance_constraint = {
            "type": "ineq",
            "fun": lambda x: timed_evaluation(
                    "clearance",
                    lambda value: (
                        geometry_evaluation(value).body_footprint_clearances_m
                        - self.config.minimum_body_clearance_m
                    ).ravel(),
                    x,
                ),
        }
        rate_constraint = {
            "type": "ineq",
            "fun": lambda x: timed_evaluation(
                    "hip_rate",
                    lambda value: self._rate_margins(value, path, previous),
                    x,
                ),
        }
        acceleration_constraint = {
            "type": "ineq",
            "fun": lambda x: timed_evaluation(
                    "hip_acceleration",
                    lambda value: self._acceleration_margins(value, path),
                    x,
                ),
        }
        if self.config.use_analytic_derivatives:
            contact_constraint["jac"] = lambda x: timed_evaluation(
                "contact_jacobian",
                lambda value: geometry_evaluation(value).contact_jacobian,
                x,
            )
            clearance_constraint["jac"] = lambda x: timed_evaluation(
                "clearance_jacobian",
                lambda value: geometry_evaluation(
                    value
                ).body_footprint_clearance_jacobian,
                x,
            )
            rate_constraint["jac"] = lambda x: timed_evaluation(
                "hip_rate_jacobian",
                lambda value: rate_jacobian,
                x,
            )
            acceleration_constraint["jac"] = lambda x: timed_evaluation(
                "hip_acceleration_jacobian",
                lambda value: acceleration_jacobian,
                x,
            )
        constraints = [
            contact_constraint,
            clearance_constraint,
            rate_constraint,
            acceleration_constraint,
        ]

        started = perf_counter()
        solver_result = minimize(
            fun=lambda x: timed_evaluation(
                "objective",
                lambda value: self._objective(
                    value,
                    path,
                    height_reference,
                    previous,
                ),
                x,
            ),
            jac=(
                (
                    lambda x: timed_evaluation(
                        "objective_gradient",
                        lambda value: self._objective_gradient(
                            value,
                            path,
                            height_reference,
                            previous,
                        ),
                        x,
                    )
                )
                if self.config.use_analytic_derivatives
                else None
            ),
            x0=initial.ravel(),
            method="SLSQP",
            bounds=self._bounds(height_reference),
            constraints=constraints,
            options={
                "maxiter": self.config.solver_max_iterations,
                "ftol": self.config.solver_ftol,
                "disp": False,
            },
        )
        solve_time = perf_counter() - started
        setup_time = started - plan_started

        base_z, roll, pitch, hips = self._unpack(solver_result.x, path.steps)
        wheels = self._wheel_axes_world(path, base_z, roll, pitch, hips)
        contact = self._contact_residuals(solver_result.x, path, terrain)
        clearance = self._body_footprint_clearances(solver_result.x, path, terrain)

        hip_rate = np.diff(hips, axis=0) / self.config.dt_s
        if previous is not None:
            hip_rate = np.vstack(((hips[0] - previous) / self.config.dt_s, hip_rate))
        max_rate = float(np.max(np.abs(hip_rate))) if hip_rate.size else 0.0

        hip_acceleration = (
            np.diff(hips, n=2, axis=0) / (self.config.dt_s**2)
            if path.steps >= 3
            else np.zeros((0, 4))
        )
        max_acceleration = (
            float(np.max(np.abs(hip_acceleration)))
            if hip_acceleration.size
            else 0.0
        )

        feasible = (
            np.max(np.abs(contact)) <= self.config.contact_tolerance_m
            and np.min(clearance)
            >= self.config.minimum_body_clearance_m
            - self.config.clearance_tolerance_m
            and max_rate
            <= self.config.hip_speed_limit_rad_s
            + self.config.rate_tolerance_rad_s
            and max_acceleration
            <= self.config.hip_acceleration_limit_rad_s2
            + self.config.rate_tolerance_rad_s
        )

        states = np.column_stack((base_z, roll, pitch, hips))
        if feasible:
            self._last_solution = states.copy()

        attributed_time = float(sum(evaluation_time.values()))
        return PlanResult(
            solver_success=bool(solver_result.success),
            feasible=bool(feasible),
            message=str(solver_result.message),
            objective=float(solver_result.fun),
            iterations=int(getattr(solver_result, "nit", 0)),
            solve_time_s=float(solve_time),
            time_s=path.time_s.copy(),
            path_x_m=path.x_m.copy(),
            path_y_m=path.y_m.copy(),
            path_yaw_rad=path.yaw_rad.copy(),
            base_z_m=base_z.copy(),
            body_roll_rad=roll.copy(),
            body_pitch_rad=pitch.copy(),
            hip_angles_rad=hips.copy(),
            wheel_axes_world_m=wheels,
            contact_residuals_m=contact,
            body_footprint_clearances_m=clearance,
            max_hip_speed_rad_s=max_rate,
            max_hip_acceleration_rad_s2=max_acceleration,
            fallback_hip_command_rad=fallback,
            diagnostics=PlannerDiagnostics(
                setup_time_s=float(setup_time),
                evaluation_counts=evaluation_counts.copy(),
                evaluation_time_s=evaluation_time.copy(),
                solver_unattributed_time_s=max(0.0, solve_time - attributed_time),
            ),
        )


def _rotation_matrices_rpy(
    roll: ArrayLike,
    pitch: ArrayLike,
    yaw: ArrayLike,
) -> FloatArray:
    """Vectorized ``Rz(yaw) @ Ry(pitch) @ Rx(roll)`` matrices."""

    roll_array, pitch_array, yaw_array = np.broadcast_arrays(
        np.asarray(roll, dtype=float),
        np.asarray(pitch, dtype=float),
        np.asarray(yaw, dtype=float),
    )
    cr, sr = np.cos(roll_array), np.sin(roll_array)
    cp, sp = np.cos(pitch_array), np.sin(pitch_array)
    cy, sy = np.cos(yaw_array), np.sin(yaw_array)

    rotation = np.empty(roll_array.shape + (3, 3), dtype=float)
    rotation[..., 0, 0] = cy * cp
    rotation[..., 0, 1] = cy * sp * sr - sy * cr
    rotation[..., 0, 2] = cy * sp * cr + sy * sr
    rotation[..., 1, 0] = sy * cp
    rotation[..., 1, 1] = sy * sp * sr + cy * cr
    rotation[..., 1, 2] = sy * sp * cr - cy * sr
    rotation[..., 2, 0] = -sp
    rotation[..., 2, 1] = cp * sr
    rotation[..., 2, 2] = cp * cr
    return rotation


def _rotation_matrices_with_derivatives(
    roll: ArrayLike,
    pitch: ArrayLike,
    yaw: ArrayLike,
) -> tuple[FloatArray, FloatArray, FloatArray]:
    roll_array, pitch_array, yaw_array = np.broadcast_arrays(
        np.asarray(roll, dtype=float),
        np.asarray(pitch, dtype=float),
        np.asarray(yaw, dtype=float),
    )
    rotation = _rotation_matrices_rpy(roll_array, pitch_array, yaw_array)
    derivative_roll = np.empty_like(rotation)
    derivative_pitch = np.empty_like(rotation)
    for index in np.ndindex(roll_array.shape):
        r = float(roll_array[index])
        p = float(pitch_array[index])
        y = float(yaw_array[index])
        cr, sr = np.cos(r), np.sin(r)
        cp, sp = np.cos(p), np.sin(p)
        cy, sy = np.cos(y), np.sin(y)
        rx = np.array(
            [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]],
            dtype=float,
        )
        derivative_rx = np.array(
            [[0.0, 0.0, 0.0], [0.0, -sr, -cr], [0.0, cr, -sr]],
            dtype=float,
        )
        ry = np.array(
            [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]],
            dtype=float,
        )
        derivative_ry = np.array(
            [[-sp, 0.0, cp], [0.0, 0.0, 0.0], [-cp, 0.0, -sp]],
            dtype=float,
        )
        rz = np.array(
            [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        derivative_roll[index] = rz @ ry @ derivative_rx
        derivative_pitch[index] = rz @ derivative_ry @ rx
    return rotation, derivative_roll, derivative_pitch


def _first_difference_matrix(
    steps: int,
    *,
    include_initial: bool,
) -> FloatArray:
    rows = steps if include_initial else max(steps - 1, 0)
    difference = np.zeros((rows, steps), dtype=float)
    if include_initial:
        difference[0, 0] = 1.0
        for row in range(1, steps):
            difference[row, row - 1] = -1.0
            difference[row, row] = 1.0
    else:
        for row in range(rows):
            difference[row, row] = -1.0
            difference[row, row + 1] = 1.0
    return difference


def _second_difference_matrix(steps: int) -> FloatArray:
    difference = np.zeros((max(steps - 2, 0), steps), dtype=float)
    for row in range(difference.shape[0]):
        difference[row, row] = 1.0
        difference[row, row + 1] = -2.0
        difference[row, row + 2] = 1.0
    return difference


def _terrain_gradient(
    terrain: "Terrain",
    x_m: ArrayLike,
    y_m: ArrayLike,
) -> tuple[FloatArray, FloatArray]:
    gradient_function = getattr(terrain, "gradient", None)
    if callable(gradient_function):
        gradient_x, gradient_y = gradient_function(x_m, y_m)
        return np.asarray(gradient_x, dtype=float), np.asarray(
            gradient_y,
            dtype=float,
        )

    # Compatibility fallback for a user-supplied Version 1 Terrain object.
    step_m = 1.0e-5
    x = np.asarray(x_m, dtype=float)
    y = np.asarray(y_m, dtype=float)
    gradient_x = (
        terrain.height(x + step_m, y) - terrain.height(x - step_m, y)
    ) / (2.0 * step_m)
    gradient_y = (
        terrain.height(x, y + step_m) - terrain.height(x, y - step_m)
    ) / (2.0 * step_m)
    return np.asarray(gradient_x, dtype=float), np.asarray(gradient_y, dtype=float)
