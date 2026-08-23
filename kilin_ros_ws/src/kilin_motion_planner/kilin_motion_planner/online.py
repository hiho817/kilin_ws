from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from time import perf_counter
from typing import Protocol

import numpy as np
from numpy.typing import ArrayLike, NDArray

from .live_terrain import ElevationWindow, TerrainDataUnavailable
from .model import KilinGeometry, rotation_matrix_rpy
from .path import PathHorizon
from .planner import PlanResult, RecedingHorizonPlanner


FloatArray = NDArray[np.float64]


class LiveCycleStatus(str, Enum):
    PLANNED = "planned"
    TERRAIN_UNAVAILABLE = "terrain_unavailable"
    INFEASIBLE = "infeasible"
    DEADLINE_MISSED = "deadline_missed"


@dataclass(frozen=True)
class LivePlannerConfig:
    """Version 2 publication policy around the Version 1 optimizer."""

    planner_deadline_s: float | None = None
    require_reference_track_valid: bool = True

    def __post_init__(self) -> None:
        if self.planner_deadline_s is not None and self.planner_deadline_s <= 0.0:
            raise ValueError("planner_deadline_s must be positive or None")


@dataclass(frozen=True)
class LiveCycleResult:
    cycle_index: int
    status: LiveCycleStatus
    message: str
    planning_time_s: float
    terrain_timestamp_s: float
    hip_command_rad: FloatArray
    wheel_rolling_speed_rad_s: FloatArray
    plan: PlanResult | None

    @property
    def published_new_plan(self) -> bool:
        return self.status is LiveCycleStatus.PLANNED


class _OnlinePlanner(Protocol):
    geometry: KilinGeometry

    def plan(
        self,
        *,
        path: PathHorizon,
        terrain: ElevationWindow,
        previous_hip_rad: ArrayLike | None = None,
    ) -> PlanResult: ...


class RecedingHorizonSession:
    """Execute one command from each overlapping Version 2 planning horizon."""

    def __init__(
        self,
        *,
        planner: _OnlinePlanner | None = None,
        config: LivePlannerConfig | None = None,
        initial_hip_rad: ArrayLike | None = None,
    ) -> None:
        self.planner = planner or RecedingHorizonPlanner()
        self.config = config or LivePlannerConfig()
        nominal = self.planner.config.nominal_hip_rad  # type: ignore[attr-defined]
        initial = nominal if initial_hip_rad is None else np.asarray(initial_hip_rad, dtype=float)
        if np.asarray(initial).shape != (4,):
            raise ValueError("initial_hip_rad must have shape (4,)")
        self._executed_hip_rad = np.asarray(initial, dtype=float).copy()
        self._cycle_index = 0
        self._last_valid_plan: PlanResult | None = None

    @property
    def executed_hip_rad(self) -> FloatArray:
        return self._executed_hip_rad.copy()

    @property
    def last_valid_plan(self) -> PlanResult | None:
        return self._last_valid_plan

    def set_measured_hip(self, measured_hip_rad: ArrayLike) -> None:
        measured = np.asarray(measured_hip_rad, dtype=float)
        if measured.shape != (4,) or not np.isfinite(measured).all():
            raise ValueError("measured_hip_rad must be a finite shape-(4,) vector")
        self._executed_hip_rad = measured.copy()

    def plan_cycle(
        self,
        *,
        path: PathHorizon,
        terrain: ElevationWindow,
        measured_hip_rad: ArrayLike | None = None,
        enforce_deadline: bool = True,
    ) -> LiveCycleResult:
        """Plan once, publish only knot zero, and retain a safe hold fallback.

        ``enforce_deadline=False`` is intended only for the stationary startup
        prime, before the timed locomotion loop begins.
        """

        if measured_hip_rad is not None:
            self.set_measured_hip(measured_hip_rad)
        hold_hip = self._executed_hip_rad.copy()
        cycle_index = self._cycle_index
        self._cycle_index += 1

        if self.config.require_reference_track_valid:
            try:
                self._require_reference_corridor(path, terrain, hold_hip)
            except TerrainDataUnavailable as exc:
                return self._fallback_result(
                    cycle_index=cycle_index,
                    status=LiveCycleStatus.TERRAIN_UNAVAILABLE,
                    message=str(exc),
                    planning_time_s=0.0,
                    terrain=terrain,
                    hold_hip=hold_hip,
                )

        snapshot_function = getattr(self.planner, "warm_start_snapshot", None)
        warm_start_snapshot = (
            snapshot_function() if callable(snapshot_function) else None
        )
        started = perf_counter()
        try:
            plan = self.planner.plan(
                path=path,
                terrain=terrain,
                previous_hip_rad=hold_hip,
            )
        except TerrainDataUnavailable as exc:
            return self._fallback_result(
                cycle_index=cycle_index,
                status=LiveCycleStatus.TERRAIN_UNAVAILABLE,
                message=str(exc),
                planning_time_s=perf_counter() - started,
                terrain=terrain,
                hold_hip=hold_hip,
            )
        elapsed = perf_counter() - started

        if not plan.feasible:
            return self._fallback_result(
                cycle_index=cycle_index,
                status=LiveCycleStatus.INFEASIBLE,
                message=plan.message,
                planning_time_s=elapsed,
                terrain=terrain,
                hold_hip=hold_hip,
                plan=plan,
            )
        if (
            enforce_deadline
            and
            self.config.planner_deadline_s is not None
            and elapsed > self.config.planner_deadline_s
        ):
            restore_function = getattr(self.planner, "restore_warm_start", None)
            if callable(restore_function):
                restore_function(warm_start_snapshot)
            return self._fallback_result(
                cycle_index=cycle_index,
                status=LiveCycleStatus.DEADLINE_MISSED,
                message=(
                    f"Planning took {elapsed:.3f} s, exceeding the "
                    f"{self.config.planner_deadline_s:.3f} s deadline"
                ),
                planning_time_s=elapsed,
                terrain=terrain,
                hold_hip=hold_hip,
                plan=plan,
            )

        hip_command = plan.first_hip_command_rad
        # This is a body-forward rolling convention, not a motor-driver sign
        # convention. Hardware-side right/left polarity mapping remains below
        # this planner interface.
        speed_rad_s = float(path.speed_m_s[0]) / self.planner.geometry.wheel_radius_m
        wheel_speed = np.full(4, speed_rad_s, dtype=float)
        self._executed_hip_rad = hip_command.copy()
        self._last_valid_plan = plan
        return LiveCycleResult(
            cycle_index=cycle_index,
            status=LiveCycleStatus.PLANNED,
            message=plan.message,
            planning_time_s=elapsed,
            terrain_timestamp_s=terrain.timestamp_s,
            hip_command_rad=hip_command,
            wheel_rolling_speed_rad_s=wheel_speed,
            plan=plan,
        )

    def _fallback_result(
        self,
        *,
        cycle_index: int,
        status: LiveCycleStatus,
        message: str,
        planning_time_s: float,
        terrain: ElevationWindow,
        hold_hip: FloatArray,
        plan: PlanResult | None = None,
    ) -> LiveCycleResult:
        return LiveCycleResult(
            cycle_index=cycle_index,
            status=status,
            message=message,
            planning_time_s=float(planning_time_s),
            terrain_timestamp_s=terrain.timestamp_s,
            hip_command_rad=hold_hip.copy(),
            wheel_rolling_speed_rad_s=np.zeros(4, dtype=float),
            plan=plan,
        )

    def _require_reference_corridor(
        self,
        path: PathHorizon,
        terrain: ElevationWindow,
        hips_rad: FloatArray,
    ) -> None:
        wheel_body = self.planner.geometry.wheel_axes_body(hips_rad)
        body_corners = self.planner.geometry.body_bottom_corners_body()
        query_x: list[FloatArray] = []
        query_y: list[FloatArray] = []
        for k in range(path.steps):
            rotation = rotation_matrix_rpy(0.0, 0.0, path.yaw_rad[k])
            translation = np.array([path.x_m[k], path.y_m[k], 0.0])
            wheels = translation + (rotation @ wheel_body.T).T
            corners = translation + (rotation @ body_corners.T).T
            query_x.extend((wheels[:, 0], corners[:, 0]))
            query_y.extend((wheels[:, 1], corners[:, 1]))
        terrain.height(np.concatenate(query_x), np.concatenate(query_y))
