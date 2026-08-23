from __future__ import annotations

import numpy as np


HIP_JOINT_NAMES = ("FL_hip", "FR_hip", "RL_hip", "RR_hip")


def named_hip_positions(names, positions) -> np.ndarray:
    """Return FL/FR/RL/RR hip positions, independent of JointState ordering."""
    if len(names) != len(positions):
        raise ValueError("JointState name and position arrays have different lengths")
    by_name = dict(zip(names, positions))
    missing = [name for name in HIP_JOINT_NAMES if name not in by_name]
    if missing:
        raise ValueError(f"JointState is missing hip joints: {', '.join(missing)}")
    hips = np.asarray([by_name[name] for name in HIP_JOINT_NAMES], dtype=float)
    if not np.all(np.isfinite(hips)):
        raise ValueError("Hip feedback contains a non-finite value")
    return hips


def bounded_position_step(current, target, max_step_rad: float) -> np.ndarray:
    current_array = np.asarray(current, dtype=float)
    target_array = np.asarray(target, dtype=float)
    if current_array.shape != (4,) or target_array.shape != (4,):
        raise ValueError("Hip vectors must contain exactly four values")
    if not np.isfinite(max_step_rad) or max_step_rad <= 0.0:
        raise ValueError("max_step_rad must be positive and finite")
    return current_array + np.clip(
        target_array - current_array, -max_step_rad, max_step_rad
    )


def smoothstep_position(start, target, elapsed_s: float, duration_s: float) -> np.ndarray:
    """Interpolate a four-hip target with zero velocity at both endpoints."""
    start_array = np.asarray(start, dtype=float)
    target_array = np.asarray(target, dtype=float)
    if start_array.shape != (4,) or target_array.shape != (4,):
        raise ValueError("Hip vectors must contain exactly four values")
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("duration_s must be positive and finite")
    phase = np.clip(elapsed_s / duration_s, 0.0, 1.0)
    blend = phase * phase * (3.0 - 2.0 * phase)
    return start_array + blend * (target_array - start_array)


def stance_wheel_speed_rad_s(
    hip_rad, hip_rate_rad_s, hip_to_wheel_m: float, wheel_radius_m: float
) -> np.ndarray:
    """Wheel rates that accommodate sagittal hip motion with a stationary body."""
    hips = np.asarray(hip_rad, dtype=float)
    rates = np.asarray(hip_rate_rad_s, dtype=float)
    if hips.shape != (4,) or rates.shape != (4,):
        raise ValueError("Hip vectors must contain exactly four values")
    if hip_to_wheel_m <= 0.0 or wheel_radius_m <= 0.0:
        raise ValueError("Link length and wheel radius must be positive")
    return -hip_to_wheel_m * np.cos(hips) * rates / wheel_radius_m
