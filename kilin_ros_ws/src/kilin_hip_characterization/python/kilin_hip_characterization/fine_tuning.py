"""Python-facing adapter for the C++ fine-tuning core.

Callers own timing: call :meth:`FineTuneStrategy.step` exactly once from their
own command tick and pass that tick's monotonic timestamp in ``sample.now_s``.
"""

from collections.abc import Mapping

from ._hip_control_strategy import ControlResult, ControlSample, FineTuneStrategy, StrategyConfig


def strategy_config_from_parameters(parameters: Mapping[str, object]) -> StrategyConfig:
    """Create the C++ configuration from one versioned YAML parameter mapping.

    This is the sole Python-to-strategy mapping. Controllers pass the resolved
    profile mapping as a whole; they never reproduce individual tuning ports.
    """
    config = StrategyConfig()
    config.enabled = parameters.get("feedforward_mode") == "angle_diff_lift_assist"
    config.pid_schedule_enabled = bool(parameters.get("lift_assist_pid_schedule_enabled", False))
    config.support_end = list(parameters["lift_assist_support_region_end_rad"])
    config.lift_start = list(parameters["lift_assist_lift_region_start_rad"])
    config.support_kp = float(parameters["lift_assist_support_kp"])
    config.support_ki = float(parameters["lift_assist_support_ki"])
    config.support_kd = float(parameters["lift_assist_support_kd"])
    config.lift_kp = float(parameters["lift_assist_lift_kp"])
    config.lift_ki = float(parameters["lift_assist_lift_ki"])
    config.lift_kd = float(parameters["lift_assist_lift_kd"])
    config.lift_start_inward_ff = float(parameters.get("lift_assist_lift_start_inward_ff_nm", 0.0))
    config.lift_max_inward_ff = float(parameters.get("lift_assist_lift_max_inward_ff_nm", 0.0))
    legacy_lift_ramp = float(parameters.get("lift_assist_lift_ramp_nm_s", 0.0))
    directional_lift_ramp = float(parameters.get("lift_assist_lift_ramp_up_nm_s", -1.0))
    config.lift_ramp_up_nm_s = legacy_lift_ramp if directional_lift_ramp < 0.0 else directional_lift_ramp
    config.lift_ramp_down_nm_s = max(0.0, float(parameters.get("lift_assist_lift_ramp_down_nm_s", -1.0)))
    config.reset_lift_on_support = float(parameters.get("lift_assist_lift_ramp_down_nm_s", -1.0)) < 0.0
    config.support_inward_ff = float(parameters.get("lift_assist_support_inward_ff_nm", 0.0))
    config.apply_rate_nm_s = float(parameters.get("lift_assist_apply_rate_nm_s", 0.0))
    config.release_rate_nm_s = float(parameters.get("lift_assist_release_rate_nm_s", 0.0))
    config.max_abs_ff = float(parameters["max_abs_hip_ff_torque_nm"])
    for gain in ("kp", "ki", "kd"):
        legacy = float(parameters.get(f"lift_assist_pid_{gain}_rate_per_s", 0.0))
        for direction in ("to_lift", "to_support"):
            value = float(parameters.get(f"{gain}_{direction}_rate_per_s", -1.0))
            setattr(config, f"{gain}_{direction}_rate_per_s", legacy if value < 0.0 else value)
    return config

__all__ = ["ControlResult", "ControlSample", "FineTuneStrategy", "StrategyConfig", "strategy_config_from_parameters"]
