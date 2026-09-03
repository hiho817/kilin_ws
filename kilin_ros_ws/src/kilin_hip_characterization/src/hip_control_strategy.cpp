#include "kilin_hip_characterization/hip_control_strategy.hpp"

#include <algorithm>
#include <cmath>

namespace kilin_hip_characterization
{
namespace
{
double clamp01(double value) { return std::clamp(value, 0.0, 1.0); }
double rate_limit(double current, double target, double rate, double dt)
{
  if (rate <= 0.0) return target;
  const double step = rate * dt;
  return current + std::clamp(target - current, -step, step);
}
}  // namespace

void HipControlStrategy::configure(const HipControlStrategyConfig & config)
{
  config_ = config;
  reset_all();
}

void HipControlStrategy::reset(std::size_t module)
{
  if (module < state_.size()) state_[module] = ModuleState{};
}

void HipControlStrategy::reset_all()
{
  for (std::size_t module = 0; module < state_.size(); ++module) reset(module);
}

HipControlOutput HipControlStrategy::update(const HipControlInput & input)
{
  HipControlOutput output{input.kp, input.ki, input.kd};
  if (input.module >= state_.size() || !config_.enabled || !input.active ||
    !input.normal_phase || !input.outward_range || !std::isfinite(input.position_diff) ||
    !std::isfinite(input.now_s))
  {
    if (input.module < state_.size()) reset(input.module);
    return output;
  }
  const auto module = input.module;
  const double support_end = config_.support_end[module];
  const double lift_start = config_.lift_start[module];
  if (!std::isfinite(support_end) || !std::isfinite(lift_start) || support_end >= lift_start) return output;

  auto & state = state_[module];
  output.valid = true;
  output.normalized_diff = (module < 2 ? 1.0 : -1.0) * input.position_diff;
  output.pid_blend = clamp01((output.normalized_diff - support_end) / (lift_start - support_end));
  const double dt = state.last_update_s == 0.0 ? 0.0 : std::clamp(input.now_s - state.last_update_s, 0.0, 0.05);
  state.last_update_s = input.now_s;
  const double cap = config_.lift_max_inward_ff > 0.0 ?
    std::min(config_.lift_max_inward_ff, config_.max_abs_ff) : config_.max_abs_ff;
  if (output.normalized_diff >= lift_start) {
    if (!state.lift_latched) {
      state.lift_latched = true;
      state.dwell_s = 0.0;
      state.held_lift_inward_ff = std::clamp(state.held_lift_inward_ff, config_.lift_start_inward_ff, cap);
    }
    state.dwell_s += dt;
    state.held_lift_inward_ff = std::min(cap, state.held_lift_inward_ff + config_.lift_ramp_up_nm_s * dt);
  } else if (output.normalized_diff <= support_end) {
    state.lift_latched = false;
    state.dwell_s = 0.0;
    state.held_lift_inward_ff = config_.reset_lift_on_support ? config_.lift_start_inward_ff :
      std::max(config_.lift_start_inward_ff,
      state.held_lift_inward_ff - config_.lift_ramp_down_nm_s * dt);
  }
  const double lift_target = state.lift_latched ? state.held_lift_inward_ff : config_.lift_start_inward_ff;
  output.target_inward_ff = config_.support_inward_ff;
  if (output.normalized_diff >= lift_start) output.target_inward_ff = lift_target;
  else if (output.normalized_diff > support_end) output.target_inward_ff =
      (1.0 - output.pid_blend) * config_.support_inward_ff + output.pid_blend * lift_target;
  const double rate = output.target_inward_ff >= state.applied_inward_ff ?
    config_.apply_rate_nm_s : config_.release_rate_nm_s;
  state.applied_inward_ff = rate_limit(state.applied_inward_ff, output.target_inward_ff, rate, dt);
  output.applied_inward_ff = state.applied_inward_ff;
  output.lift_dwell_s = state.dwell_s;
  output.lift_latched = state.lift_latched;
  output.motor_ff = std::clamp((module < 2 ? 1.0 : -1.0) * output.applied_inward_ff,
    -config_.max_abs_ff, config_.max_abs_ff);
  if (!config_.pid_schedule_enabled) return output;

  const double target_kp = (1.0 - output.pid_blend) * config_.support_kp + output.pid_blend * config_.lift_kp;
  const double target_ki = (1.0 - output.pid_blend) * config_.support_ki + output.pid_blend * config_.lift_ki;
  const double target_kd = (1.0 - output.pid_blend) * config_.support_kd + output.pid_blend * config_.lift_kd;
  if (!state.pid_initialized) {
    state.applied_kp = input.kp; state.applied_ki = input.ki; state.applied_kd = input.kd;
    state.previous_blend = 0.0; state.pid_initialized = true;
  }
  const bool toward_lift = output.pid_blend >= state.previous_blend;
  state.applied_kp = rate_limit(state.applied_kp, target_kp,
    toward_lift ? config_.kp_to_lift_rate_per_s : config_.kp_to_support_rate_per_s, dt);
  state.applied_ki = rate_limit(state.applied_ki, target_ki,
    toward_lift ? config_.ki_to_lift_rate_per_s : config_.ki_to_support_rate_per_s, dt);
  state.applied_kd = rate_limit(state.applied_kd, target_kd,
    toward_lift ? config_.kd_to_lift_rate_per_s : config_.kd_to_support_rate_per_s, dt);
  state.previous_blend = output.pid_blend;
  output.kp = state.applied_kp; output.ki = state.applied_ki; output.kd = state.applied_kd;
  return output;
}
}  // namespace kilin_hip_characterization
