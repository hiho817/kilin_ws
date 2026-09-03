#pragma once

#include <array>
#include <cstddef>

namespace kilin_hip_characterization
{
struct HipControlStrategyConfig
{
  bool enabled{};
  bool pid_schedule_enabled{};
  std::array<double, 4> support_end{};
  std::array<double, 4> lift_start{};
  double support_kp{}, support_ki{}, support_kd{};
  double lift_kp{}, lift_ki{}, lift_kd{};
  double lift_start_inward_ff{}, lift_max_inward_ff{};
  double lift_ramp_up_nm_s{}, lift_ramp_down_nm_s{};
  bool reset_lift_on_support{};
  double support_inward_ff{}, apply_rate_nm_s{}, release_rate_nm_s{}, max_abs_ff{};
  double kp_to_lift_rate_per_s{}, kp_to_support_rate_per_s{};
  double ki_to_lift_rate_per_s{}, ki_to_support_rate_per_s{};
  double kd_to_lift_rate_per_s{}, kd_to_support_rate_per_s{};
};

// One complete sample. New strategy inputs belong here, not in a growing API.
struct HipControlInput
{
  std::size_t module{};
  double motor_position{}, position_diff{}, commanded_motor_position{}, now_s{};
  bool active{}, normal_phase{}, outward_range{};
  double kp{}, ki{}, kd{};
};

struct HipControlOutput
{
  double kp{}, ki{}, kd{}, motor_ff{}, normalized_diff{}, pid_blend{};
  double target_inward_ff{}, applied_inward_ff{}, lift_dwell_s{};
  bool lift_latched{}, valid{};
};

// Stateful, ROS-independent four-module engine used by self-test and runtime.
class HipControlStrategy
{
public:
  void configure(const HipControlStrategyConfig & config);
  HipControlOutput update(const HipControlInput & input);
  void reset(std::size_t module);
  void reset_all();

private:
  struct ModuleState
  {
    double last_update_s{}, held_lift_inward_ff{}, applied_inward_ff{};
    double applied_kp{}, applied_ki{}, applied_kd{}, previous_blend{}, dwell_s{};
    bool lift_latched{}, pid_initialized{};
  };
  HipControlStrategyConfig config_{};
  std::array<ModuleState, 4> state_{};
};
}  // namespace kilin_hip_characterization
