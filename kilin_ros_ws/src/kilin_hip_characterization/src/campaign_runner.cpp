#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/motor_state_stamped.hpp"

namespace fs = std::filesystem;
constexpr int kRest = 0, kPosition = 4, kVelocity = 5, kTorque = 6, kBrake = 7;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kRadSToRpm10 = 60.0 * 10.0 / (2.0 * M_PI);
constexpr std::array<const char *, 4> kModuleNames{"A", "B", "C", "D"};

enum class Phase { kWaitForState, kStartupMove, kStartHold, kMoveToB,
                   kHoldAtB, kMoveToA, kRecoveryRest, kRecoveryMove,
                   kComplete, kAborted };

const char *phaseName(Phase phase) {
  switch (phase) {
    case Phase::kWaitForState: return "waiting_for_state";
    case Phase::kStartupMove: return "startup_move_to_state_a";
    case Phase::kStartHold: return "state_a_hold";
    case Phase::kMoveToB: return "move_to_state_b";
    case Phase::kHoldAtB: return "state_b_hold";
    case Phase::kMoveToA: return "return_to_state_a";
    case Phase::kRecoveryRest: return "recovery_rest";
    case Phase::kRecoveryMove: return "recovery_move";
    case Phase::kComplete: return "complete";
    case Phase::kAborted: return "aborted";
  }
  return "unknown";
}

class CampaignRunner final : public rclcpp::Node {
 public:
  CampaignRunner() : Node("kilin_hip_characterization") {
    armed_ = declare_parameter<bool>("armed", false);
    command_topic_ = declare_parameter<std::string>("command_topic", "/kilin/hip_characterization/command_preview");
    state_topic_ = declare_parameter<std::string>("state_topic", "/motor/state");
    run_dir_ = declare_parameter<std::string>("run_dir", "");
    strategy_name_ = declare_parameter<std::string>("strategy_name", "phase_a_two_state_baseline");
    strategy_version_ = declare_parameter<std::string>("strategy_version", "1.5.2");
    active_modules_ = declare_parameter<std::vector<std::string>>("active_modules", {"A", "B"});
    repetitions_ = declare_parameter<int>("repetitions", 3);
    state_a_deg_ = declare_parameter<double>("state_a_deg", 0.0);
    state_b_deg_ = declare_parameter<double>("state_b_deg", 45.0);
    startup_speed_ = declare_parameter<double>("startup_move_speed_rad_s", 0.1);
    recovery_deg_ = declare_parameter<double>("recovery_position_deg", 0.0);
    recovery_speed_ = declare_parameter<double>("recovery_move_speed_rad_s", 0.1);
    recovery_rest_s_ = declare_parameter<double>("recovery_rest_s", 1.0);
    hip_speed_ = declare_parameter<double>("hip_speed_rad_s", 0.2);
    kp_ = declare_parameter<double>("kp", 360.0);
    ki_ = declare_parameter<double>("ki", 0.0);
    kd_ = declare_parameter<double>("kd", 5.0);
    max_ff_ = declare_parameter<double>("max_abs_hip_ff_torque_nm", 200.0);
    static_breakaway_policy_ = declare_parameter<std::string>("static_breakaway_policy", "disabled");
    static_breakaway_steps_outward_ = declare_parameter<std::vector<double>>("static_breakaway_steps_outward_nm", {0.0, 0.0, 0.0});
    static_breakaway_steps_inward_ = declare_parameter<std::vector<double>>("static_breakaway_steps_inward_nm", {0.0, 0.0, 0.0});
    static_breakaway_step_1_s_ = declare_parameter<double>("static_breakaway_step_1_s", 0.10);
    static_breakaway_step_2_s_ = declare_parameter<double>("static_breakaway_step_2_s", 0.30);
    static_breakaway_exp_start_outward_ = declare_parameter<double>("static_breakaway_exp_start_outward_nm", 0.0);
    static_breakaway_exp_peak_outward_ = declare_parameter<double>("static_breakaway_exp_peak_outward_nm", 0.0);
    static_breakaway_exp_start_inward_ = declare_parameter<double>("static_breakaway_exp_start_inward_nm", 0.0);
    static_breakaway_exp_peak_inward_ = declare_parameter<double>("static_breakaway_exp_peak_inward_nm", 0.0);
    static_breakaway_exp_tau_s_ = declare_parameter<double>("static_breakaway_exp_tau_s", 0.20);
    static_breakaway_error_enable_rad_ = declare_parameter<double>("static_breakaway_error_enable_rad", 0.02);
    static_breakaway_error_full_rad_ = declare_parameter<double>("static_breakaway_error_full_rad", 0.05);
    static_breakaway_error_disable_rad_ = declare_parameter<double>("static_breakaway_error_disable_rad", 0.01);
    static_breakaway_dwell_speed_rad_s_ = declare_parameter<double>("static_breakaway_dwell_speed_rad_s", 0.03);
    static_breakaway_release_speed_rad_s_ = declare_parameter<double>("static_breakaway_release_speed_rad_s", 0.10);
    static_breakaway_velocity_fade_rad_s_ = declare_parameter<double>("static_breakaway_velocity_fade_rad_s", 0.05);
    static_breakaway_velocity_fade_power_ = declare_parameter<double>("static_breakaway_velocity_fade_power", 2.0);
    static_breakaway_velocity_filter_alpha_ = declare_parameter<double>("static_breakaway_velocity_filter_alpha", 0.20);
    static_breakaway_velocity_source_ = declare_parameter<std::string>("static_breakaway_velocity_source", "raw");
    static_breakaway_angle_mode_ = declare_parameter<std::string>("static_breakaway_angle_mode", "none");
    static_breakaway_angle_blend_outward_ = declare_parameter<double>("static_breakaway_angle_blend_outward", 0.0);
    static_breakaway_angle_blend_inward_ = declare_parameter<double>("static_breakaway_angle_blend_inward", 0.0);
    static_breakaway_angle_min_deg_ = declare_parameter<double>("static_breakaway_angle_min_deg", 15.0);
    static_breakaway_angle_max_deg_ = declare_parameter<double>("static_breakaway_angle_max_deg", 90.0);
    start_hold_s_ = declare_parameter<double>("start_zero_hold_s", 2.0);
    segment_hold_s_ = declare_parameter<double>("segment_hold_s", 1.0);
    max_state_age_s_ = declare_parameter<double>("max_state_age_s", 0.10);
    max_error_rad_ = declare_parameter<double>("max_abs_hip_error_rad", 0.35);
    max_torque_ = declare_parameter<double>("max_abs_hip_torque_nm", 400.0);
    wheel_mode_ = declare_parameter<std::string>("wheel_mode", "rest");
    wheel_torque_outward_ = declare_parameter<double>("wheel_torque_outward_nm", 0.0);
    wheel_torque_inward_ = declare_parameter<double>("wheel_torque_inward_nm", 0.0);
    max_wheel_torque_ = declare_parameter<double>("max_abs_hub_torque_nm", 20.0);
    hip_to_wheel_m_ = declare_parameter<double>("hip_to_wheel_m", 0.260);
    wheel_radius_m_ = declare_parameter<double>("wheel_radius_m", 0.051);
    wheel_rate_deadband_rad_s_ = declare_parameter<double>("wheel_rate_deadband_rad_s", 0.02);

    if (armed_ && run_dir_.empty()) throw std::runtime_error("armed run requires run_dir");
    if (wheel_mode_ == "speed") wheel_mode_ = "speed_ik";
    if (wheel_mode_ == "torque") wheel_mode_ = "torque_assist";
    const bool known_wheel_mode = wheel_mode_ == "rest" || wheel_mode_ == "speed_ik" || wheel_mode_ == "torque_assist";
    const bool known_breakaway_policy = static_breakaway_policy_ == "disabled" ||
        static_breakaway_policy_ == "steps" || static_breakaway_policy_ == "exponential";
    const bool known_angle_mode = static_breakaway_angle_mode_ == "none" ||
        static_breakaway_angle_mode_ == "sine";
    const bool known_velocity_source = static_breakaway_velocity_source_ == "filtered" ||
        static_breakaway_velocity_source_ == "raw";
    if (repetitions_ < 1 || startup_speed_ <= 0.0 || recovery_speed_ <= 0.0 || hip_speed_ <= 0.0 ||
        !known_wheel_mode ||
        max_wheel_torque_ < 0.0 || hip_to_wheel_m_ <= 0.0 || wheel_radius_m_ <= 0.0 ||
        wheel_rate_deadband_rad_s_ < 0.0 || !known_breakaway_policy || !known_angle_mode || !known_velocity_source ||
        static_breakaway_steps_outward_.size() != 3 || static_breakaway_steps_inward_.size() != 3 ||
        static_breakaway_step_1_s_ < 0.0 || static_breakaway_step_2_s_ < static_breakaway_step_1_s_ ||
        static_breakaway_exp_tau_s_ <= 0.0 || static_breakaway_error_disable_rad_ < 0.0 ||
        static_breakaway_error_enable_rad_ < static_breakaway_error_disable_rad_ ||
        static_breakaway_error_full_rad_ < static_breakaway_error_enable_rad_ ||
        static_breakaway_dwell_speed_rad_s_ < 0.0 || static_breakaway_release_speed_rad_s_ < 0.0 ||
        static_breakaway_velocity_fade_rad_s_ < 0.0 || static_breakaway_velocity_fade_power_ <= 0.0 ||
        static_breakaway_velocity_filter_alpha_ <= 0.0 || static_breakaway_velocity_filter_alpha_ > 1.0 ||
        static_breakaway_angle_blend_outward_ < 0.0 || static_breakaway_angle_blend_outward_ > 1.0 ||
        static_breakaway_angle_blend_inward_ < 0.0 || static_breakaway_angle_blend_inward_ > 1.0 ||
        static_breakaway_angle_min_deg_ < 0.0 || static_breakaway_angle_max_deg_ < static_breakaway_angle_min_deg_) {
      throw std::runtime_error("invalid trajectory, wheel-mode, or near-zero-breakaway setting");
    }
    for (const auto &name : active_modules_) {
      if (std::find(kModuleNames.begin(), kModuleNames.end(), name) == kModuleNames.end()) throw std::runtime_error("unknown active module");
    }
    publisher_ = create_publisher<kilin_msgs::msg::MotorCmdStamped>(command_topic_, 10);
    subscriber_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(state_topic_, rclcpp::QoS(20).best_effort(), std::bind(&CampaignRunner::onState, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&CampaignRunner::tick, this));
    RCLCPP_INFO(get_logger(), "Runner ready: strategy=%s v%s armed=%s command_topic=%s wheel_mode=%s", strategy_name_.c_str(), strategy_version_.c_str(), armed_ ? "true" : "false", command_topic_.c_str(), wheel_mode_.c_str());
  }

 private:
  struct HipState { double motor{NAN}; double diff{NAN}; double velocity{NAN}; double torque{NAN}; int error{0}; };
  struct BreakawayState {
    double dwell_s{0.0};
    double last_update_s{0.0};
    int direction_sign{0};
    bool error_enabled{false};
    bool released{false};
  };

  bool active(size_t i) const { return std::find(active_modules_.begin(), active_modules_.end(), kModuleNames[i]) != active_modules_.end(); }
  double actual(size_t i) const { return hips_[i].motor + hips_[i].diff; }
  double sign(size_t i) const { return i < 2 ? -1.0 : 1.0; }
  double stateA(size_t i) const { return active(i) ? sign(i) * state_a_deg_ * kDegToRad : 0.0; }
  double stateB(size_t i) const { return active(i) ? sign(i) * state_b_deg_ * kDegToRad : 0.0; }
  double recovery(size_t i) const { return active(i) ? sign(i) * recovery_deg_ * kDegToRad : 0.0; }
  bool freshState() const { return have_state_ && (now() - last_state_).seconds() <= max_state_age_s_; }
  double elapsed() const { return (now() - phase_start_).seconds(); }
  static double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }
  static double smooth(double x) { x = clamp01(x); return x * x * (3.0 - 2.0 * x); }

  void onState(const kilin_msgs::msg::MotorStateStamped::SharedPtr message) {
    const std::array<kilin_msgs::msg::LegState, 4> legs{message->module_a, message->module_b, message->module_c, message->module_d};
    for (size_t i = 0; i < 4; ++i) {
      hips_[i] = {legs[i].hip.position, legs[i].hip.position_diff, legs[i].hip.velocity, legs[i].hip.torque, legs[i].hip.error_code};
      if (!std::isfinite(hips_[i].velocity)) {
        filtered_hip_velocity_[i] = 0.0;
      } else if (!std::isfinite(filtered_hip_velocity_[i])) {
        filtered_hip_velocity_[i] = hips_[i].velocity;
      } else {
        filtered_hip_velocity_[i] = static_breakaway_velocity_filter_alpha_ * hips_[i].velocity +
            (1.0 - static_breakaway_velocity_filter_alpha_) * filtered_hip_velocity_[i];
      }
    }
    last_state_ = now();
    have_state_ = true;
  }

  void begin() {
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(actual(i))) throw std::runtime_error("non-finite hip feedback");
      move_start_[i] = hips_[i].motor;
      commanded_[i] = hips_[i].motor;
      previous_target_[i] = hips_[i].motor;
    }
    previous_target_time_ = now();
    openEvidence();
    setPhase(Phase::kStartupMove, "moving to configured state A with wheels rest and zero FF");
  }

  void setPhase(Phase next, const std::string &detail) {
    phase_ = next;
    phase_start_ = now();
    if (next == Phase::kMoveToB || next == Phase::kMoveToA) {
      for (auto &state : breakaway_) state = BreakawayState{};
    }
    RCLCPP_INFO(get_logger(), "Phase %s: %s", phaseName(next), detail.c_str());
  }

  double moveDuration(bool to_recovery) const {
    double distance = 0.0;
    for (size_t i = 0; i < 4; ++i) distance = std::max(distance, std::abs((to_recovery ? recovery(i) : stateA(i)) - move_start_[i]));
    return std::max(0.01, distance / (to_recovery ? recovery_speed_ : startup_speed_));
  }
  double fullStrokeDuration() const {
    double distance = 0.0;
    for (size_t i = 0; i < 4; ++i) distance = std::max(distance, std::abs(stateB(i) - stateA(i)));
    return std::max(0.01, distance / hip_speed_);
  }

  std::string safetyViolation() const {
    for (size_t i = 0; i < 4; ++i) {
      std::ostringstream message;
      message << "module=" << kModuleNames[i] << " phase=" << phaseName(phase_);
      if (!std::isfinite(actual(i))) return message.str() + " reason=non_finite_actual_hip";
      if (!std::isfinite(hips_[i].torque)) return message.str() + " reason=non_finite_hip_torque";
      if (hips_[i].error != 0) return message.str() + " reason=hip_motor_reported_fault error_code=" + std::to_string(hips_[i].error);
      if (std::abs(hips_[i].torque) > max_torque_) {
        message << " reason=hip_torque_limit measured=" << hips_[i].torque << " limit=" << max_torque_;
        return message.str();
      }
      // Control is intentionally in the raw motor-position frame for strategy
      // 1.1.0.  position_diff remains an observed quantity only; feeding its
      // noisy/backlash-dependent value back into the motor position target
      // caused command chatter on the real robot.
      const double tracking_error = hips_[i].motor - commanded_[i];
      if (std::abs(tracking_error) > max_error_rad_) {
        message << " reason=hip_motor_tracking_error_limit commanded_motor_rad=" << commanded_[i]
                << " motor_position_rad=" << hips_[i].motor << " error_rad=" << tracking_error
                << " abs_limit_rad=" << max_error_rad_;
        return message.str();
      }
    }
    return "";
  }

  bool wheelMotionPhase() const {
    return phase_ == Phase::kMoveToB || phase_ == Phase::kMoveToA;
  }

  double wheelRateRadS(double hip_rate_rad_s, double commanded_hip_rad) const {
    return -hip_to_wheel_m_ * std::cos(commanded_hip_rad) * hip_rate_rad_s / wheel_radius_m_;
  }

  void commandHub(kilin_msgs::msg::LegCmd &leg, size_t i, double hip_rate_rad_s, double commanded_hip_rad) {
    commanded_hub_velocity_rpm10_[i] = 0.0;
    commanded_hub_torque_[i] = 0.0;
    commanded_hub_mode_[i] = kRest;
    leg.hub.motor_mode = kRest;
    if (!active(i) || !wheelMotionPhase() || std::abs(hip_rate_rad_s) < 1e-6 || wheel_mode_ == "rest") return;

    const double wheel_rate = wheelRateRadS(hip_rate_rad_s, commanded_hip_rad);
    if (std::abs(wheel_rate) < wheel_rate_deadband_rad_s_) return;
    if (wheel_mode_ == "speed_ik") {
      leg.hub.motor_mode = kVelocity;
      leg.hub.velocity = wheel_rate * kRadSToRpm10;
      commanded_hub_velocity_rpm10_[i] = leg.hub.velocity;
      commanded_hub_mode_[i] = kVelocity;
      return;
    }

    const bool outward = hip_rate_rad_s * sign(i) > 0.0;
    const double configured_torque = outward ? wheel_torque_outward_ : wheel_torque_inward_;
    leg.hub.motor_mode = kTorque;
    // A positive configured value assists the calculated IK wheel direction;
    // a negative value deliberately opposes it.  The clamp preserves that
    // signed policy instead of silently turning a negative value positive.
    leg.hub.torque = std::clamp(std::copysign(1.0, wheel_rate) * configured_torque,
                                -max_wheel_torque_, max_wheel_torque_);
    commanded_hub_torque_[i] = leg.hub.torque;
    commanded_hub_mode_[i] = kTorque;
  }

  void commandPosition(const std::array<double, 4> &target, bool allow_ff) {
    kilin_msgs::msg::MotorCmdStamped message;
    stamp(message);
    std::array<kilin_msgs::msg::LegCmd, 4> legs;
    const auto command_time = now();
    const double dt_s = std::max(1e-3, (command_time - previous_target_time_).seconds());
    for (size_t i = 0; i < 4; ++i) {
      const double hip_rate = (target[i] - previous_target_[i]) / dt_s;
      commanded_[i] = target[i];
      legs[i].hip.motor_mode = kPosition;
      // Do not compensate the motor command with position_diff.  It is kept in
      // the state/logging path as the reconstructed actual hip angle, but it is
      // not part of this low-level position-control loop.
      legs[i].hip.position = target[i];
      legs[i].hip.kp = kp_; legs[i].hip.ki = ki_; legs[i].hip.kd = kd_;
      raw_hip_velocity_trace_[i] = hips_[i].velocity;
      breakaway_velocity_used_trace_[i] = breakawayVelocity(i);
      commanded_hip_ff_[i] = allow_ff ? feedforward(i, target[i] - hips_[i].motor) : 0.0;
      if (!allow_ff) {
        commanded_static_breakaway_ff_[i] = 0.0;
        breakaway_dwell_trace_[i] = 0.0;
        breakaway_released_trace_[i] = 0;
      }
      legs[i].hip.torque = commanded_hip_ff_[i];
      legs[i].steering.motor_mode = kPosition;
      legs[i].steering.position = 0.0;
      commandHub(legs[i], i, hip_rate, target[i]);
      previous_target_[i] = target[i];
    }
    previous_target_time_ = command_time;
    message.module_a = legs[0]; message.module_b = legs[1]; message.module_c = legs[2]; message.module_d = legs[3];
    publisher_->publish(message);
    writeTrace();
  }

  double breakawayAngleFactor(size_t i, bool outward) const {
    if (static_breakaway_angle_mode_ == "none") return 1.0;
    const double angle = std::abs(commanded_[i]);
    const double shape = std::max(0.0, std::sin(angle));
    const double blend = outward ? static_breakaway_angle_blend_outward_ : static_breakaway_angle_blend_inward_;
    return (1.0 - blend) + blend * shape;
  }

  bool breakawayAngleInRange(size_t i) const {
    const double angle_deg = std::abs(commanded_[i]) / kDegToRad;
    return angle_deg >= static_breakaway_angle_min_deg_ && angle_deg <= static_breakaway_angle_max_deg_;
  }

  double breakawayBaseAmplitude(bool outward, double dwell_s) const {
    if (static_breakaway_policy_ == "steps") {
      const auto &steps = outward ? static_breakaway_steps_outward_ : static_breakaway_steps_inward_;
      if (dwell_s < static_breakaway_step_1_s_) return steps[0];
      if (dwell_s < static_breakaway_step_2_s_) return steps[1];
      return steps[2];
    }
    const double start = outward ? static_breakaway_exp_start_outward_ : static_breakaway_exp_start_inward_;
    const double peak = outward ? static_breakaway_exp_peak_outward_ : static_breakaway_exp_peak_inward_;
    return start + (peak - start) * (1.0 - std::exp(-dwell_s / static_breakaway_exp_tau_s_));
  }

  double breakawayVelocity(size_t i) const {
    const double selected = static_breakaway_velocity_source_ == "raw"
        ? hips_[i].velocity : filtered_hip_velocity_[i];
    return std::isfinite(selected) ? selected : 0.0;
  }

  double breakawayVelocityGate(size_t i) const {
    if (static_breakaway_velocity_fade_rad_s_ == 0.0) return 1.0;
    const double ratio = std::abs(breakawayVelocity(i)) / static_breakaway_velocity_fade_rad_s_;
    return std::exp(-std::pow(ratio, static_breakaway_velocity_fade_power_));
  }

  double staticBreakawayFF(size_t i, double error, bool outward) {
    commanded_static_breakaway_ff_[i] = 0.0;
    breakaway_dwell_trace_[i] = 0.0;
    breakaway_released_trace_[i] = 0;
    if (static_breakaway_policy_ == "disabled") return NAN;
    if (!breakawayAngleInRange(i)) return 0.0;

    auto &state = breakaway_[i];
    const double time_s = now().seconds();
    const double dt_s = state.last_update_s == 0.0 ? 0.0 : std::clamp(time_s - state.last_update_s, 0.0, 0.05);
    state.last_update_s = time_s;
    const int direction_sign = error >= 0.0 ? 1 : -1;
    if (state.direction_sign != 0 && state.direction_sign != direction_sign) {
      state = BreakawayState{};
      state.last_update_s = time_s;
    }
    state.direction_sign = direction_sign;

    const double error_abs = std::abs(error);
    if (!state.error_enabled && error_abs >= static_breakaway_error_enable_rad_) state.error_enabled = true;
    if (state.error_enabled && error_abs <= static_breakaway_error_disable_rad_) state.error_enabled = false;
    if (!state.error_enabled) {
      state.dwell_s = 0.0;
      return 0.0;
    }

    const double velocity_abs = std::abs(breakawayVelocity(i));
    if (static_breakaway_release_speed_rad_s_ > 0.0 && velocity_abs >= static_breakaway_release_speed_rad_s_) {
      state.released = true;
    }
    if (state.released) {
      breakaway_released_trace_[i] = 1;
      return 0.0;
    }
    if (static_breakaway_dwell_speed_rad_s_ > 0.0 && velocity_abs > static_breakaway_dwell_speed_rad_s_) {
      state.dwell_s = 0.0;
      return 0.0;
    }
    state.dwell_s += dt_s;

    const double error_gate = static_breakaway_error_full_rad_ == static_breakaway_error_enable_rad_
        ? 1.0
        : smooth((error_abs - static_breakaway_error_enable_rad_) /
                 (static_breakaway_error_full_rad_ - static_breakaway_error_enable_rad_));
    const double amplitude = breakawayBaseAmplitude(outward, state.dwell_s) *
        error_gate * breakawayVelocityGate(i) * breakawayAngleFactor(i, outward);
    const double command = std::clamp(std::copysign(1.0, error) * amplitude, -max_ff_, max_ff_);
    commanded_static_breakaway_ff_[i] = command;
    breakaway_dwell_trace_[i] = state.dwell_s;
    return command;
  }

  double feedforward(size_t i, double error) {
    commanded_static_breakaway_ff_[i] = 0.0;
    breakaway_dwell_trace_[i] = 0.0;
    breakaway_released_trace_[i] = 0;
    if (!active(i) || std::abs(error) < 0.01) return 0.0;
    // The raw error sign sets corrective torque polarity. The module convention
    // maps that same error into the physical outward/inward label: identical
    // error signs mean opposite labels for front and rear modules.
    const bool outward = error * sign(i) > 0.0;
    if (wheelMotionPhase()) {
      const double breakaway = staticBreakawayFF(i, error, outward);
      if (std::isfinite(breakaway)) return breakaway;
    }
    return 0.0;
  }

  std::array<double, 4> interpolated(const std::array<double, 4> &from, const std::array<double, 4> &to, double fraction) const {
    std::array<double, 4> result{};
    for (size_t i = 0; i < 4; ++i) result[i] = from[i] + fraction * (to[i] - from[i]);
    return result;
  }

  std::array<double, 4> stateArray(bool b) const { std::array<double, 4> result{}; for (size_t i = 0; i < 4; ++i) result[i] = b ? stateB(i) : stateA(i); return result; }
  std::array<double, 4> recoveryArray() const { std::array<double, 4> result{}; for (size_t i = 0; i < 4; ++i) result[i] = recovery(i); return result; }

  void tick() {
    if (!armed_) return;
    if (!freshState()) { if (phase_ != Phase::kWaitForState) abort("/motor/state is stale"); return; }
    if (phase_ == Phase::kWaitForState) { try { begin(); } catch (const std::exception &e) { abort(e.what()); } return; }
    const std::string violation = safetyViolation();
    if (!violation.empty()) { abort(violation); return; }
    const auto a = stateArray(false), b = stateArray(true), r = recoveryArray();
    switch (phase_) {
      case Phase::kStartupMove:
        commandPosition(interpolated(move_start_, a, smooth(elapsed() / moveDuration(false))), false);
        if (elapsed() >= moveDuration(false)) setPhase(Phase::kStartHold, "state A reached; beginning test hold");
        break;
      case Phase::kStartHold:
        commandPosition(a, false);
        if (elapsed() >= start_hold_s_) setPhase(Phase::kMoveToB, "beginning dynamic move to state B");
        break;
      case Phase::kMoveToB:
        commandPosition(interpolated(a, b, clamp01(elapsed() / fullStrokeDuration())), true);
        if (elapsed() >= fullStrokeDuration()) setPhase(Phase::kHoldAtB, "state B reached");
        break;
      case Phase::kHoldAtB:
        commandPosition(b, false);
        if (elapsed() >= segment_hold_s_) setPhase(Phase::kMoveToA, "returning to state A");
        break;
      case Phase::kMoveToA:
        commandPosition(interpolated(b, a, clamp01(elapsed() / fullStrokeDuration())), true);
        if (elapsed() >= fullStrokeDuration()) {
          if (++completed_repetitions_ >= repetitions_) setPhase(Phase::kRecoveryRest, "unit test complete; resting all motors");
          else setPhase(Phase::kStartHold, "next repetition");
        }
        break;
      case Phase::kRecoveryRest:
        publishRest();
        if (elapsed() >= recovery_rest_s_) { for (size_t i = 0; i < 4; ++i) move_start_[i] = hips_[i].motor; setPhase(Phase::kRecoveryMove, "moving to configured recovery position"); }
        break;
      case Phase::kRecoveryMove:
        commandPosition(interpolated(move_start_, r, smooth(elapsed() / moveDuration(true))), false);
        if (elapsed() >= moveDuration(true)) complete();
        break;
      default: break;
    }
  }

  void abort(const std::string &reason) {
    if (phase_ == Phase::kAborted) return;
    publishRest();
    setPhase(Phase::kAborted, reason);
    RCLCPP_ERROR(get_logger(), "Campaign aborted: %s", reason.c_str());
    if (!run_dir_.empty()) {
      std::ofstream out(fs::path(run_dir_) / "abort_reason.txt");
      out << "phase: " << phaseName(phase_) << "\nreason: " << reason << "\n";
    }
    rclcpp::shutdown();
  }
  void complete() { publishRest(); setPhase(Phase::kComplete, "all repetitions and recovery complete"); RCLCPP_INFO(get_logger(), "Campaign complete."); rclcpp::shutdown(); }
  void stamp(kilin_msgs::msg::MotorCmdStamped &message) { const auto t = now(); message.header.seq = sequence_++; message.header.time.sec = static_cast<int32_t>(t.seconds()); message.header.time.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL); message.header.frame_id = "kilin_hip_characterization"; }
  void publishRest() { kilin_msgs::msg::MotorCmdStamped message; stamp(message); kilin_msgs::msg::LegCmd leg; leg.hip.motor_mode=kRest; leg.steering.motor_mode=kRest; leg.hub.motor_mode=kRest; message.module_a=leg;message.module_b=leg;message.module_c=leg;message.module_d=leg;publisher_->publish(message); }
  void openEvidence() {
    if (run_dir_.empty()) return;
    fs::create_directories(run_dir_);
    trace_.open(fs::path(run_dir_) / "command_state_trace.csv");
    trace_ << "time_s,phase,trial,module,commanded_motor_rad,motor_position_rad,position_diff_rad,actual_hip_angle_rad,hip_torque,commanded_hip_ff,raw_hip_velocity_rad_s,filtered_hip_velocity_rad_s,breakaway_velocity_used_rad_s,static_breakaway_ff,static_breakaway_dwell_s,static_breakaway_released,hub_mode,commanded_hub_velocity_rpm10,commanded_hub_torque,error_code\n";
    std::ofstream manifest(fs::path(run_dir_) / "trial_manifest.yaml");
    manifest << "strategy_name: " << strategy_name_ << "\nstrategy_version: " << strategy_version_
             << "\nangle_convention: actual_hip_angle_rad = motor_position + position_diff"
             << "\ncontrol_angle_reference: motor_position (position_diff is measurement-only)"
             << "\nwheel_mode: " << wheel_mode_
             << "\nwheel_torque_outward_nm: " << wheel_torque_outward_
             << "\nwheel_torque_inward_nm: " << wheel_torque_inward_
             << "\nmax_abs_hub_torque_nm: " << max_wheel_torque_
             << "\nstatic_breakaway_policy: " << static_breakaway_policy_
             << "\nstatic_breakaway_steps_outward_nm: [" << static_breakaway_steps_outward_[0] << ", " << static_breakaway_steps_outward_[1] << ", " << static_breakaway_steps_outward_[2] << "]"
             << "\nstatic_breakaway_steps_inward_nm: [" << static_breakaway_steps_inward_[0] << ", " << static_breakaway_steps_inward_[1] << ", " << static_breakaway_steps_inward_[2] << "]"
             << "\nstatic_breakaway_step_1_s: " << static_breakaway_step_1_s_
             << "\nstatic_breakaway_step_2_s: " << static_breakaway_step_2_s_
             << "\nstatic_breakaway_exp_start_outward_nm: " << static_breakaway_exp_start_outward_
             << "\nstatic_breakaway_exp_peak_outward_nm: " << static_breakaway_exp_peak_outward_
             << "\nstatic_breakaway_exp_start_inward_nm: " << static_breakaway_exp_start_inward_
             << "\nstatic_breakaway_exp_peak_inward_nm: " << static_breakaway_exp_peak_inward_
             << "\nstatic_breakaway_exp_tau_s: " << static_breakaway_exp_tau_s_
             << "\nstatic_breakaway_angle_mode: " << static_breakaway_angle_mode_
             << "\nstatic_breakaway_angle_blend_outward: " << static_breakaway_angle_blend_outward_
             << "\nstatic_breakaway_angle_blend_inward: " << static_breakaway_angle_blend_inward_
             << "\nstatic_breakaway_angle_min_deg: " << static_breakaway_angle_min_deg_
             << "\nstatic_breakaway_angle_max_deg: " << static_breakaway_angle_max_deg_
             << "\nstatic_breakaway_error_enable_rad: " << static_breakaway_error_enable_rad_
             << "\nstatic_breakaway_error_full_rad: " << static_breakaway_error_full_rad_
             << "\nstatic_breakaway_error_disable_rad: " << static_breakaway_error_disable_rad_
             << "\nstatic_breakaway_dwell_speed_rad_s: " << static_breakaway_dwell_speed_rad_s_
             << "\nstatic_breakaway_release_speed_rad_s: " << static_breakaway_release_speed_rad_s_
             << "\nstatic_breakaway_velocity_fade_rad_s: " << static_breakaway_velocity_fade_rad_s_
             << "\nstatic_breakaway_velocity_fade_power: " << static_breakaway_velocity_fade_power_
             << "\nstatic_breakaway_velocity_filter_alpha: " << static_breakaway_velocity_filter_alpha_
             << "\nstatic_breakaway_velocity_source: " << static_breakaway_velocity_source_
             << "\n";
  }
  void writeTrace() {
    if (!trace_) return;
    for (size_t i = 0; i < 4; ++i) {
      trace_ << now().seconds() << ',' << phaseName(phase_) << ',' << completed_repetitions_ + 1 << ','
             << kModuleNames[i] << ',' << commanded_[i] << ',' << hips_[i].motor << ',' << hips_[i].diff
             << ',' << actual(i) << ',' << hips_[i].torque << ',' << commanded_hip_ff_[i] << ','
             << raw_hip_velocity_trace_[i] << ',' << filtered_hip_velocity_[i] << ','
             << breakaway_velocity_used_trace_[i] << ',' << commanded_static_breakaway_ff_[i] << ','
             << breakaway_dwell_trace_[i] << ',' << breakaway_released_trace_[i] << ','
             << commanded_hub_mode_[i] << ',' << commanded_hub_velocity_rpm10_[i] << ','
             << commanded_hub_torque_[i] << ',' << hips_[i].error << '\n';
    }
  }

  bool armed_{false}, have_state_{false}; int repetitions_{3}, completed_repetitions_{0}; uint32_t sequence_{0};
  std::string command_topic_, state_topic_, run_dir_, strategy_name_, strategy_version_; std::vector<std::string> active_modules_;
  double state_a_deg_{0}, state_b_deg_{45}, startup_speed_{.1}, recovery_deg_{0}, recovery_speed_{.1}, recovery_rest_s_{1}, hip_speed_{.2}, kp_{360}, ki_{0}, kd_{5}, max_ff_{200}, start_hold_s_{2}, segment_hold_s_{1}, max_state_age_s_{.1}, max_error_rad_{.35}, max_torque_{400}, wheel_torque_outward_{0}, wheel_torque_inward_{0}, max_wheel_torque_{20}, hip_to_wheel_m_{.260}, wheel_radius_m_{.051}, wheel_rate_deadband_rad_s_{.02};
  double static_breakaway_step_1_s_{.10}, static_breakaway_step_2_s_{.30}, static_breakaway_exp_start_outward_{0}, static_breakaway_exp_peak_outward_{0}, static_breakaway_exp_start_inward_{0}, static_breakaway_exp_peak_inward_{0}, static_breakaway_exp_tau_s_{.20}, static_breakaway_error_enable_rad_{.02}, static_breakaway_error_full_rad_{.05}, static_breakaway_error_disable_rad_{.01}, static_breakaway_dwell_speed_rad_s_{.03}, static_breakaway_release_speed_rad_s_{.10}, static_breakaway_velocity_fade_rad_s_{.05}, static_breakaway_velocity_fade_power_{2}, static_breakaway_velocity_filter_alpha_{.20}, static_breakaway_angle_blend_outward_{0}, static_breakaway_angle_blend_inward_{0}, static_breakaway_angle_min_deg_{15}, static_breakaway_angle_max_deg_{90};
  std::string wheel_mode_, static_breakaway_policy_, static_breakaway_angle_mode_, static_breakaway_velocity_source_;
  std::vector<double> static_breakaway_steps_outward_, static_breakaway_steps_inward_;
  std::array<HipState, 4> hips_{};
  std::array<BreakawayState, 4> breakaway_{};
  std::array<double, 4> commanded_{}, move_start_{}, previous_target_{}, commanded_hub_velocity_rpm10_{}, commanded_hub_torque_{}, commanded_hip_ff_{}, commanded_static_breakaway_ff_{}, raw_hip_velocity_trace_{}, filtered_hip_velocity_{}, breakaway_velocity_used_trace_{}, breakaway_dwell_trace_{};
  std::array<int, 4> commanded_hub_mode_{}, breakaway_released_trace_{};
  Phase phase_{Phase::kWaitForState};
  rclcpp::Time phase_start_{0,0,RCL_ROS_TIME}, last_state_{0,0,RCL_ROS_TIME}, previous_target_time_{0,0,RCL_ROS_TIME};
  std::ofstream trace_;
  rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr publisher_; rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr subscriber_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CampaignRunner>()); rclcpp::shutdown(); return 0; }
