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
    strategy_version_ = declare_parameter<std::string>("strategy_version", "2.6.0");
    active_modules_ = declare_parameter<std::vector<std::string>>("active_modules", {"A", "B"});
    repetitions_ = declare_parameter<int>("repetitions", 3);
    motion_mode_ = declare_parameter<std::string>("motion_mode", "two_state_cycle");
    state_a_deg_ = declare_parameter<double>("state_a_deg", 0.0);
    state_b_deg_ = declare_parameter<double>("state_b_deg", 45.0);
    module_state_a_deg_ = declare_parameter<std::vector<double>>("module_state_a_deg", std::vector<double>());
    module_state_b_deg_ = declare_parameter<std::vector<double>>("module_state_b_deg", std::vector<double>());
    startup_speed_ = declare_parameter<double>("startup_move_speed_rad_s", 0.1);
    recovery_deg_ = declare_parameter<double>("recovery_position_deg", 0.0);
    recovery_speed_ = declare_parameter<double>("recovery_move_speed_rad_s", 0.1);
    recovery_rest_s_ = declare_parameter<double>("recovery_rest_s", 1.0);
    hip_speed_ = declare_parameter<double>("hip_speed_rad_s", 0.2);
    kp_ = declare_parameter<double>("kp", 360.0);
    ki_ = declare_parameter<double>("ki", 0.0);
    kd_ = declare_parameter<double>("kd", 5.0);
    max_ff_ = declare_parameter<double>("max_abs_hip_ff_torque_nm", 200.0);
    feedforward_mode_ = declare_parameter<std::string>("feedforward_mode", "static_breakaway");
    lift_assist_support_region_end_ = declare_parameter<std::vector<double>>(
        "lift_assist_support_region_end_rad", std::vector<double>{-kDegToRad, -kDegToRad, -kDegToRad, -kDegToRad});
    lift_assist_lift_region_start_ = declare_parameter<std::vector<double>>(
        "lift_assist_lift_region_start_rad", std::vector<double>{kDegToRad, kDegToRad, kDegToRad, kDegToRad});
    lift_assist_pid_schedule_enabled_ = declare_parameter<bool>("lift_assist_pid_schedule_enabled", false);
    lift_assist_support_kp_ = declare_parameter<double>("lift_assist_support_kp", kp_);
    lift_assist_support_ki_ = declare_parameter<double>("lift_assist_support_ki", ki_);
    lift_assist_support_kd_ = declare_parameter<double>("lift_assist_support_kd", kd_);
    lift_assist_lift_kp_ = declare_parameter<double>("lift_assist_lift_kp", kp_);
    lift_assist_lift_ki_ = declare_parameter<double>("lift_assist_lift_ki", ki_);
    lift_assist_lift_kd_ = declare_parameter<double>("lift_assist_lift_kd", kd_);
    lift_assist_pid_kp_rate_ = declare_parameter<double>("lift_assist_pid_kp_rate_per_s", 0.0);
    lift_assist_pid_ki_rate_ = declare_parameter<double>("lift_assist_pid_ki_rate_per_s", 0.0);
    lift_assist_pid_kd_rate_ = declare_parameter<double>("lift_assist_pid_kd_rate_per_s", 0.0);
    kp_to_lift_rate_ = declare_parameter<double>("kp_to_lift_rate_per_s", -1.0);
    kp_to_support_rate_ = declare_parameter<double>("kp_to_support_rate_per_s", -1.0);
    ki_to_lift_rate_ = declare_parameter<double>("ki_to_lift_rate_per_s", -1.0);
    ki_to_support_rate_ = declare_parameter<double>("ki_to_support_rate_per_s", -1.0);
    kd_to_lift_rate_ = declare_parameter<double>("kd_to_lift_rate_per_s", -1.0);
    kd_to_support_rate_ = declare_parameter<double>("kd_to_support_rate_per_s", -1.0);
    lift_assist_support_inward_ff_ = declare_parameter<double>("lift_assist_support_inward_ff_nm", 0.0);
    lift_assist_lift_start_inward_ff_ = declare_parameter<double>("lift_assist_lift_start_inward_ff_nm", 0.0);
    lift_assist_lift_ramp_ = declare_parameter<double>("lift_assist_lift_ramp_nm_s", 0.0);
    lift_assist_lift_ramp_up_ = declare_parameter<double>("lift_assist_lift_ramp_up_nm_s", -1.0);
    lift_assist_lift_ramp_down_ = declare_parameter<double>("lift_assist_lift_ramp_down_nm_s", -1.0);
    lift_assist_lift_max_inward_ff_ = declare_parameter<double>("lift_assist_lift_max_inward_ff_nm", 0.0);
    lift_assist_apply_rate_ = declare_parameter<double>("lift_assist_apply_rate_nm_s", 0.0);
    lift_assist_release_rate_ = declare_parameter<double>("lift_assist_release_rate_nm_s", 0.0);
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
    hub_travel_validation_enabled_ = declare_parameter<bool>("hub_travel_validation_enabled", true);
    hub_travel_ratio_min_ = declare_parameter<double>("hub_travel_ratio_min", 0.75);
    hub_travel_ratio_max_ = declare_parameter<double>("hub_travel_ratio_max", 1.25);
    hub_travel_min_command_rad_ = declare_parameter<double>("hub_travel_min_command_rad", 0.25);

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
    const bool lift_assist_mode = feedforward_mode_ == "angle_diff_lift_assist";
    const bool known_feedforward_mode = feedforward_mode_ == "static_breakaway" || lift_assist_mode;
    const bool per_module_motion = motion_mode_ == "per_module_two_state_cycle";
    const bool known_motion_mode = motion_mode_ == "two_state_cycle" || per_module_motion;
    if (repetitions_ < 1 || startup_speed_ <= 0.0 || recovery_speed_ <= 0.0 || hip_speed_ <= 0.0 ||
        !known_wheel_mode || !known_motion_mode || !known_feedforward_mode ||
        max_wheel_torque_ < 0.0 || hip_to_wheel_m_ <= 0.0 || wheel_radius_m_ <= 0.0 ||
        wheel_rate_deadband_rad_s_ < 0.0 || !known_breakaway_policy || !known_angle_mode || !known_velocity_source ||
        hub_travel_ratio_min_ <= 0.0 || hub_travel_ratio_max_ < hub_travel_ratio_min_ ||
        hub_travel_min_command_rad_ < 0.0 ||
        static_breakaway_steps_outward_.size() != 3 || static_breakaway_steps_inward_.size() != 3 ||
        static_breakaway_step_1_s_ < 0.0 || static_breakaway_step_2_s_ < static_breakaway_step_1_s_ ||
        static_breakaway_exp_tau_s_ <= 0.0 || static_breakaway_error_disable_rad_ < 0.0 ||
        static_breakaway_error_enable_rad_ < static_breakaway_error_disable_rad_ ||
        static_breakaway_error_full_rad_ < static_breakaway_error_enable_rad_ ||
        static_breakaway_dwell_speed_rad_s_ < 0.0 || static_breakaway_release_speed_rad_s_ < 0.0 ||
        static_breakaway_velocity_filter_alpha_ <= 0.0 || static_breakaway_velocity_filter_alpha_ > 1.0 ||
        static_breakaway_angle_blend_outward_ < 0.0 || static_breakaway_angle_blend_outward_ > 1.0 ||
        static_breakaway_angle_blend_inward_ < 0.0 || static_breakaway_angle_blend_inward_ > 1.0 ||
        static_breakaway_angle_min_deg_ < 0.0 || static_breakaway_angle_max_deg_ < static_breakaway_angle_min_deg_) {
      throw std::runtime_error("invalid trajectory, wheel-mode, or near-zero-breakaway setting");
    }
    if (per_module_motion &&
        (module_state_a_deg_.size() != kModuleNames.size() || module_state_b_deg_.size() != kModuleNames.size())) {
      throw std::runtime_error("per_module_two_state_cycle requires four module_state_a_deg and module_state_b_deg values in A,B,C,D order");
    }
    if (lift_assist_pid_schedule_enabled_ && !lift_assist_mode) {
      throw std::runtime_error("lift_assist_pid_schedule_enabled requires feedforward_mode=angle_diff_lift_assist");
    }
    if (lift_assist_mode &&
        (lift_assist_support_region_end_.size() != kModuleNames.size() ||
         lift_assist_lift_region_start_.size() != kModuleNames.size())) {
      throw std::runtime_error("angle_diff_lift_assist requires four lift-assist threshold values in A,B,C,D order");
    }
    if (lift_assist_mode) {
      if (lift_assist_lift_start_inward_ff_ < 0.0 || lift_assist_lift_ramp_ < 0.0 ||
          (lift_assist_lift_ramp_up_ < 0.0 && lift_assist_lift_ramp_up_ != -1.0) ||
          (lift_assist_lift_ramp_down_ < 0.0 && lift_assist_lift_ramp_down_ != -1.0) ||
          lift_assist_apply_rate_ < 0.0 || lift_assist_release_rate_ < 0.0 ||
          lift_assist_lift_max_inward_ff_ < 0.0 ||
          (lift_assist_lift_max_inward_ff_ > 0.0 &&
           lift_assist_lift_max_inward_ff_ < lift_assist_lift_start_inward_ff_)) {
        throw std::runtime_error("invalid angle_diff_lift_assist: lift start/rates and apply/release rates must be nonnegative; directional lift ramps may be -1 only for legacy fallback; lift maximum must be zero (use global FF clamp) or at least lift start");
      }
      if (lift_assist_pid_schedule_enabled_ &&
          (!std::isfinite(lift_assist_support_kp_) || !std::isfinite(lift_assist_support_ki_) ||
           !std::isfinite(lift_assist_support_kd_) || !std::isfinite(lift_assist_lift_kp_) ||
           !std::isfinite(lift_assist_lift_ki_) || !std::isfinite(lift_assist_lift_kd_) ||
           !std::isfinite(lift_assist_pid_kp_rate_) || !std::isfinite(lift_assist_pid_ki_rate_) ||
           !std::isfinite(lift_assist_pid_kd_rate_) || !std::isfinite(kp_to_lift_rate_) ||
           !std::isfinite(kp_to_support_rate_) || !std::isfinite(ki_to_lift_rate_) ||
           !std::isfinite(ki_to_support_rate_) || !std::isfinite(kd_to_lift_rate_) ||
           !std::isfinite(kd_to_support_rate_) ||
           lift_assist_support_kp_ < 0.0 || lift_assist_support_ki_ < 0.0 || lift_assist_support_kd_ < 0.0 ||
           lift_assist_lift_kp_ < 0.0 || lift_assist_lift_ki_ < 0.0 || lift_assist_lift_kd_ < 0.0 ||
           lift_assist_pid_kp_rate_ < 0.0 || lift_assist_pid_ki_rate_ < 0.0 || lift_assist_pid_kd_rate_ < 0.0 ||
           (kp_to_lift_rate_ < 0.0 && kp_to_lift_rate_ != -1.0) ||
           (kp_to_support_rate_ < 0.0 && kp_to_support_rate_ != -1.0) ||
           (ki_to_lift_rate_ < 0.0 && ki_to_lift_rate_ != -1.0) ||
           (ki_to_support_rate_ < 0.0 && ki_to_support_rate_ != -1.0) ||
           (kd_to_lift_rate_ < 0.0 && kd_to_lift_rate_ != -1.0) ||
           (kd_to_support_rate_ < 0.0 && kd_to_support_rate_ != -1.0))) {
        throw std::runtime_error("invalid angle_diff_lift_assist PID schedule: gains/rates must be finite; directional rates must be nonnegative or -1 for legacy symmetric fallback");
      }
      for (size_t i = 0; i < kModuleNames.size(); ++i) {
        if (!std::isfinite(lift_assist_support_region_end_[i]) || !std::isfinite(lift_assist_lift_region_start_[i]) ||
            lift_assist_support_region_end_[i] >= 0.0 || lift_assist_lift_region_start_[i] <= 0.0 ||
            lift_assist_support_region_end_[i] >= lift_assist_lift_region_start_[i]) {
          throw std::runtime_error("lift-assist support-region end must be negative and lift-region start positive for every module");
        }
      }
    }
    for (const auto &name : active_modules_) {
      if (std::find(kModuleNames.begin(), kModuleNames.end(), name) == kModuleNames.end()) throw std::runtime_error("unknown active module");
    }
    publisher_ = create_publisher<kilin_msgs::msg::MotorCmdStamped>(command_topic_, 10);
    subscriber_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(state_topic_, rclcpp::QoS(20).best_effort(), std::bind(&CampaignRunner::onState, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&CampaignRunner::tick, this));
    RCLCPP_INFO(get_logger(), "Runner ready: strategy=%s v%s motion_mode=%s feedforward_mode=%s armed=%s command_topic=%s wheel_mode=%s", strategy_name_.c_str(), strategy_version_.c_str(), motion_mode_.c_str(), feedforward_mode_.c_str(), armed_ ? "true" : "false", command_topic_.c_str(), wheel_mode_.c_str());
  }

 private:
  struct HipState { double motor{NAN}; double diff{NAN}; double velocity{NAN}; double torque{NAN}; int error{0}; };
  struct HubState { double position{NAN}; double velocity{NAN}; double torque{NAN}; int mode{0}; int error{0}; };
  struct HubTravelState {
    bool active{false};
    double start_position{NAN};
    double commanded_travel_rad{0.0};
    double feedback_travel_rad{NAN};
    double ratio{NAN};
    int valid{-1};  // -1 pending/not applicable, 0 invalid, 1 valid.
  };
  struct BreakawayState {
    double dwell_s{0.0};
    double last_update_s{0.0};
    int direction_sign{0};
    bool error_enabled{false};
    bool released{false};
  };
  struct LiftAssistState {
    double dwell_s{0.0};
    double last_update_s{0.0};
    double held_lift_inward_ff{0.0};
    double applied_inward_ff{0.0};
    bool lift_latched{false};
  };

  bool active(size_t i) const { return std::find(active_modules_.begin(), active_modules_.end(), kModuleNames[i]) != active_modules_.end(); }
  double actual(size_t i) const { return hips_[i].motor + hips_[i].diff; }
  double sign(size_t i) const { return i < 2 ? -1.0 : 1.0; }
  double outwardMagnitude(size_t i, double motor_position_rad) const { return sign(i) * motor_position_rad; }
  bool inLiftAssistOutwardRange(size_t i, double motor_position_rad) const {
    return std::isfinite(motor_position_rad) && outwardMagnitude(i, motor_position_rad) >= 0.0 &&
           outwardMagnitude(i, motor_position_rad) <= M_PI / 2.0;
  }
  double trimLiftAssistTarget(size_t i, double target_rad) const {
    if (feedforward_mode_ != "angle_diff_lift_assist") return target_rad;
    return sign(i) * std::clamp(outwardMagnitude(i, target_rad), 0.0, M_PI / 2.0);
  }
  double normalizedLiftDifference(size_t i) const { return -sign(i) * hips_[i].diff; }
  double stateA(size_t i) const {
    const double magnitude_deg = motion_mode_ == "per_module_two_state_cycle" ? module_state_a_deg_[i] : state_a_deg_;
    return active(i) ? sign(i) * magnitude_deg * kDegToRad : 0.0;
  }
  double stateB(size_t i) const {
    const double magnitude_deg = motion_mode_ == "per_module_two_state_cycle" ? module_state_b_deg_[i] : state_b_deg_;
    return active(i) ? sign(i) * magnitude_deg * kDegToRad : 0.0;
  }
  double recovery(size_t i) const { return active(i) ? sign(i) * recovery_deg_ * kDegToRad : 0.0; }
  bool freshState() const { return have_state_ && (now() - last_state_).seconds() <= max_state_age_s_; }
  double elapsed() const { return (now() - phase_start_).seconds(); }
  static double clamp01(double x) { return std::clamp(x, 0.0, 1.0); }
  static double smooth(double x) { x = clamp01(x); return x * x * (3.0 - 2.0 * x); }

  void onState(const kilin_msgs::msg::MotorStateStamped::SharedPtr message) {
    const std::array<kilin_msgs::msg::LegState, 4> legs{message->module_a, message->module_b, message->module_c, message->module_d};
    for (size_t i = 0; i < 4; ++i) {
      hips_[i] = {legs[i].hip.position, legs[i].hip.position_diff, legs[i].hip.velocity, legs[i].hip.torque, legs[i].hip.error_code};
      hubs_[i] = {legs[i].hub.position, legs[i].hub.velocity, legs[i].hub.torque, legs[i].hub.motor_mode, legs[i].hub.error_code};
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
      if (wheel_mode_ == "speed_ik" && !std::isfinite(hubs_[i].position)) {
        throw std::runtime_error("non-finite hub-position feedback required for speed-IK validation");
      }
      move_start_[i] = hips_[i].motor;
      commanded_[i] = hips_[i].motor;
      previous_target_[i] = hips_[i].motor;
    }
    previous_target_time_ = now();
    openEvidence();
    setPhase(Phase::kStartupMove, "moving to configured state A with wheels rest and zero FF");
  }

  void setPhase(Phase next, const std::string &detail) {
    if (wheelMotionPhase() && next != phase_) finalizeHubTravelForPhase();
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
  bool normalTestPhase() const {
    return phase_ == Phase::kStartHold || phase_ == Phase::kMoveToB ||
           phase_ == Phase::kHoldAtB || phase_ == Phase::kMoveToA;
  }
  bool feedforwardAllowedInPhase() const {
    return feedforward_mode_ == "angle_diff_lift_assist" ? normalTestPhase() : wheelMotionPhase();
  }
  struct ScheduledPid { double kp; double ki; double kd; double blend; };
  struct AppliedScheduledPid { double kp; double ki; double kd; double last_target_blend; bool initialized; };
  ScheduledPid scheduledPid(size_t i, double target_rad) const {
    ScheduledPid result{kp_, ki_, kd_, 0.0};
    if (!lift_assist_pid_schedule_enabled_ || feedforward_mode_ != "angle_diff_lift_assist" ||
        !normalTestPhase() || !inLiftAssistOutwardRange(i, target_rad) ||
        !inLiftAssistOutwardRange(i, hips_[i].motor) || !std::isfinite(hips_[i].diff)) {
      return result;
    }
    const double support_end = lift_assist_support_region_end_[i];
    const double lift_start = lift_assist_lift_region_start_[i];
    result.blend = clamp01((normalizedLiftDifference(i) - support_end) / (lift_start - support_end));
    result.kp = (1.0 - result.blend) * lift_assist_support_kp_ + result.blend * lift_assist_lift_kp_;
    result.ki = (1.0 - result.blend) * lift_assist_support_ki_ + result.blend * lift_assist_lift_ki_;
    result.kd = (1.0 - result.blend) * lift_assist_support_kd_ + result.blend * lift_assist_lift_kd_;
    return result;
  }

  static double rateLimit(double current, double target, double transition_rate_per_s, double dt_s) {
    const double rate_per_s = transition_rate_per_s;
    if (rate_per_s <= 0.0) return target;
    const double step = rate_per_s * dt_s;
    return current + std::clamp(target - current, -step, step);
  }

  static double directionalRate(double directional_rate, double legacy_symmetric_rate) {
    return directional_rate >= 0.0 ? directional_rate : legacy_symmetric_rate;
  }

  ScheduledPid applyScheduledPidRateLimit(size_t i, const ScheduledPid &target, double dt_s) {
    const bool schedule_active = lift_assist_pid_schedule_enabled_ &&
                                 feedforward_mode_ == "angle_diff_lift_assist" && normalTestPhase();
    auto &applied = applied_scheduled_pid_[i];
    if (!schedule_active) {
      applied.initialized = false;
      return target;
    }
    if (!applied.initialized) {
      // Always begin a normal test phase at global gains.  A nonzero limit then
      // makes entry to the support/lift schedule gradual as well.
      applied = AppliedScheduledPid{kp_, ki_, kd_, 0.0, true};
    }
    const bool support_to_lift = target.blend >= applied.last_target_blend;
    applied.kp = rateLimit(applied.kp, target.kp,
                           directionalRate(support_to_lift ? kp_to_lift_rate_ : kp_to_support_rate_, lift_assist_pid_kp_rate_), dt_s);
    applied.ki = rateLimit(applied.ki, target.ki,
                           directionalRate(support_to_lift ? ki_to_lift_rate_ : ki_to_support_rate_, lift_assist_pid_ki_rate_), dt_s);
    applied.kd = rateLimit(applied.kd, target.kd,
                           directionalRate(support_to_lift ? kd_to_lift_rate_ : kd_to_support_rate_, lift_assist_pid_kd_rate_), dt_s);
    applied.last_target_blend = target.blend;
    return ScheduledPid{applied.kp, applied.ki, applied.kd, target.blend};
  }

  double wheelRateRadS(double hip_rate_rad_s, double commanded_hip_rad) const {
    return -hip_to_wheel_m_ * std::cos(commanded_hip_rad) * hip_rate_rad_s / wheel_radius_m_;
  }

  bool usesHubTravelValidation() const {
    return hub_travel_validation_enabled_ && wheel_mode_ == "speed_ik";
  }

  void beginHubTravel(size_t i) {
    auto &travel = hub_travel_[i];
    if (travel.active) return;
    travel = HubTravelState{};
    travel.active = true;
    travel.start_position = hubs_[i].position;
  }

  void updateHubTravel(size_t i, double wheel_rate_rad_s, double dt_s) {
    if (!usesHubTravelValidation()) return;
    beginHubTravel(i);
    auto &travel = hub_travel_[i];
    travel.commanded_travel_rad += wheel_rate_rad_s * dt_s;
    travel.feedback_travel_rad = hubs_[i].position - travel.start_position;
    if (std::abs(travel.commanded_travel_rad) >= hub_travel_min_command_rad_) {
      travel.ratio = travel.feedback_travel_rad / travel.commanded_travel_rad;
    }
  }

  void finalizeHubTravel(size_t i) {
    auto &travel = hub_travel_[i];
    if (!travel.active) return;
    travel.feedback_travel_rad = hubs_[i].position - travel.start_position;
    const bool enough_command = std::abs(travel.commanded_travel_rad) >= hub_travel_min_command_rad_;
    if (!usesHubTravelValidation() || !enough_command || !std::isfinite(travel.feedback_travel_rad)) {
      travel.valid = -1;
    } else {
      travel.ratio = travel.feedback_travel_rad / travel.commanded_travel_rad;
      travel.valid = travel.ratio >= hub_travel_ratio_min_ && travel.ratio <= hub_travel_ratio_max_ ? 1 : 0;
    }

    if (hub_travel_summary_) {
      hub_travel_summary_ << phaseName(phase_) << ',' << completed_repetitions_ + 1 << ',' << kModuleNames[i] << ','
                          << travel.start_position << ',' << hubs_[i].position << ','
                          << travel.commanded_travel_rad << ',' << travel.feedback_travel_rad << ','
                          << travel.ratio << ',' << travel.valid << ',' << hubs_[i].torque << ','
                          << hubs_[i].velocity << ',' << hubs_[i].error << '\n';
    }
    if (travel.valid == 0) {
      ++invalid_hub_travel_segments_;
      RCLCPP_WARN(get_logger(),
                  "Hub travel INVALID phase=%s trial=%d module=%s command_rad=%.4f feedback_rad=%.4f ratio=%.3f allowed=[%.2f,%.2f]",
                  phaseName(phase_), completed_repetitions_ + 1, kModuleNames[i], travel.commanded_travel_rad,
                  travel.feedback_travel_rad, travel.ratio, hub_travel_ratio_min_, hub_travel_ratio_max_);
    }
    travel.active = false;
  }

  void finalizeHubTravelForPhase() {
    if (!wheelMotionPhase()) return;
    for (size_t i = 0; i < 4; ++i) finalizeHubTravel(i);
  }

  void commandHub(kilin_msgs::msg::LegCmd &leg, size_t i, double hip_rate_rad_s,
                  double commanded_hip_rad, double dt_s) {
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
      updateHubTravel(i, wheel_rate, dt_s);
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
      const double safe_target = trimLiftAssistTarget(i, target[i]);
      const double hip_rate = (safe_target - previous_target_[i]) / dt_s;
      commanded_[i] = safe_target;
      legs[i].hip.motor_mode = kPosition;
      // Do not compensate the motor command with position_diff.  It is kept in
      // the state/logging path as the reconstructed actual hip angle, but it is
      // not part of this low-level position-control loop.
      legs[i].hip.position = safe_target;
      const ScheduledPid pid_target = scheduledPid(i, safe_target);
      const ScheduledPid pid = applyScheduledPidRateLimit(i, pid_target, dt_s);
      legs[i].hip.kp = pid.kp; legs[i].hip.ki = pid.ki; legs[i].hip.kd = pid.kd;
      lift_assist_pid_blend_trace_[i] = pid.blend;
      scheduled_target_kp_trace_[i] = pid_target.kp;
      scheduled_target_ki_trace_[i] = pid_target.ki;
      scheduled_target_kd_trace_[i] = pid_target.kd;
      scheduled_kp_trace_[i] = pid.kp;
      scheduled_ki_trace_[i] = pid.ki;
      scheduled_kd_trace_[i] = pid.kd;
      raw_hip_velocity_trace_[i] = hips_[i].velocity;
      breakaway_velocity_used_trace_[i] = breakawayVelocity(i);
      commanded_hip_ff_[i] = allow_ff ? feedforward(i, safe_target - hips_[i].motor) : 0.0;
      if (!allow_ff) {
        commanded_static_breakaway_ff_[i] = 0.0;
        breakaway_dwell_trace_[i] = 0.0;
        breakaway_released_trace_[i] = 0;
        resetLiftAssist(i);
      }
      legs[i].hip.torque = commanded_hip_ff_[i];
      legs[i].steering.motor_mode = kPosition;
      legs[i].steering.position = 0.0;
      commandHub(legs[i], i, hip_rate, safe_target, dt_s);
      previous_target_[i] = safe_target;
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

    // Preserve the enable/full/disable ramp, but use the direct linear slope.
    // The shared smooth() helper remains only for trajectory interpolation.
    const double error_gate = static_breakaway_error_full_rad_ == static_breakaway_error_enable_rad_
        ? 1.0
        : clamp01((error_abs - static_breakaway_error_enable_rad_) /
                  (static_breakaway_error_full_rad_ - static_breakaway_error_enable_rad_));
    const double amplitude = breakawayBaseAmplitude(outward, state.dwell_s) *
        error_gate * breakawayAngleFactor(i, outward);
    const double command = std::clamp(std::copysign(1.0, error) * amplitude, -max_ff_, max_ff_);
    commanded_static_breakaway_ff_[i] = command;
    breakaway_dwell_trace_[i] = state.dwell_s;
    return command;
  }

  void clearLiftAssistTrace(size_t i) {
    lift_assist_normalized_diff_trace_[i] = NAN;
    lift_assist_target_inward_ff_trace_[i] = 0.0;
    lift_assist_physical_inward_ff_trace_[i] = 0.0;
    lift_assist_dwell_trace_[i] = 0.0;
    lift_assist_active_trace_[i] = 0;
    lift_assist_latched_trace_[i] = 0;
  }

  void resetLiftAssist(size_t i) {
    lift_assist_[i] = LiftAssistState{};
    clearLiftAssistTrace(i);
  }

  double angleDiffLiftAssistFF(size_t i) {
    clearLiftAssistTrace(i);
    if (!active(i) || !inLiftAssistOutwardRange(i, commanded_[i]) ||
        !inLiftAssistOutwardRange(i, hips_[i].motor) || !std::isfinite(hips_[i].diff)) {
      lift_assist_[i] = LiftAssistState{};
      return 0.0;
    }

    const double normalized_diff = normalizedLiftDifference(i);
    lift_assist_normalized_diff_trace_[i] = normalized_diff;
    lift_assist_active_trace_[i] = 1;
    const double support_end = lift_assist_support_region_end_[i];
    const double lift_start = lift_assist_lift_region_start_[i];
    auto &state = lift_assist_[i];
    const double time_s = now().seconds();
    const double dt_s = state.last_update_s == 0.0 ? 0.0 : std::clamp(time_s - state.last_update_s, 0.0, 0.05);
    state.last_update_s = time_s;
    const double policy_lift_cap = lift_assist_lift_max_inward_ff_ > 0.0
        ? lift_assist_lift_max_inward_ff_ : max_ff_;
    const double lift_ramp_up = directionalRate(lift_assist_lift_ramp_up_, lift_assist_lift_ramp_);

    if (normalized_diff >= lift_start) {
      if (!state.lift_latched) {
        state.lift_latched = true;
        state.dwell_s = 0.0;
        state.held_lift_inward_ff = std::clamp(state.held_lift_inward_ff,
                                                lift_assist_lift_start_inward_ff_, policy_lift_cap);
      }
      state.dwell_s += dt_s;
      state.held_lift_inward_ff = std::min(policy_lift_cap,
                                            state.held_lift_inward_ff + lift_ramp_up * dt_s);
    } else if (normalized_diff <= support_end) {
      // The two existing thresholds form the lift latch. A transient crossing
      // below the lift-entry boundary cannot reset accumulated lift strength.
      state.lift_latched = false;
      state.dwell_s = 0.0;
      if (lift_assist_lift_ramp_down_ < 0.0) {
        // Legacy profiles reset stored lift strength immediately on support.
        state.held_lift_inward_ff = lift_assist_lift_start_inward_ff_;
      } else {
        state.held_lift_inward_ff = std::max(lift_assist_lift_start_inward_ff_,
            state.held_lift_inward_ff - lift_assist_lift_ramp_down_ * dt_s);
      }
    }

    const double lift_target = state.lift_latched ? state.held_lift_inward_ff : lift_assist_lift_start_inward_ff_;
    double target_inward_ff = lift_assist_support_inward_ff_;
    if (normalized_diff >= lift_start) {
      target_inward_ff = lift_target;
    } else if (normalized_diff > support_end) {
      const double blend = (normalized_diff - support_end) / (lift_start - support_end);
      target_inward_ff = (1.0 - blend) * lift_assist_support_inward_ff_ + blend * lift_target;
    }

    if (target_inward_ff > state.applied_inward_ff && lift_assist_apply_rate_ > 0.0) {
      state.applied_inward_ff = std::min(target_inward_ff, state.applied_inward_ff + lift_assist_apply_rate_ * dt_s);
    } else if (target_inward_ff < state.applied_inward_ff && lift_assist_release_rate_ > 0.0) {
      state.applied_inward_ff = std::max(target_inward_ff, state.applied_inward_ff - lift_assist_release_rate_ * dt_s);
    } else {
      state.applied_inward_ff = target_inward_ff;
    }

    lift_assist_target_inward_ff_trace_[i] = target_inward_ff;
    lift_assist_physical_inward_ff_trace_[i] = state.applied_inward_ff;
    lift_assist_dwell_trace_[i] = state.dwell_s;
    lift_assist_latched_trace_[i] = state.lift_latched ? 1 : 0;
    // Physical inward is positive in profile space. A/B motors are positive
    // inward; C/D motors are negative inward under the established convention.
    return std::clamp(-sign(i) * state.applied_inward_ff, -max_ff_, max_ff_);
  }

  double feedforward(size_t i, double error) {
    commanded_static_breakaway_ff_[i] = 0.0;
    breakaway_dwell_trace_[i] = 0.0;
    breakaway_released_trace_[i] = 0;
    if (!active(i)) return 0.0;
    if (feedforward_mode_ == "angle_diff_lift_assist") return angleDiffLiftAssistFF(i);
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
        commandPosition(a, feedforwardAllowedInPhase());
        if (elapsed() >= start_hold_s_) setPhase(Phase::kMoveToB, "beginning dynamic move to state B");
        break;
      case Phase::kMoveToB:
        commandPosition(interpolated(a, b, clamp01(elapsed() / fullStrokeDuration())), feedforwardAllowedInPhase());
        if (elapsed() >= fullStrokeDuration()) setPhase(Phase::kHoldAtB, "state B reached");
        break;
      case Phase::kHoldAtB:
        commandPosition(b, feedforwardAllowedInPhase());
        if (elapsed() >= segment_hold_s_) setPhase(Phase::kMoveToA, "returning to state A");
        break;
      case Phase::kMoveToA:
        commandPosition(interpolated(b, a, clamp01(elapsed() / fullStrokeDuration())), feedforwardAllowedInPhase());
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
  void complete() {
    publishRest();
    setPhase(Phase::kComplete, "all repetitions and recovery complete");
    if (invalid_hub_travel_segments_ > 0) {
      RCLCPP_WARN(get_logger(), "Campaign complete with %d invalid speed-IK hub-travel segments; exclude this unit from wheel-condition analysis.", invalid_hub_travel_segments_);
    } else {
      RCLCPP_INFO(get_logger(), "Campaign complete with all speed-IK hub-travel segments valid.");
    }
    rclcpp::shutdown();
  }
  void stamp(kilin_msgs::msg::MotorCmdStamped &message) { const auto t = now(); message.header.seq = sequence_++; message.header.time.sec = static_cast<int32_t>(t.seconds()); message.header.time.nanosec = static_cast<uint32_t>(t.nanoseconds() % 1000000000LL); message.header.frame_id = "kilin_hip_characterization"; }
  void publishRest() { kilin_msgs::msg::MotorCmdStamped message; stamp(message); kilin_msgs::msg::LegCmd leg; leg.hip.motor_mode=kRest; leg.steering.motor_mode=kRest; leg.hub.motor_mode=kRest; message.module_a=leg;message.module_b=leg;message.module_c=leg;message.module_d=leg;publisher_->publish(message); }
  void openEvidence() {
    if (run_dir_.empty()) return;
    fs::create_directories(run_dir_);
    trace_.open(fs::path(run_dir_) / "command_state_trace.csv");
    trace_ << "time_s,phase,trial,module,commanded_motor_rad,motor_position_rad,position_diff_rad,actual_hip_angle_rad,hip_torque,commanded_hip_ff,raw_hip_velocity_rad_s,filtered_hip_velocity_rad_s,breakaway_velocity_used_rad_s,static_breakaway_ff,static_breakaway_dwell_s,static_breakaway_released,lift_assist_normalized_diff_rad,lift_assist_target_inward_ff,lift_assist_physical_inward_ff,lift_assist_dwell_s,lift_assist_active,lift_assist_latched,lift_assist_pid_blend,scheduled_target_kp,scheduled_target_ki,scheduled_target_kd,scheduled_kp,scheduled_ki,scheduled_kd,hub_mode,commanded_hub_velocity_rpm10,commanded_hub_torque,hub_position_rad,hub_velocity_feedback,hub_torque_feedback,hub_feedback_mode,hub_error_code,hub_travel_command_rad,hub_travel_feedback_rad,hub_travel_ratio,hub_travel_valid,error_code\n";
    hub_travel_summary_.open(fs::path(run_dir_) / "hub_travel_summary.csv");
    hub_travel_summary_ << "phase,trial,module,start_hub_position_rad,end_hub_position_rad,commanded_travel_rad,feedback_travel_rad,travel_ratio,valid,final_hub_torque,final_hub_velocity,hub_error_code\n";
    std::ofstream manifest(fs::path(run_dir_) / "trial_manifest.yaml");
    manifest << "strategy_name: " << strategy_name_ << "\nstrategy_version: " << strategy_version_
             << "\nangle_convention: actual_hip_angle_rad = motor_position + position_diff"
             << "\ncontrol_angle_reference: motor_position (position_diff is measurement-only)"
             << "\nwheel_mode: " << wheel_mode_
             << "\nhub_travel_validation_enabled: " << (hub_travel_validation_enabled_ ? "true" : "false")
             << "\nhub_travel_ratio_min: " << hub_travel_ratio_min_
             << "\nhub_travel_ratio_max: " << hub_travel_ratio_max_
             << "\nhub_travel_min_command_rad: " << hub_travel_min_command_rad_
             << "\nwheel_torque_outward_nm: " << wheel_torque_outward_
             << "\nwheel_torque_inward_nm: " << wheel_torque_inward_
             << "\nmax_abs_hub_torque_nm: " << max_wheel_torque_
             << "\nfeedforward_mode: " << feedforward_mode_
             << "\nlift_assist_support_region_end_rad: [" << lift_assist_support_region_end_[0] << ", " << lift_assist_support_region_end_[1] << ", " << lift_assist_support_region_end_[2] << ", " << lift_assist_support_region_end_[3] << "]"
             << "\nlift_assist_lift_region_start_rad: [" << lift_assist_lift_region_start_[0] << ", " << lift_assist_lift_region_start_[1] << ", " << lift_assist_lift_region_start_[2] << ", " << lift_assist_lift_region_start_[3] << "]"
             << "\nlift_assist_support_inward_ff_nm: " << lift_assist_support_inward_ff_
             << "\nlift_assist_lift_start_inward_ff_nm: " << lift_assist_lift_start_inward_ff_
             << "\nlift_assist_lift_ramp_nm_s: " << lift_assist_lift_ramp_
             << "\nlift_assist_lift_ramp_up_nm_s: " << lift_assist_lift_ramp_up_
             << "\nlift_assist_lift_ramp_down_nm_s: " << lift_assist_lift_ramp_down_
             << "\nlift_assist_lift_max_inward_ff_nm: " << lift_assist_lift_max_inward_ff_
             << "\nlift_assist_apply_rate_nm_s: " << lift_assist_apply_rate_
             << "\nlift_assist_release_rate_nm_s: " << lift_assist_release_rate_
             << "\nlift_assist_pid_schedule_enabled: " << (lift_assist_pid_schedule_enabled_ ? "true" : "false")
             << "\nlift_assist_support_kp: " << lift_assist_support_kp_
             << "\nlift_assist_support_ki: " << lift_assist_support_ki_
             << "\nlift_assist_support_kd: " << lift_assist_support_kd_
             << "\nlift_assist_lift_kp: " << lift_assist_lift_kp_
             << "\nlift_assist_lift_ki: " << lift_assist_lift_ki_
             << "\nlift_assist_lift_kd: " << lift_assist_lift_kd_
             << "\nlift_assist_pid_kp_rate_per_s: " << lift_assist_pid_kp_rate_
             << "\nlift_assist_pid_ki_rate_per_s: " << lift_assist_pid_ki_rate_
             << "\nlift_assist_pid_kd_rate_per_s: " << lift_assist_pid_kd_rate_
             << "\nkp_to_lift_rate_per_s: " << kp_to_lift_rate_
             << "\nkp_to_support_rate_per_s: " << kp_to_support_rate_
             << "\nki_to_lift_rate_per_s: " << ki_to_lift_rate_
             << "\nki_to_support_rate_per_s: " << ki_to_support_rate_
             << "\nkd_to_lift_rate_per_s: " << kd_to_lift_rate_
             << "\nkd_to_support_rate_per_s: " << kd_to_support_rate_
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
             << lift_assist_normalized_diff_trace_[i] << ',' << lift_assist_target_inward_ff_trace_[i] << ','
             << lift_assist_physical_inward_ff_trace_[i] << ',' << lift_assist_dwell_trace_[i] << ','
             << lift_assist_active_trace_[i] << ',' << lift_assist_latched_trace_[i] << ','
             << lift_assist_pid_blend_trace_[i] << ',' << scheduled_target_kp_trace_[i] << ','
             << scheduled_target_ki_trace_[i] << ',' << scheduled_target_kd_trace_[i] << ','
             << scheduled_kp_trace_[i] << ',' << scheduled_ki_trace_[i] << ',' << scheduled_kd_trace_[i] << ','
             << commanded_hub_mode_[i] << ',' << commanded_hub_velocity_rpm10_[i] << ','
             << commanded_hub_torque_[i] << ',' << hubs_[i].position << ',' << hubs_[i].velocity << ','
             << hubs_[i].torque << ',' << hubs_[i].mode << ',' << hubs_[i].error << ','
             << hub_travel_[i].commanded_travel_rad << ',' << hub_travel_[i].feedback_travel_rad << ','
             << hub_travel_[i].ratio << ',' << hub_travel_[i].valid << ',' << hips_[i].error << '\n';
    }
  }

  bool armed_{false}, have_state_{false}; int repetitions_{3}, completed_repetitions_{0}; uint32_t sequence_{0};
  std::string command_topic_, state_topic_, run_dir_, strategy_name_, strategy_version_, motion_mode_; std::vector<std::string> active_modules_;
  double state_a_deg_{0}, state_b_deg_{45}, startup_speed_{.1}, recovery_deg_{0}, recovery_speed_{.1}, recovery_rest_s_{1}, hip_speed_{.2}, kp_{360}, ki_{0}, kd_{5}, max_ff_{200}, start_hold_s_{2}, segment_hold_s_{1}, max_state_age_s_{.1}, max_error_rad_{.35}, max_torque_{400}, wheel_torque_outward_{0}, wheel_torque_inward_{0}, max_wheel_torque_{20}, hip_to_wheel_m_{.260}, wheel_radius_m_{.051}, wheel_rate_deadband_rad_s_{.02}, hub_travel_ratio_min_{.75}, hub_travel_ratio_max_{1.25}, hub_travel_min_command_rad_{.25};
  double static_breakaway_step_1_s_{.10}, static_breakaway_step_2_s_{.30}, static_breakaway_exp_start_outward_{0}, static_breakaway_exp_peak_outward_{0}, static_breakaway_exp_start_inward_{0}, static_breakaway_exp_peak_inward_{0}, static_breakaway_exp_tau_s_{.20}, static_breakaway_error_enable_rad_{.02}, static_breakaway_error_full_rad_{.05}, static_breakaway_error_disable_rad_{.01}, static_breakaway_dwell_speed_rad_s_{.03}, static_breakaway_release_speed_rad_s_{.10}, static_breakaway_velocity_filter_alpha_{.20}, static_breakaway_angle_blend_outward_{0}, static_breakaway_angle_blend_inward_{0}, static_breakaway_angle_min_deg_{15}, static_breakaway_angle_max_deg_{90};
  std::string wheel_mode_, feedforward_mode_, static_breakaway_policy_, static_breakaway_angle_mode_, static_breakaway_velocity_source_;
  std::vector<double> module_state_a_deg_, module_state_b_deg_, lift_assist_support_region_end_, lift_assist_lift_region_start_, static_breakaway_steps_outward_, static_breakaway_steps_inward_;
  std::array<HipState, 4> hips_{};
  std::array<HubState, 4> hubs_{};
  std::array<BreakawayState, 4> breakaway_{};
  std::array<LiftAssistState, 4> lift_assist_{};
  std::array<AppliedScheduledPid, 4> applied_scheduled_pid_{};
  std::array<HubTravelState, 4> hub_travel_{};
  std::array<double, 4> commanded_{}, move_start_{}, previous_target_{}, commanded_hub_velocity_rpm10_{}, commanded_hub_torque_{}, commanded_hip_ff_{}, commanded_static_breakaway_ff_{}, raw_hip_velocity_trace_{}, filtered_hip_velocity_{}, breakaway_velocity_used_trace_{}, breakaway_dwell_trace_{}, lift_assist_normalized_diff_trace_{}, lift_assist_target_inward_ff_trace_{}, lift_assist_physical_inward_ff_trace_{}, lift_assist_dwell_trace_{}, lift_assist_pid_blend_trace_{}, scheduled_target_kp_trace_{}, scheduled_target_ki_trace_{}, scheduled_target_kd_trace_{}, scheduled_kp_trace_{}, scheduled_ki_trace_{}, scheduled_kd_trace_{};
  std::array<int, 4> commanded_hub_mode_{}, breakaway_released_trace_{}, lift_assist_active_trace_{}, lift_assist_latched_trace_{};
  double lift_assist_support_inward_ff_{0.0}, lift_assist_lift_start_inward_ff_{0.0}, lift_assist_lift_ramp_{0.0}, lift_assist_lift_ramp_up_{-1.0}, lift_assist_lift_ramp_down_{-1.0}, lift_assist_lift_max_inward_ff_{0.0}, lift_assist_apply_rate_{0.0}, lift_assist_release_rate_{0.0};
  bool lift_assist_pid_schedule_enabled_{false};
  double lift_assist_support_kp_{0.0}, lift_assist_support_ki_{0.0}, lift_assist_support_kd_{0.0}, lift_assist_lift_kp_{0.0}, lift_assist_lift_ki_{0.0}, lift_assist_lift_kd_{0.0}, lift_assist_pid_kp_rate_{0.0}, lift_assist_pid_ki_rate_{0.0}, lift_assist_pid_kd_rate_{0.0}, kp_to_lift_rate_{-1.0}, kp_to_support_rate_{-1.0}, ki_to_lift_rate_{-1.0}, ki_to_support_rate_{-1.0}, kd_to_lift_rate_{-1.0}, kd_to_support_rate_{-1.0};
  bool hub_travel_validation_enabled_{true};
  int invalid_hub_travel_segments_{0};
  Phase phase_{Phase::kWaitForState};
  rclcpp::Time phase_start_{0,0,RCL_ROS_TIME}, last_state_{0,0,RCL_ROS_TIME}, previous_target_time_{0,0,RCL_ROS_TIME};
  std::ofstream trace_, hub_travel_summary_;
  rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr publisher_; rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr subscriber_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CampaignRunner>()); rclcpp::shutdown(); return 0; }
