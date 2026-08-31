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
constexpr int kRest = 0, kPosition = 4, kBrake = 7;
constexpr double kDegToRad = M_PI / 180.0;
constexpr std::array<const char *, 4> kModuleNames{"A", "B", "C", "D"};

enum class Phase { kWaitForState, kStartupMove, kStartHold, kStaticRelease, kMoveToB,
                   kHoldAtB, kMoveToA, kRecoveryRest, kRecoveryMove,
                   kComplete, kAborted };

const char *phaseName(Phase phase) {
  switch (phase) {
    case Phase::kWaitForState: return "waiting_for_state";
    case Phase::kStartupMove: return "startup_move_to_state_a";
    case Phase::kStartHold: return "state_a_hold";
    case Phase::kStaticRelease: return "static_release";
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
    strategy_version_ = declare_parameter<std::string>("strategy_version", "1.1.0");
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
    ff_outward_ = declare_parameter<double>("hip_ff_outward_direct", 0.0);
    ff_inward_ = declare_parameter<double>("hip_ff_inward_direct", 0.0);
    ff_static_outward_ = declare_parameter<double>("hip_ff_static_outward_direct", 0.0);
    ff_static_inward_ = declare_parameter<double>("hip_ff_static_inward_direct", 0.0);
    max_ff_ = declare_parameter<double>("max_abs_hip_ff_torque_nm", 200.0);
    start_hold_s_ = declare_parameter<double>("start_zero_hold_s", 2.0);
    static_release_s_ = declare_parameter<double>("static_release_ramp_s", 1.0);
    static_release_fraction_ = declare_parameter<double>("static_release_fraction", 0.15);
    segment_hold_s_ = declare_parameter<double>("segment_hold_s", 1.0);
    max_state_age_s_ = declare_parameter<double>("max_state_age_s", 0.10);
    max_error_rad_ = declare_parameter<double>("max_abs_hip_error_rad", 0.35);
    max_torque_ = declare_parameter<double>("max_abs_hip_torque_nm", 400.0);

    if (armed_ && run_dir_.empty()) throw std::runtime_error("armed run requires run_dir");
    const bool static_release_disabled = static_release_s_ == 0.0 && static_release_fraction_ == 0.0;
    const bool static_release_enabled = static_release_s_ > 0.0 && static_release_fraction_ > 0.0 && static_release_fraction_ < 1.0;
    if (repetitions_ < 1 || startup_speed_ <= 0.0 || recovery_speed_ <= 0.0 || hip_speed_ <= 0.0 || (!static_release_disabled && !static_release_enabled)) throw std::runtime_error("invalid repetitions, speed, or static-release settings");
    for (const auto &name : active_modules_) {
      if (std::find(kModuleNames.begin(), kModuleNames.end(), name) == kModuleNames.end()) throw std::runtime_error("unknown active module");
    }
    publisher_ = create_publisher<kilin_msgs::msg::MotorCmdStamped>(command_topic_, 10);
    subscriber_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(state_topic_, rclcpp::QoS(20).best_effort(), std::bind(&CampaignRunner::onState, this, std::placeholders::_1));
    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&CampaignRunner::tick, this));
    RCLCPP_INFO(get_logger(), "Runner ready: strategy=%s v%s armed=%s command_topic=%s", strategy_name_.c_str(), strategy_version_.c_str(), armed_ ? "true" : "false", command_topic_.c_str());
  }

 private:
  struct HipState { double motor{NAN}; double diff{NAN}; double torque{NAN}; int error{0}; };

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
    for (size_t i = 0; i < 4; ++i) hips_[i] = {legs[i].hip.position, legs[i].hip.position_diff, legs[i].hip.torque, legs[i].hip.error_code};
    last_state_ = now();
    have_state_ = true;
  }

  void begin() {
    for (size_t i = 0; i < 4; ++i) {
      if (!std::isfinite(actual(i))) throw std::runtime_error("non-finite hip feedback");
      move_start_[i] = hips_[i].motor;
      commanded_[i] = hips_[i].motor;
    }
    openEvidence();
    setPhase(Phase::kStartupMove, "moving to configured state A with wheels rest and zero FF");
  }

  void setPhase(Phase next, const std::string &detail) {
    phase_ = next;
    phase_start_ = now();
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

  void commandPosition(const std::array<double, 4> &target, bool allow_ff) {
    kilin_msgs::msg::MotorCmdStamped message;
    stamp(message);
    std::array<kilin_msgs::msg::LegCmd, 4> legs;
    for (size_t i = 0; i < 4; ++i) {
      commanded_[i] = target[i];
      legs[i].hip.motor_mode = kPosition;
      // Do not compensate the motor command with position_diff.  It is kept in
      // the state/logging path as the reconstructed actual hip angle, but it is
      // not part of this low-level position-control loop.
      legs[i].hip.position = target[i];
      legs[i].hip.kp = kp_; legs[i].hip.ki = ki_; legs[i].hip.kd = kd_;
      legs[i].hip.torque = allow_ff ? feedforward(i, target[i] - hips_[i].motor) : 0.0;
      legs[i].steering.motor_mode = kPosition;
      legs[i].steering.position = 0.0;
      legs[i].hub.motor_mode = kRest;
    }
    message.module_a = legs[0]; message.module_b = legs[1]; message.module_c = legs[2]; message.module_d = legs[3];
    publisher_->publish(message);
    writeTrace();
  }

  double feedforward(size_t i, double direction) const {
    if (!active(i) || std::abs(direction) < 0.01) return 0.0;
    const bool outward = direction * sign(i) > 0.0;
    const double magnitude = phase_ == Phase::kStaticRelease
        ? (outward ? ff_static_outward_ : ff_static_inward_)
        : (outward ? ff_outward_ : ff_inward_);
    return std::clamp(std::copysign(magnitude, direction), -max_ff_, max_ff_);
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
        if (elapsed() >= start_hold_s_) {
          if (static_release_fraction_ == 0.0) setPhase(Phase::kMoveToB, "static release disabled; beginning dynamic move to state B");
          else setPhase(Phase::kStaticRelease, "static-release ramp for breakaway friction");
        }
        break;
      case Phase::kStaticRelease:
        commandPosition(interpolated(a, b, static_release_fraction_ * smooth(elapsed() / static_release_s_)), true);
        if (elapsed() >= static_release_s_) setPhase(Phase::kMoveToB, "constant-speed dynamic move to state B");
        break;
      case Phase::kMoveToB:
        commandPosition(interpolated(a, b, static_release_fraction_ + (1.0 - static_release_fraction_) * clamp01(elapsed() / ((1.0 - static_release_fraction_) * fullStrokeDuration()))), true);
        if (elapsed() >= (1.0 - static_release_fraction_) * fullStrokeDuration()) setPhase(Phase::kHoldAtB, "state B reached");
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
  void publishRest() { kilin_msgs::msg::MotorCmdStamped message; stamp(message); kilin_msgs::msg::LegCmd leg; leg.hip.motor_mode=kRest; leg.steering.motor_mode=kRest; leg.hub.motor_mode=kBrake; message.module_a=leg;message.module_b=leg;message.module_c=leg;message.module_d=leg;publisher_->publish(message); }
  void openEvidence() { if (run_dir_.empty()) return; fs::create_directories(run_dir_); trace_.open(fs::path(run_dir_) / "command_state_trace.csv"); trace_ << "time_s,phase,trial,module,commanded_motor_rad,motor_position_rad,position_diff_rad,actual_hip_angle_rad,hip_torque,error_code\n"; std::ofstream manifest(fs::path(run_dir_) / "trial_manifest.yaml"); manifest << "strategy_name: " << strategy_name_ << "\nstrategy_version: " << strategy_version_ << "\nangle_convention: actual_hip_angle_rad = motor_position + position_diff\ncontrol_angle_reference: motor_position (position_diff is measurement-only)\n"; }
  void writeTrace() { if (!trace_) return; for (size_t i=0;i<4;++i) trace_ << now().seconds() << ',' << phaseName(phase_) << ',' << completed_repetitions_ + 1 << ',' << kModuleNames[i] << ',' << commanded_[i] << ',' << hips_[i].motor << ',' << hips_[i].diff << ',' << actual(i) << ',' << hips_[i].torque << ',' << hips_[i].error << '\n'; }

  bool armed_{false}, have_state_{false}; int repetitions_{3}, completed_repetitions_{0}; uint32_t sequence_{0};
  std::string command_topic_, state_topic_, run_dir_, strategy_name_, strategy_version_; std::vector<std::string> active_modules_;
  double state_a_deg_{0}, state_b_deg_{45}, startup_speed_{.1}, recovery_deg_{0}, recovery_speed_{.1}, recovery_rest_s_{1}, hip_speed_{.2}, kp_{360}, ki_{0}, kd_{5}, ff_outward_{0}, ff_inward_{0}, ff_static_outward_{0}, ff_static_inward_{0}, max_ff_{200}, start_hold_s_{2}, static_release_s_{1}, static_release_fraction_{.15}, segment_hold_s_{1}, max_state_age_s_{.1}, max_error_rad_{.35}, max_torque_{400};
  std::array<HipState, 4> hips_{}; std::array<double, 4> commanded_{}, move_start_{}; Phase phase_{Phase::kWaitForState}; rclcpp::Time phase_start_{0,0,RCL_ROS_TIME}, last_state_{0,0,RCL_ROS_TIME}; std::ofstream trace_;
  rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr publisher_; rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr subscriber_; rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc, char **argv) { rclcpp::init(argc, argv); rclcpp::spin(std::make_shared<CampaignRunner>()); rclcpp::shutdown(); return 0; }
