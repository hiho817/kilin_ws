#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "kilin_hip_characterization/hip_control_strategy.hpp"
#include "kilin_msgs/msg/hip_fine_tune_command.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/motor_state_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
class FineTuneProcessor final : public rclcpp::Node
{
public:
  FineTuneProcessor() : Node("kilin_hip_fine_tune")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/kilin/hip_fine_tune/input");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/kilin/motor_cmd_raw");
    const auto state_topic = declare_parameter<std::string>("state_topic", "/motor/state");
    max_state_age_s_ = declare_parameter<double>("max_state_age_s", 0.10);
    kilin_hip_characterization::HipControlStrategyConfig config;
    config.enabled = declare_parameter<bool>("enabled", false);
    config.pid_schedule_enabled = declare_parameter<bool>("pid_schedule_enabled", false);
    const auto support = declare_parameter<std::vector<double>>(
      "support_region_end_rad", {-M_PI / 180.0, -M_PI / 180.0, -M_PI / 180.0, -M_PI / 180.0});
    const auto lift = declare_parameter<std::vector<double>>(
      "lift_region_start_rad", {M_PI / 180.0, M_PI / 180.0, M_PI / 180.0, M_PI / 180.0});
    if (support.size() != 4 || lift.size() != 4 || max_state_age_s_ <= 0.0) {
      throw std::runtime_error("fine-tune regions must contain four values and state timeout must be positive");
    }
    for (size_t i = 0; i < 4; ++i) { config.support_end[i] = support[i]; config.lift_start[i] = lift[i]; }
    config.support_kp = declare_parameter<double>("support_kp", 350.0);
    config.support_ki = declare_parameter<double>("support_ki", 0.0);
    config.support_kd = declare_parameter<double>("support_kd", 5.0);
    config.lift_kp = declare_parameter<double>("lift_kp", config.support_kp);
    config.lift_ki = declare_parameter<double>("lift_ki", config.support_ki);
    config.lift_kd = declare_parameter<double>("lift_kd", config.support_kd);
    config.lift_start_inward_ff = declare_parameter<double>("lift_start_inward_ff_nm", 0.0);
    config.lift_max_inward_ff = declare_parameter<double>("lift_max_inward_ff_nm", 0.0);
    config.lift_ramp_up_nm_s = declare_parameter<double>("lift_ramp_up_nm_s", 0.0);
    config.lift_ramp_down_nm_s = declare_parameter<double>("lift_ramp_down_nm_s", 0.0);
    config.reset_lift_on_support = declare_parameter<bool>("reset_lift_on_support", true);
    config.support_inward_ff = declare_parameter<double>("support_inward_ff_nm", 0.0);
    config.apply_rate_nm_s = declare_parameter<double>("apply_rate_nm_s", 0.0);
    config.release_rate_nm_s = declare_parameter<double>("release_rate_nm_s", 0.0);
    config.max_abs_ff = declare_parameter<double>("max_abs_ff_nm", 200.0);
    config.kp_to_lift_rate_per_s = declare_parameter<double>("kp_to_lift_rate_per_s", 0.0);
    config.kp_to_support_rate_per_s = declare_parameter<double>("kp_to_support_rate_per_s", 0.0);
    config.ki_to_lift_rate_per_s = declare_parameter<double>("ki_to_lift_rate_per_s", 0.0);
    config.ki_to_support_rate_per_s = declare_parameter<double>("ki_to_support_rate_per_s", 0.0);
    config.kd_to_lift_rate_per_s = declare_parameter<double>("kd_to_lift_rate_per_s", 0.0);
    config.kd_to_support_rate_per_s = declare_parameter<double>("kd_to_support_rate_per_s", 0.0);
    strategy_.configure(config);
    publisher_ = create_publisher<kilin_msgs::msg::MotorCmdStamped>(output_topic, 10);
    state_sub_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(state_topic,
      rclcpp::QoS(20).best_effort(),
      [this](const kilin_msgs::msg::MotorStateStamped::SharedPtr message) {
        state_ = *message; state_time_ = now(); have_state_ = true;
      });
    input_sub_ = create_subscription<kilin_msgs::msg::HipFineTuneCommand>(input_topic, 10,
      [this](const kilin_msgs::msg::HipFineTuneCommand::SharedPtr message) { process(*message); });
  }

private:
  void process(const kilin_msgs::msg::HipFineTuneCommand & input)
  {
    if (!have_state_ || (now() - state_time_).seconds() > max_state_age_s_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Fine tune is waiting for fresh motor state");
      return;
    }
    auto output = input.base_command;
    const std::array<kilin_msgs::msg::LegState, 4> states{
      state_.module_a, state_.module_b, state_.module_c, state_.module_d};
    std::array<kilin_msgs::msg::LegCmd *, 4> legs{
      &output.module_a, &output.module_b, &output.module_c, &output.module_d};
    const bool normal = input.context == kilin_msgs::msg::HipFineTuneCommand::NORMAL_CONTROL;
    for (size_t i = 0; i < legs.size(); ++i) {
      const auto & feedback = states[i].hip;
      const double target = legs[i]->hip.position;
      const double outward = (i < 2 ? -1.0 : 1.0) * target;
      const auto tuned = strategy_.update({i, feedback.position, feedback.position_diff, target,
        now().seconds(), input.active_modules[i], normal, outward >= 0.0 && outward <= M_PI / 2.0,
        legs[i]->hip.kp, legs[i]->hip.ki, legs[i]->hip.kd});
      legs[i]->hip.kp = tuned.kp;
      legs[i]->hip.ki = tuned.ki;
      legs[i]->hip.kd = tuned.kd;
      legs[i]->hip.torque += tuned.motor_ff;
    }
    publisher_->publish(output);
  }

  double max_state_age_s_{};
  rclcpp::Time state_time_{0, 0, RCL_ROS_TIME};
  bool have_state_{false};
  kilin_msgs::msg::MotorStateStamped state_{};
  kilin_hip_characterization::HipControlStrategy strategy_{};
  rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr publisher_;
  rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr state_sub_;
  rclcpp::Subscription<kilin_msgs::msg::HipFineTuneCommand>::SharedPtr input_sub_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FineTuneProcessor>());
  rclcpp::shutdown();
  return 0;
}
