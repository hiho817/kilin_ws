// Copyright 2026 Ian

#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kilin_msgs/msg/balance_state_stamped.hpp>
#include <kilin_msgs/msg/motor_state_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kilin_com_estimator/joint_mapping.hpp"
#include "kilin_com_estimator/robot_com_model.hpp"

namespace kilin_com_estimator
{
namespace
{
const std::vector<std::string> kDefaultInputArmJoints = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"};
const std::vector<std::string> kDefaultUrdfArmJoints = {
  "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4",
  "arm_joint_5", "arm_joint_6", "arm_joint_7"};
constexpr double kExpectedModelMassKg = 41.8929641;
}  // namespace

class KilinComEstimator : public rclcpp::Node
{
public:
  KilinComEstimator()
  : Node("kilin_com_estimator")
  {
    declare_parameter<std::string>("urdf_path", "");
    declare_parameter<std::string>("motor_state_topic", "/motor/state");
    declare_parameter<std::string>("arm_joint_state_topic", "/joint_states");
    declare_parameter<std::string>("balance_state_topic", "/kilin/balance_state");
    declare_parameter<double>("publish_rate_hz", 30.0);
    declare_parameter<double>("input_timeout_sec", 0.5);
    declare_parameter<double>("wheel_radius_m", 0.0525);
    declare_parameter<bool>("assume_level_base", true);
    declare_parameter<std::vector<std::string>>(
      "arm_input_joint_names", kDefaultInputArmJoints);
    declare_parameter<std::vector<std::string>>(
      "arm_urdf_joint_names", kDefaultUrdfArmJoints);

    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();
    wheel_radius_m_ = get_parameter("wheel_radius_m").as_double();
    assume_level_base_ = get_parameter("assume_level_base").as_bool();
    input_arm_joint_names_ = get_parameter("arm_input_joint_names").as_string_array();
    urdf_arm_joint_names_ = get_parameter("arm_urdf_joint_names").as_string_array();

    if (publish_rate_hz <= 0.0 || input_timeout_sec_ <= 0.0 || wheel_radius_m_ <= 0.0) {
      throw std::invalid_argument("publish rate, timeout, and wheel radius must be positive");
    }
    if (!assume_level_base_) {
      throw std::invalid_argument(
              "this estimator version has no base-orientation source; "
              "assume_level_base must remain true");
    }
    if (input_arm_joint_names_.size() != 7 || urdf_arm_joint_names_.size() != 7) {
      throw std::invalid_argument("exactly seven input and URDF Kinova joint names are required");
    }

    std::string urdf_path = get_parameter("urdf_path").as_string();
    if (urdf_path.empty()) {
      urdf_path = ament_index_cpp::get_package_share_directory("kilin_robot_description") +
        "/urdf/kilin_gen3_hardware.urdf";
    }
    model_ = std::make_unique<RobotComModel>(urdf_path);
    if (std::abs(model_->model_mass() - kExpectedModelMassKg) > 1e-4) {
      throw std::runtime_error(
              "unexpected combined URDF mass; expected 41.8929641 kg but loaded " +
              std::to_string(model_->model_mass()) + " kg");
    }

    const auto motor_state_topic = get_parameter("motor_state_topic").as_string();
    const auto arm_joint_state_topic = get_parameter("arm_joint_state_topic").as_string();
    const auto balance_state_topic = get_parameter("balance_state_topic").as_string();

    motor_state_sub_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(
      motor_state_topic, rclcpp::QoS(10),
      std::bind(&KilinComEstimator::motor_state_callback, this, std::placeholders::_1));
    arm_joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_joint_state_topic, rclcpp::QoS(10),
      std::bind(&KilinComEstimator::arm_joint_state_callback, this, std::placeholders::_1));
    balance_state_pub_ = create_publisher<kilin_msgs::msg::BalanceStateStamped>(
      balance_state_topic, rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      std::bind(&KilinComEstimator::publish_estimate, this));

    RCLCPP_INFO(
      get_logger(),
      "Hardware COM estimator ready: mass=%.4f kg, motor='%s', arm='%s', output='%s'",
      model_->model_mass(), motor_state_topic.c_str(), arm_joint_state_topic.c_str(),
      balance_state_topic.c_str());
    RCLCPP_WARN(
      get_logger(),
      "No base orientation is available: output frame is base_link_assumed_level. "
      "Use this version for flat-ground validation, not stair closed-loop control.");
  }

private:
  void motor_state_callback(const kilin_msgs::msg::MotorStateStamped::SharedPtr msg)
  {
    const std::array<std::pair<std::string, const kilin_msgs::msg::MotorState *>, 4> hips = {{
      {"FL_hip", &msg->module_a.hip},
      {"FR_hip", &msg->module_b.hip},
      {"RL_hip", &msg->module_c.hip},
      {"RR_hip", &msg->module_d.hip},
    }};

    std::map<std::string, double> mapped;
    for (const auto & entry : hips) {
      const double actual = actual_motor_position(
        entry.second->position, entry.second->position_diff);
      if (!std::isfinite(actual)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "MotorState contains a non-finite hip position or position_diff");
        return;
      }
      mapped[entry.first] = actual;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    hip_positions_ = std::move(mapped);
    last_motor_receive_time_ = now();
    have_motor_state_ = true;
  }

  void arm_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::map<std::string, double> mapped;
    std::string error;
    if (!map_named_arm_positions(
        msg->name, msg->position, input_arm_joint_names_, urdf_arm_joint_names_,
        mapped, error))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring Kinova JointState: %s", error.c_str());
      return;
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    arm_positions_ = std::move(mapped);
    last_arm_receive_time_ = now();
    have_arm_state_ = true;
  }

  void publish_estimate()
  {
    std::map<std::string, double> joint_positions;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (!have_motor_state_ || !have_arm_state_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for both Kilin motor state and Kinova joint state");
        return;
      }
      const auto current_time = now();
      const double motor_age = (current_time - last_motor_receive_time_).seconds();
      const double arm_age = (current_time - last_arm_receive_time_).seconds();
      if (motor_age > input_timeout_sec_ || arm_age > input_timeout_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Estimator input stale: motor=%.3f s, arm=%.3f s", motor_age, arm_age);
        return;
      }
      joint_positions = hip_positions_;
      joint_positions.insert(arm_positions_.begin(), arm_positions_.end());
    }

    RobotComResult estimate;
    try {
      estimate = model_->compute(joint_positions);
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "COM computation failed: %s", error.what());
      return;
    }

    kilin_msgs::msg::BalanceStateStamped output;
    output.header.seq = static_cast<int32_t>(sequence_++);
    const auto stamp_ns = now().nanoseconds();
    output.header.time.sec = static_cast<int32_t>(stamp_ns / 1000000000LL);
    output.header.time.nanosec = static_cast<uint32_t>(stamp_ns % 1000000000LL);
    output.header.frame_id = "base_link_assumed_level";
    output.com.x = estimate.com.x();
    output.com.y = estimate.com.y();
    output.com.z = estimate.com.z();
    for (std::size_t index = 0; index < estimate.wheel_centers.size(); ++index) {
      output.contact_points[index].x = estimate.wheel_centers[index].x();
      output.contact_points[index].y = estimate.wheel_centers[index].y();
      output.contact_points[index].z = estimate.wheel_centers[index].z() - wheel_radius_m_;
      output.sensor_valid[index] = false;
      output.in_contact[index] = false;
      output.supported[index] = false;
      output.contact_force[index] = std::numeric_limits<double>::quiet_NaN();
    }
    balance_state_pub_->publish(output);
  }

  std::unique_ptr<RobotComModel> model_;
  std::vector<std::string> input_arm_joint_names_;
  std::vector<std::string> urdf_arm_joint_names_;
  double input_timeout_sec_{0.5};
  double wheel_radius_m_{0.0525};
  bool assume_level_base_{true};
  uint32_t sequence_{0};

  std::mutex state_mutex_;
  std::map<std::string, double> hip_positions_;
  std::map<std::string, double> arm_positions_;
  rclcpp::Time last_motor_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_receive_time_{0, 0, RCL_ROS_TIME};
  bool have_motor_state_{false};
  bool have_arm_state_{false};

  rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_sub_;
  rclcpp::Publisher<kilin_msgs::msg::BalanceStateStamped>::SharedPtr balance_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace kilin_com_estimator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kilin_com_estimator::KilinComEstimator>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("kilin_com_estimator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
