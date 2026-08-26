// Copyright 2026 Ian

#include <algorithm>
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
#include <kilin_msgs/msg/stair_terrain_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "kilin_com_estimator/com_bias.hpp"
#include "kilin_com_estimator/joint_mapping.hpp"
#include "kilin_com_estimator/robot_com_model.hpp"
#include "kilin_com_estimator/terrain_orientation.hpp"

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
    declare_parameter<std::vector<double>>(
      "com_bias_base_m", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<bool>("assume_level_base", true);
    declare_parameter<bool>("require_terrain_state", false);
    declare_parameter<std::string>("terrain_state_topic", "/kilin/stair_terrain");
    declare_parameter<std::vector<std::string>>(
      "arm_input_joint_names", kDefaultInputArmJoints);
    declare_parameter<std::vector<std::string>>(
      "arm_urdf_joint_names", kDefaultUrdfArmJoints);

    const double publish_rate_hz = get_parameter("publish_rate_hz").as_double();
    input_timeout_sec_ = get_parameter("input_timeout_sec").as_double();
    wheel_radius_m_ = get_parameter("wheel_radius_m").as_double();
    com_bias_base_ = parse_com_bias(
      get_parameter("com_bias_base_m").as_double_array());
    assume_level_base_ = get_parameter("assume_level_base").as_bool();
    require_terrain_state_ = get_parameter("require_terrain_state").as_bool();
    input_arm_joint_names_ = get_parameter("arm_input_joint_names").as_string_array();
    urdf_arm_joint_names_ = get_parameter("arm_urdf_joint_names").as_string_array();

    if (publish_rate_hz <= 0.0 || input_timeout_sec_ <= 0.0 || wheel_radius_m_ <= 0.0) {
      throw std::invalid_argument("publish rate, timeout, and wheel radius must be positive");
    }
    if (!assume_level_base_ && !require_terrain_state_) {
      throw std::invalid_argument(
              "assume_level_base=false requires require_terrain_state=true");
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
    const auto terrain_state_topic = get_parameter("terrain_state_topic").as_string();

    motor_state_sub_ = create_subscription<kilin_msgs::msg::MotorStateStamped>(
      motor_state_topic, rclcpp::QoS(10),
      std::bind(&KilinComEstimator::motor_state_callback, this, std::placeholders::_1));
    arm_joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      arm_joint_state_topic, rclcpp::QoS(10),
      std::bind(&KilinComEstimator::arm_joint_state_callback, this, std::placeholders::_1));
    terrain_state_sub_ = create_subscription<kilin_msgs::msg::StairTerrainStamped>(
      terrain_state_topic, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&KilinComEstimator::terrain_state_callback, this, std::placeholders::_1));
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
    RCLCPP_INFO(
      get_logger(), "Base-frame COM bias: [%.1f, %.1f, %.1f] mm",
      com_bias_base_.x() * 1000.0, com_bias_base_.y() * 1000.0,
      com_bias_base_.z() * 1000.0);
    if (require_terrain_state_) {
      RCLCPP_INFO(
        get_logger(), "Known-stair orientation input: %s", terrain_state_topic.c_str());
    } else {
      RCLCPP_WARN(
        get_logger(),
        "Terrain orientation is optional: output remains level-assumed until metadata arrives.");
    }
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

  void terrain_state_callback(const kilin_msgs::msg::StairTerrainStamped::SharedPtr msg)
  {
    if (!msg->valid || !std::isfinite(msg->stair_rise_m) || msg->stair_rise_m <= 0.0 ||
      std::count(msg->supported.begin(), msg->supported.end(), true) < 3)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Ignoring invalid stair terrain metadata");
      return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    terrain_state_ = *msg;
    last_terrain_receive_time_ = now();
    have_terrain_state_ = true;
  }

  void publish_estimate()
  {
    std::map<std::string, double> joint_positions;
    kilin_msgs::msg::StairTerrainStamped terrain;
    bool use_terrain = false;
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
      if (have_terrain_state_) {
        const double terrain_age = (current_time - last_terrain_receive_time_).seconds();
        if (terrain_age <= input_timeout_sec_) {
          terrain = terrain_state_;
          use_terrain = true;
        } else if (require_terrain_state_) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Required stair terrain metadata is stale: %.3f s", terrain_age);
          return;
        }
      } else if (require_terrain_state_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000, "Waiting for required stair terrain metadata");
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
    std::array<Eigen::Vector3d, 4> contact_points;
    for (std::size_t index = 0; index < estimate.wheel_centers.size(); ++index) {
      // Tread-height differences are identical for wheel centers and contact
      // points because every wheel has the same radius.  Solving from centers
      // avoids assuming that base -Z is the gravity/contact direction.
      contact_points[index] = estimate.wheel_centers[index];
    }

    Eigen::Matrix3d base_to_output = Eigen::Matrix3d::Identity();
    output.orientation_valid = false;
    output.terrain_seq = -1;
    output.base_to_output_rotation.w = 1.0;
    output.header.frame_id = "base_link_assumed_level";
    if (use_terrain) {
      const auto orientation = estimate_terrain_orientation(
        contact_points, terrain.tread_level, terrain.supported,
        terrain.stair_rise_m, previous_vertical_);
      if (!orientation) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Known stair heights are inconsistent with the current wheel geometry");
        return;
      }
      previous_vertical_ = orientation->vertical_in_base;
      base_to_output = orientation->base_to_level;
      const Eigen::Quaterniond quaternion(base_to_output);
      output.orientation_valid = true;
      output.terrain_seq = terrain.header.seq;
      output.base_to_output_rotation.x = quaternion.x();
      output.base_to_output_rotation.y = quaternion.y();
      output.base_to_output_rotation.z = quaternion.z();
      output.base_to_output_rotation.w = quaternion.w();
      output.header.frame_id = "base_link_gravity_aligned";
    }

    // The force-plate calibration is expressed in base_link. Apply it before
    // rotating the COM into the gravity-aligned output frame.
    const Eigen::Vector3d output_com = base_to_output * (estimate.com + com_bias_base_);
    output.com.x = output_com.x();
    output.com.y = output_com.y();
    output.com.z = output_com.z();
    for (std::size_t index = 0; index < estimate.wheel_centers.size(); ++index) {
      const Eigen::Vector3d output_point =
        base_to_output * contact_points[index] -
        Eigen::Vector3d(0.0, 0.0, wheel_radius_m_);
      output.contact_points[index].x = output_point.x();
      output.contact_points[index].y = output_point.y();
      output.contact_points[index].z = output_point.z();
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
  Eigen::Vector3d com_bias_base_{Eigen::Vector3d::Zero()};
  bool assume_level_base_{true};
  bool require_terrain_state_{false};
  uint32_t sequence_{0};

  std::mutex state_mutex_;
  std::map<std::string, double> hip_positions_;
  std::map<std::string, double> arm_positions_;
  rclcpp::Time last_motor_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_arm_receive_time_{0, 0, RCL_ROS_TIME};
  bool have_motor_state_{false};
  bool have_arm_state_{false};
  bool have_terrain_state_{false};
  kilin_msgs::msg::StairTerrainStamped terrain_state_;
  rclcpp::Time last_terrain_receive_time_{0, 0, RCL_ROS_TIME};
  std::optional<Eigen::Vector3d> previous_vertical_;

  rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr motor_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_joint_state_sub_;
  rclcpp::Subscription<kilin_msgs::msg::StairTerrainStamped>::SharedPtr terrain_state_sub_;
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
