/////////////////////////////////////////////////////////////////////////////
//
// imu_subscriber_logger.cpp
// 
// trigger using gpio09 on jetson Orin
// high to start recording, low to stop recording
// can be tested in test mode by setting parameter test_mode to true
// in test mode, logging is toggled by calling the toggle_logging service
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/string.hpp"

#include <fstream>
#include <filesystem>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <memory>

#include <gpiod.h>

using namespace std::chrono_literals;

#define TRIGGER_PIN 8 // GPIO pin number for triggering (example: GPIO 24)
#define TRIGGER_CHIP "gpiochip1" // GPIO chip name (example: "gpiochip0")

class IMULoggerNode : public rclcpp::Node {
public:
  IMULoggerNode() : Node("imu_logger_node"),
										logging_(false),
										test_mode_(false),
										pin_state_(false),
										last_pin_state_(false) {
    declare_parameter("test_mode", true);
    declare_parameter<std::string>(
      "log_dir",
      std::string(std::getenv("HOME")) + "/kilin_ws/logs/imu_logs"
    );

    log_dir_ = get_parameter("log_dir").as_string();
    test_mode_ = get_parameter("test_mode").as_bool();

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 10,std::bind(&IMULoggerNode::imu_callback, this, std::placeholders::_1)
		);

    toggle_service_ = create_service<std_srvs::srv::Trigger>(
      "toggle_logging",
      std::bind(&IMULoggerNode::handle_toggle, this, std::placeholders::_1, std::placeholders::_2)
		);

    // configuration for gpio
    chip_ = gpiod_chip_open_by_name(TRIGGER_CHIP);
    if (!chip_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open chip %s", TRIGGER_CHIP);
      return;
    }

    line_ = gpiod_chip_get_line(chip_, TRIGGER_PIN);
    if (!line_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get line %d", TRIGGER_PIN);
      return;
    }

    if (gpiod_line_request_input(line_, "ros2_gpio_reader") < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to request line as input");
      return;
    }

    timer_ = create_wall_timer(100ms, std::bind(&IMULoggerNode::check_gpio, this));

    RCLCPP_INFO(get_logger(), "IMU Logger started in %s mode", test_mode_ ? "TEST" : "GPIO");
  }
  
  ~IMULoggerNode() {
    if (line_) gpiod_line_release(line_);
    if (chip_) gpiod_chip_close(chip_);
  }

private:
  void exit_gracefully(const char *message){
    RCLCPP_INFO(this->get_logger(), "node dead: %s", message);
    rclcpp::shutdown();
    exit(0);
  }


  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    if (!logging_) return;

    auto now = this->now();
    log_file_ << std::fixed << std::setprecision(3)
							<< now.seconds() << ","
							<< msg->orientation.x << ","
							<< msg->orientation.y << ","
							<< msg->orientation.z << ","
							<< msg->orientation.w << ","
							<< msg->angular_velocity.x << ","
							<< msg->angular_velocity.y << ","
							<< msg->angular_velocity.z << ","
							<< msg->linear_acceleration.x << ","
							<< msg->linear_acceleration.y << ","
							<< msg->linear_acceleration.z << "\n";
  }

  void check_gpio() {
    bool current_state = test_mode_ ? pin_state_ : read_gpio();

    if (current_state && !last_pin_state_) {
      start_logging();
    } else if (!current_state && last_pin_state_) {
      stop_logging();
    }

    last_pin_state_ = current_state;
  }

  bool read_gpio() {
    int value = gpiod_line_get_value(line_);
    if (value < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read GPIO value");
      return false;
    }
    // RCLCPP_INFO(this->get_logger(), "GPIO value: %d", value);
    return value == 1;
  }

  void start_logging() {
    if (logging_) return;

    // -------------------------------------------------------------
    // Ensure log directory exists
    // -------------------------------------------------------------
    std::error_code ec;
    std::filesystem::create_directories(log_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to create log directory '%s': %s",
        log_dir_.c_str(),
        ec.message().c_str()
      );
      return;
    }

    std::string filename = generate_filename();
    std::string full_path = log_dir_ + "/" + filename;

    log_file_.open(full_path, std::ios::out);
    if (!log_file_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file '%s'", full_path.c_str());
      return;
    }

    log_file_ << "timestamp,q_x,q_y,q_z,q_w,"
              << "g_x,g_y,g_z,"
              << "a_x,a_y,a_z\n";

    logging_ = true;
    RCLCPP_INFO(get_logger(), "Started logging to %s", full_path.c_str());
  }

  void stop_logging() {
    if (!logging_) return;

    log_file_.close();
    logging_ = false;
    RCLCPP_INFO(get_logger(), "Stopped logging");
  }

  std::string generate_filename() {
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    std::ostringstream oss;
    oss << "imu_log_" << std::put_time(&tm, "%Y%m%d_%H%M%S") << ".csv";
    return oss.str();
  }


  void handle_toggle(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    pin_state_ = !pin_state_;
    response->success = true;
    response->message = std::string("Pin toggled to ") + (pin_state_ ? "HIGH" : "LOW");
  }

  bool logging_, test_mode_, pin_state_, last_pin_state_;
  std::string log_dir_;
  std::ofstream log_file_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_service_;
  rclcpp::TimerBase::SharedPtr timer_;

  
  gpiod_chip *chip_ = nullptr;
  gpiod_line *line_ = nullptr;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IMULoggerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
