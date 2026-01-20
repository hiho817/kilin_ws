/////////////////////////////////////////////////////////////////////////////
//
// imu_puglisher.cpp
//
// derived from CX5_GX5_CV5_15_25_Example.cpp
//
/////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////
// Include Files
////////////////////////////////////////////////////////////////////////////////
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "std_msgs/msg/string.hpp"

// above is from node example

#include <mip/mip_all.hpp>
#include <array>
#include "mip_utils/mip_utils.hpp"
#include "rclcpp/rclcpp.hpp"

#include <Eigen/Dense>
#include "sensor_msgs/msg/imu.hpp"

#include <cmath>


using namespace std::chrono_literals;
using namespace mip;


#define PORT "/dev/ttyACM0"
#define BAUD 115200
#define SENSOR_SAMPLE_RATE 100 // in Hz
#define PUBLISH_PERIOD 100 // in ms
#define PRINT_INFO_PERIOD 10000 // in ms
#define GYRO_BIAS_SAMPLE_TIME 2000 // in ms
#define FILTER_SAMPLE_RATE 100 // in Hz


////////////////////////////////////////////////////////////////////////////////
// node, publisher
////////////////////////////////////////////////////////////////////////////////

class IMUpublisher : public rclcpp::Node
{
  public:
    IMUpublisher()
    : Node("imu_publisher"),
      filter_state_running(false),
      publish_counter(0)
    {
      // -------------------------------------------------------------
      // Parameters
      // -------------------------------------------------------------
      declare_parameter("port", PORT);
      declare_parameter("baud", BAUD);
      declare_parameter("publish_period", PUBLISH_PERIOD);
      declare_parameter("print_info_period", PRINT_INFO_PERIOD);

      port_ = get_parameter("port").as_string();
      baud_ = get_parameter("baud").as_int();

      RCLCPP_INFO(this->get_logger(), "Opening connection to device on port %s at %d baud", port_.c_str(), baud_);

      utils_ = openFromArgs(port_, std::to_string(baud_), "");
      device_ = utils_->device.get();

      int print_info_period = get_parameter("print_info_period").as_int();
      int pub_period = get_parameter("publish_period").as_int();

      RCLCPP_INFO(this->get_logger(), "will publish imu data at %d ms period, print at %d ms", pub_period, print_info_period);

      publish_period_ = std::chrono::milliseconds(pub_period);
      print_info_period_ = std::chrono::milliseconds(print_info_period);

      // -------------------------------------------------------------
      // Connect and configure sensor
      // -------------------------------------------------------------
      RCLCPP_INFO(this->get_logger(), "Connecting to and configuring sensor.");

      // Ping the device
      if(commands_base::ping(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not ping the device!");

      // Idle the device
      if(commands_base::setIdle(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set the device to idle!");

      // Load default settings
      if(commands_3dm::defaultDeviceSettings(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not load default device settings!");

      // -------------------------------------------------------------
      // Capture gyro bias
      // -------------------------------------------------------------
      float gyro_bias[3] = {0, 0, 0};

      const uint32_t sampling_time = GYRO_BIAS_SAMPLE_TIME;
      const int32_t old_mip_sdk_timeout = device_->baseReplyTimeout();

      RCLCPP_INFO(this->get_logger(), "Capturing gyro bias. This will take %d seconds", sampling_time/1000);

      device_->setBaseReplyTimeout(sampling_time * 2);

      if(commands_3dm::captureGyroBias(*device_, sampling_time, gyro_bias) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not capture gyro bias!");

      if(commands_3dm::saveGyroBias(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not save gyro bias!");

      const uint8_t device_selector = 3;
      const uint8_t enable_flag = 1;
      if(commands_3dm::writeDatastreamControl(*device_, device_selector, enable_flag) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not enable device data stream!");

      // Reset the timeout
      device_->setBaseReplyTimeout(old_mip_sdk_timeout);

      RCLCPP_INFO(this->get_logger(),
                  "Gyro bias captured with sampling time: %d, and gyro bias captured as: %f %f %f.",
                  sampling_time, gyro_bias[0], gyro_bias[1], gyro_bias[2]);

      // -------------------------------------------------------------
      // Setup Sensor data format (IMU)
      // -------------------------------------------------------------
      uint16_t sensor_base_rate;

      if(commands_3dm::imuGetBaseRate(*device_, &sensor_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get sensor base rate format!");

      RCLCPP_INFO(this->get_logger(), "Sensor base rate: %d", sensor_base_rate);

      const uint16_t sensor_sample_rate = SENSOR_SAMPLE_RATE; // Hz
      const uint16_t sensor_decimation = sensor_base_rate / sensor_sample_rate;

      std::array<DescriptorRate, 3> sensor_descriptors = {{
        { data_sensor::DATA_TIME_STAMP_GPS, sensor_decimation },
        { data_sensor::DATA_ACCEL_SCALED,   sensor_decimation },
        { data_sensor::DATA_GYRO_SCALED,    sensor_decimation },
      }};

      if(commands_3dm::writeImuMessageFormat(*device_, sensor_descriptors.size(), sensor_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor message format!");

      // -------------------------------------------------------------
      // Setup FILTER data format
      // -------------------------------------------------------------
      uint16_t filter_base_rate;

      if(commands_3dm::filterGetBaseRate(*device_, &filter_base_rate) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not get filter base rate format!");

      const uint16_t filter_sample_rate = FILTER_SAMPLE_RATE; // Hz
      const uint16_t filter_decimation = filter_base_rate / filter_sample_rate;

      std::array<DescriptorRate, 5> filter_descriptors = {{
        { data_filter::DATA_FILTER_TIMESTAMP,               filter_decimation },
        { data_filter::DATA_FILTER_STATUS,                  filter_decimation },
        { data_filter::DATA_ATT_EULER_ANGLES,               filter_decimation },
        { data_filter::DATA_COMPENSATED_ANGULAR_RATE,       filter_decimation },
        { data_filter::DATA_COMPENSATED_ACCELERATION,       filter_decimation },
      }};

      if(commands_3dm::writeFilterMessageFormat(*device_, filter_descriptors.size(), filter_descriptors.data()) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter message format!");

      // Sensor-to-vehicle rotation
      if(commands_filter::writeSensorToVehicleRotationEuler(*device_,
                                                           sensor_to_vehicle_rotation_euler[0],
                                                           sensor_to_vehicle_rotation_euler[1],
                                                           sensor_to_vehicle_rotation_euler[2]) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set sensor-2-vehicle rotation!");

      // Enable filter auto-initialization
      if(commands_filter::writeAutoInitControl(*device_, 1) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not set filter autoinit control!");

      // Reset the filter
      if(commands_filter::reset(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not reset the filter!");

      // -------------------------------------------------------------
      // Register data callbacks
      // IMPORTANT: DispatchHandler MUST outlive the device updates,
      // so they must be class members (NOT local variables).
      // -------------------------------------------------------------

      // Sensor Data
      device_->registerExtractor(sensor_data_handlers_[0], &sensor_gps_time);
      device_->registerExtractor(sensor_data_handlers_[1], &sensor_accel);
      device_->registerExtractor(sensor_data_handlers_[2], &sensor_gyro);

      // Filter Data
      device_->registerExtractor(filter_data_handlers_[0], &filter_gps_time);
      device_->registerExtractor(filter_data_handlers_[1], &filter_status);
      device_->registerExtractor(filter_data_handlers_[2], &filter_euler_angles);
      device_->registerExtractor(filter_data_handlers_[3], &filter_comp_angular_rate);
      device_->registerExtractor(filter_data_handlers_[4], &filter_comp_accel);

      // Resume the device
      if(commands_base::resume(*device_) != CmdResult::ACK_OK)
        exit_gracefully("ERROR: Could not resume the device!");

      RCLCPP_INFO(this->get_logger(), "Sensor is configured... waiting for filter to enter running mode.");

      // -------------------------------------------------------------
      // ROS2 publisher + timers
      // -------------------------------------------------------------
      publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);

      publish_timer_ = this->create_wall_timer(
        publish_period_,
        std::bind(&IMUpublisher::publish_timer_callback, this)
      );

      print_info_timer_ = this->create_wall_timer(
        print_info_period_,
        std::bind(&IMUpublisher::print_info_timer_callback, this)
      );
    }

  private:
    void exit_gracefully(const char *message){
      RCLCPP_INFO(this->get_logger(), "node dead: %s", message);
      rclcpp::shutdown();
      exit(0);
    }

    double deg2rad(double deg){
      return deg * M_PI / 180.0;
    }

    void print_info_timer_callback(){
      RCLCPP_INFO(this->get_logger(), "published %d IMU messages", publish_counter);
      publish_counter = 0;
      return;
    }

    void publish_timer_callback(){
      // -------------------------------------------------------------
      // Update device (extractors fill data_* structs)
      // -------------------------------------------------------------
      device_->update();

      publish_counter++;

      // -------------------------------------------------------------
      // Check Filter State
      // -------------------------------------------------------------
      if((!filter_state_running) &&
         ((filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_ERROR) ||
          (filter_status.filter_state == data_filter::FilterMode::GX5_RUN_SOLUTION_VALID)))
      {
        RCLCPP_INFO(this->get_logger(), "NOTE: Filter has entered running mode");
        filter_state_running = true;
      }

      // If filter is not running yet, do not publish orientation-based message
      if(!filter_state_running)
        return;

      // -------------------------------------------------------------
      // Build quaternion from Euler angles
      // -------------------------------------------------------------
      q = Eigen::AngleAxisd(filter_euler_angles.yaw,   Eigen::Vector3d::UnitZ()) *
          Eigen::AngleAxisd(filter_euler_angles.pitch, Eigen::Vector3d::UnitY()) *
          Eigen::AngleAxisd(filter_euler_angles.roll,  Eigen::Vector3d::UnitX());
      q.normalize();

      auto imu_msg = sensor_msgs::msg::Imu();

      // --- header ---
      imu_msg.header.stamp = this->get_clock()->now();
      imu_msg.header.frame_id = "imu_link";

      // --- orientation  ---
      imu_msg.orientation.x = q.x();
      imu_msg.orientation.y = q.y();
      imu_msg.orientation.z = q.z();
      imu_msg.orientation.w = q.w();

      // --- angular velocity ---
      imu_msg.angular_velocity.x = filter_comp_angular_rate.gyro[0];
      imu_msg.angular_velocity.y = filter_comp_angular_rate.gyro[1];
      imu_msg.angular_velocity.z = filter_comp_angular_rate.gyro[2];

      // --- linear acceleration ---
      imu_msg.linear_acceleration.x = filter_comp_accel.accel[0];
      imu_msg.linear_acceleration.y = filter_comp_accel.accel[1];
      imu_msg.linear_acceleration.z = filter_comp_accel.accel[2];

      publisher_->publish(imu_msg);
    }

  private:
    std::chrono::milliseconds publish_period_;
    std::chrono::milliseconds print_info_period_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    rclcpp::TimerBase::SharedPtr print_info_timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

    std::unique_ptr<ExampleUtils> utils_;
    std::string port_;
    int baud_;
    mip::DeviceInterface* device_;

    // Sensor-to-vehicle frame rotation (Euler Angles)
    static constexpr float sensor_to_vehicle_rotation_euler[3] = {0.0, 0.0, 0.0};

    // Device data stores
    data_sensor::GpsTimestamp sensor_gps_time;
    data_sensor::ScaledAccel  sensor_accel;
    data_sensor::ScaledGyro   sensor_gyro;

    data_filter::Timestamp        filter_gps_time;
    data_filter::Status           filter_status;
    data_filter::EulerAngles      filter_euler_angles;
    data_filter::CompAngularRate  filter_comp_angular_rate;
    data_filter::CompAccel        filter_comp_accel;

    bool filter_state_running;
    Eigen::Quaterniond q;
    int publish_counter;

    // Dispatch handlers MUST be members (lifetime must cover device_->update())
    std::array<mip::DispatchHandler, 3> sensor_data_handlers_;
    std::array<mip::DispatchHandler, 5> filter_data_handlers_;
};

////////////////////////////////////////////////////////////////////////////////
// Main Function
////////////////////////////////////////////////////////////////////////////////

int main(int argc, const char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IMUpublisher>());
  rclcpp::shutdown();
  return 0;
}
