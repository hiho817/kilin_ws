#include "rclcpp/rclcpp.hpp"

#include <iostream>
#include <mutex>
#include <vector>

// gRPC headers
#include "NodeHandler.h"
#include "Motor.pb.h"
#include "Power.pb.h"

// ROS messages (assumed to be generated in package kilin_msgs)
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/motor_state_stamped.hpp"
#include "kilin_msgs/msg/trigger_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Global mutexes for thread safety
std::mutex mutex_ros_motor_state;
std::mutex mutex_grpc_motor_cmd;

// Global ROS message objects
kilin_msgs::msg::MotorCmdStamped   ros_motor_cmd;
kilin_msgs::msg::MotorStateStamped ros_motor_state;

// Global gRPC message objects
motor_msg::MotorCmdStamped  grpc_motor_cmd;
motor_msg::MotorStateStamped grpc_motor_state;

// Global pointer to the gRPC publisher for motor commands
core::Publisher<motor_msg::MotorCmdStamped>* grpc_motor_cmd_pub = nullptr;

// Global ROS publisher for motor state messages
rclcpp::Publisher<kilin_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub = nullptr;

//
// Callback: When a ROS motor command message is received, convert it to a gRPC message and publish via gRPC.
//
void ros_motor_cmd_cb(const kilin_msgs::msg::MotorCmdStamped::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_grpc_motor_cmd);
  ros_motor_cmd = *msg;  // Copy the incoming ROS message

  // Convert the ROS motor command into a gRPC motor command.
  // (Assuming each motor command contains four modules.)
  std::vector<motor_msg::LegCmd*> grpc_motor_modules = {
      grpc_motor_cmd.mutable_module_a(),
      grpc_motor_cmd.mutable_module_b(),
      grpc_motor_cmd.mutable_module_c(),
      grpc_motor_cmd.mutable_module_d()
  };

  // Assume the ROS message has corresponding modules.
  std::vector<kilin_msgs::msg::LegCmd> ros_motor_modules = {
      ros_motor_cmd.module_a,
      ros_motor_cmd.module_b,
      ros_motor_cmd.module_c,
      ros_motor_cmd.module_d
  };

  //TODO: Should be update with the new message structure.
  // for (int i = 0; i < 4; ++i) {
  //   grpc_motor_modules[i]->set_theta(ros_motor_modules[i].theta);
  //   grpc_motor_modules[i]->set_beta(ros_motor_modules[i].beta);
  //   grpc_motor_modules[i]->set_kp(ros_motor_modules[i].kp);
  //   grpc_motor_modules[i]->set_ki(ros_motor_modules[i].ki);
  //   grpc_motor_modules[i]->set_kd(ros_motor_modules[i].kd);
  //   grpc_motor_modules[i]->set_torque_r(ros_motor_modules[i].torque_r);
  //   grpc_motor_modules[i]->set_torque_l(ros_motor_modules[i].torque_l);
  // }

  // Copy header information.
  // Now using header.time instead of header.stamp.
  grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
  grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.time.sec);
  grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.time.nanosec);

  // Publish the converted motor command via gRPC.
  if (grpc_motor_cmd_pub != nullptr) {
    grpc_motor_cmd_pub->publish(grpc_motor_cmd);
  }
}

//
// Callback: When a gRPC motor state message is received, convert it and publish it on the ROS topic.
//
// Note: The subscribe function expects a callback that takes the message by value.
void grpc_motor_state_cb(motor_msg::MotorStateStamped state)
{
  std::lock_guard<std::mutex> lock(mutex_ros_motor_state);
  grpc_motor_state = state;  // Copy the incoming gRPC state

  // Convert each module's state.
  std::vector<const motor_msg::LegState*> grpc_motor_modules = {
      &grpc_motor_state.module_a(),
      &grpc_motor_state.module_b(),
      &grpc_motor_state.module_c(),
      &grpc_motor_state.module_d()
  };

  std::vector<kilin_msgs::msg::LegState*> ros_motor_modules = {
      &ros_motor_state.module_a,
      &ros_motor_state.module_b,
      &ros_motor_state.module_c,
      &ros_motor_state.module_d
  };

  //TODO: Should be update with the new message structure.
  // for (int i = 0; i < 4; ++i) {
  //   ros_motor_modules[i]->theta      = grpc_motor_modules[i]->theta();
  //   ros_motor_modules[i]->beta       = grpc_motor_modules[i]->beta();
  //   ros_motor_modules[i]->velocity_r = grpc_motor_modules[i]->velocity_r();
  //   ros_motor_modules[i]->velocity_l = grpc_motor_modules[i]->velocity_l();
  //   ros_motor_modules[i]->torque_r   = grpc_motor_modules[i]->torque_r();
  //   ros_motor_modules[i]->torque_l   = grpc_motor_modules[i]->torque_l();
  // }

  // Copy header information.
  // The ROS message header now uses .time, so assign accordingly.
  ros_motor_state.header.seq = grpc_motor_state.header().seq();
  ros_motor_state.header.time.sec = grpc_motor_state.header().stamp().sec();
  ros_motor_state.header.time.nanosec = grpc_motor_state.header().stamp().usec() * 1000;

  // Publish the converted motor state on the ROS topic.
  if (ros_motor_state_pub) {
    ros_motor_state_pub->publish(ros_motor_state);
  }
}

//
// Main function
//
int main(int argc, char * argv[])
{
  // Initialize the ROS 2 system.
  rclcpp::init(argc, argv);

  // Create a ROS 2 node.
  auto node = std::make_shared<rclcpp::Node>("ros2_bridge");
  RCLCPP_INFO(node->get_logger(), "ROS2 Bridge Started");

  // Create a ROS 2 subscriber for motor command messages.
  auto ros_motor_cmd_sub = node->create_subscription<kilin_msgs::msg::MotorCmdStamped>(
      "motor/command", 10, ros_motor_cmd_cb);

  // Create a ROS 2 publisher for motor state messages.
  ros_motor_state_pub = node->create_publisher<kilin_msgs::msg::MotorStateStamped>("motor/state", 10);

  // --- Set up the gRPC side ---
  // Create a NodeHandler for gRPC operations.
  core::NodeHandler nh_;

  // Subscribe to gRPC motor state messages.
  // Capture the subscriber by reference (to avoid copying the non-copyable object).
  auto &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>(
      "motor/state", 1000, grpc_motor_state_cb);

  // Advertise the gRPC publisher for motor command messages.
  grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));

  // Main loop: process both ROS 2 and gRPC events.
  rclcpp::WallRate loop_rate(1000);  // 1000 Hz loop rate
  while (rclcpp::ok()) {
    // Process available ROS 2 events.
    rclcpp::spin_some(node);
    // Process available gRPC events.
    core::spinOnce();
    loop_rate.sleep();
  }

  // Shutdown the ROS 2 system.
  rclcpp::shutdown();
  return 0;
}
