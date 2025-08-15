#include <iostream>
#include <mutex>
#include <vector>
#include <sys/time.h>
#include "rclcpp/rclcpp.hpp"

#include "NodeHandler.h"
#include "Motor.pb.h"

// Global mutexes for thread safety
std::mutex mutex_motor_state;

// Global gRPC message objects
motor_msg::MotorCmdStamped         motor_cmd;
motor_msg::MotorStateStamped       motor_state;

// Callback for motor command messages
void motor_cmd_cb(const motor_msg::MotorCmdStamped cmd) {
    std::lock_guard<std::mutex> lock(mutex_motor_state);

    // Create vectors for conversion - use auto to let compiler deduce the correct type
    auto motor_states = std::vector<motor_msg::LegState*>{
        motor_state.mutable_module_a(),
        motor_state.mutable_module_b(),
        motor_state.mutable_module_c(),
        motor_state.mutable_module_d()
    };

    auto motor_cmds = std::vector<const motor_msg::LegCmd*>{
        &cmd.module_a(),
        &cmd.module_b(),
        &cmd.module_c(),
        &cmd.module_d()
    };
    
    //TODO: Should be update with the new message structure.
    // std::cout << "TB_A: (" << motor_cmds[0]->theta() << ", " << motor_cmds[0]->beta() << "); " << std::endl
    //           << "TB_B: (" << motor_cmds[1]->theta() << ", " << motor_cmds[1]->beta() << "); " << std::endl
    //           << "TB_C: (" << motor_cmds[2]->theta() << ", " << motor_cmds[2]->beta() << "); " << std::endl
    //           << "TB_D: (" << motor_cmds[3]->theta() << ", " << motor_cmds[3]->beta() << "); " << std::endl << std::endl;
    //TODO: Should be update with the new message structure.
    // for (int i = 0; i < 4; i++) {
    //     motor_states[i]->set_theta(motor_cmds[i]->theta());
    //     motor_states[i]->set_beta(motor_cmds[i]->beta());
    //     motor_states[i]->set_velocity_r(1);
    //     motor_states[i]->set_velocity_l(1);
    //     motor_states[i]->set_torque_r(1);
    //     motor_states[i]->set_torque_l(1);
    // }

    // Get current time and update header (using 'time' field)
    timeval currentTime;
    gettimeofday(&currentTime, nullptr);
    motor_state.mutable_header()->set_seq(cmd.header().seq());
    motor_state.mutable_header()->mutable_stamp()->set_sec(currentTime.tv_sec);
    // Convert microseconds to nanoseconds
    motor_state.mutable_header()->mutable_stamp()->set_usec(currentTime.tv_usec);
}

int main(int argc, char **argv) {
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // (Optional) Create a ROS 2 node if needed for logging, parameters, etc.
    auto node = std::make_shared<rclcpp::Node>("corgi_virtual_agent");
    RCLCPP_INFO(node->get_logger(), "Corgi Virtual Agent Started");

    // Set up the gRPC side using NodeHandler
    core::NodeHandler nh_;

    // Advertise gRPC publishers for state messages
    auto &motor_state_pub = nh_.advertise<motor_msg::MotorStateStamped>("motor/state");

    // Subscribe to gRPC command messages
    auto &motor_cmd_sub = nh_.subscribe<motor_msg::MotorCmdStamped>("motor/command", 1000, motor_cmd_cb);

    // Main loop: process gRPC events and publish state messages at 1000 Hz
    rclcpp::WallRate rate(1000);
    while (rclcpp::ok()) {
        core::spinOnce();

        {
            std::lock_guard<std::mutex> lock(mutex_motor_state);
            motor_state_pub.publish(motor_state);
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
