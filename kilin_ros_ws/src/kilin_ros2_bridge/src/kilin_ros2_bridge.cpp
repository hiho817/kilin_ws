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
#include "kilin_msgs/msg/power_cmd_stamped.hpp"
#include "kilin_msgs/msg/power_state_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Global mutexes for thread safety
std::mutex mutex_ros_motor_state;
std::mutex mutex_grpc_motor_cmd;
std::mutex mutex_grpc_power_cmd;
std::mutex mutex_ros_power_state;

// Global ROS message objects
kilin_msgs::msg::MotorCmdStamped   ros_motor_cmd;
kilin_msgs::msg::MotorStateStamped ros_motor_state;
kilin_msgs::msg::PowerCmdStamped   ros_power_cmd;
kilin_msgs::msg::PowerStateStamped ros_power_state;

// Global gRPC message objects
motor_msg::MotorCmdStamped  grpc_motor_cmd;
motor_msg::MotorStateStamped grpc_motor_state;
power_msg::PowerCmdStamped  grpc_power_cmd;
power_msg::PowerStateStamped grpc_power_state;

// Global pointer to the gRPC publisher for motor commands
core::Publisher<motor_msg::MotorCmdStamped>* grpc_motor_cmd_pub = nullptr;

// Global ROS publisher for motor state messages
rclcpp::Publisher<kilin_msgs::msg::MotorStateStamped>::SharedPtr ros_motor_state_pub = nullptr;

// Global pointer to the gRPC publisher for power commands
core::Publisher<power_msg::PowerCmdStamped>* grpc_power_cmd_pub = nullptr;

// Global ROS publisher for power state messages
rclcpp::Publisher<kilin_msgs::msg::PowerStateStamped>::SharedPtr ros_power_state_pub = nullptr;

//
// Callback: When a ROS motor command message is received, convert it to a gRPC message and publish via gRPC.
//
void ros_motor_cmd_cb(const kilin_msgs::msg::MotorCmdStamped::SharedPtr msg) {
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

    RCLCPP_INFO(rclcpp::get_logger("ros2_bridge"),
                "ROS->gRPC: got cmd seq=%u", msg->header.seq);

    //TODO: Should be update with the new message structure.
    for (int i = 0; i < 4; ++i) {
        // grpc_motor_modules[i]->set_theta(ros_motor_modules[i].theta);
        // grpc_motor_modules[i]->set_beta(ros_motor_modules[i].beta);
        // grpc_motor_modules[i]->set_kp(ros_motor_modules[i].kp);
        // grpc_motor_modules[i]->set_ki(ros_motor_modules[i].ki);
        // grpc_motor_modules[i]->set_kd(ros_motor_modules[i].kd);
        // grpc_motor_modules[i]->set_torque_r(ros_motor_modules[i].torque_r);
        // grpc_motor_modules[i]->set_torque_l(ros_motor_modules[i].torque_l);

        const auto& src = ros_motor_modules[i];  
        auto* dst = grpc_motor_modules[i]; 

        // hip
        auto* hip = dst->mutable_hip();
        hip->set_position(src.hip.position);
        hip->set_kp(src.hip.kp);
        hip->set_ki(src.hip.ki);
        hip->set_kd(src.hip.kd);
        hip->set_torque(src.hip.torque);
        hip->set_velocity(src.hip.velocity);

        // steering
        auto* steering = dst->mutable_steering();
        steering->set_position(src.steering.position);
        steering->set_kp(src.steering.kp);
        steering->set_ki(src.steering.ki);
        steering->set_kd(src.steering.kd);
        steering->set_torque(src.steering.torque);
        steering->set_velocity(src.steering.velocity);

        // hub
        auto* hub = dst->mutable_hub();
        hub->set_position(src.hub.position);
        hub->set_kp(src.hub.kp);
        hub->set_ki(src.hub.ki);
        hub->set_kd(src.hub.kd);
        hub->set_torque(src.hub.torque);
        hub->set_velocity(src.hub.velocity);
    }

    // Copy header information.
    // Now using header.time instead of header.stamp.
    grpc_motor_cmd.mutable_header()->set_seq(ros_motor_cmd.header.seq);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_sec(ros_motor_cmd.header.time.sec);
    grpc_motor_cmd.mutable_header()->mutable_stamp()->set_usec(ros_motor_cmd.header.time.nanosec);

    // Publish the converted motor command via gRPC.
    if (grpc_motor_cmd_pub != nullptr) {
        grpc_motor_cmd_pub->publish(grpc_motor_cmd);
        // RCLCPP_INFO(rclcpp::get_logger("ros2_bridge"),
        //             "ROS->gRPC: published cmd seq=%u", msg->header.seq);
    }
}

//
// Callback: When a gRPC motor state message is received, convert it and publish it on the ROS topic.
//
// Note: The subscribe function expects a callback that takes the message by value.
void grpc_motor_state_cb(motor_msg::MotorStateStamped state) {
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
    for (int i = 0; i < 4; ++i) {
    //   ros_motor_modules[i]->theta      = grpc_motor_modules[i]->theta();
    //   ros_motor_modules[i]->beta       = grpc_motor_modules[i]->beta();
    //   ros_motor_modules[i]->velocity_r = grpc_motor_modules[i]->velocity_r();
    //   ros_motor_modules[i]->velocity_l = grpc_motor_modules[i]->velocity_l();
    //   ros_motor_modules[i]->torque_r   = grpc_motor_modules[i]->torque_r();
    //   ros_motor_modules[i]->torque_l   = grpc_motor_modules[i]->torque_l();
        const auto* src = grpc_motor_modules[i];  // motor_msg::LegState*
        auto* dst = ros_motor_modules[i];   // kilin_msgs::msg::LegState*

        // hip
        dst->hip.position = src->hip().position();
        dst->hip.velocity = src->hip().velocity();
        dst->hip.torque   = src->hip().torque();

        // steering
        dst->steering.position = src->steering().position();
        dst->steering.velocity = src->steering().velocity();
        dst->steering.torque   = src->steering().torque();

        // hub
        dst->hub.position = src->hub().position();
        dst->hub.velocity = src->hub().velocity();
        dst->hub.torque   = src->hub().torque();
    }

    // Copy header information.
    // The ROS message header now uses .time, so assign accordingly.
    ros_motor_state.header.seq = grpc_motor_state.header().seq();
    ros_motor_state.header.time.sec = grpc_motor_state.header().stamp().sec();
    ros_motor_state.header.time.nanosec = grpc_motor_state.header().stamp().usec() * 1000;

    // RCLCPP_INFO(rclcpp::get_logger("ros2_bridge"),
    //           "gRPC->ROS: got state seq=%u",
    //           grpc_motor_state.header().seq());

    // Publish the converted motor state on the ROS topic.
    if (ros_motor_state_pub) {
        ros_motor_state_pub->publish(ros_motor_state);
    }
}

// Callback: When a ROS power command message is received, convert it to a gRPC message and publish via gRPC.
void ros_power_cmd_cb(const kilin_msgs::msg::PowerCmdStamped::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_grpc_power_cmd);
    ros_power_cmd = *msg;

    RCLCPP_INFO(rclcpp::get_logger("ros2_bridge"),
                "ROS->gRPC: got power cmd seq=%u", msg->header.seq);

    // Header
    grpc_power_cmd.mutable_header()->set_seq(ros_power_cmd.header.seq);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_sec(ros_power_cmd.header.time.sec);
    grpc_power_cmd.mutable_header()->mutable_stamp()->set_usec(ros_power_cmd.header.time.nanosec / 1000);

    // Payload
    grpc_power_cmd.set_digital(ros_power_cmd.digital);
    grpc_power_cmd.set_signal(ros_power_cmd.signal);
    grpc_power_cmd.set_power(ros_power_cmd.power);
    grpc_power_cmd.set_clean(ros_power_cmd.clean);
    grpc_power_cmd.set_trigger(ros_power_cmd.trigger);
    grpc_power_cmd.set_steering_cali(ros_power_cmd.steering_cali);
    grpc_power_cmd.set_robot_mode(static_cast<power_msg::ROBOTMODE>(ros_power_cmd.robot_mode));

    // Publish via gRPC
    if (grpc_power_cmd_pub != nullptr) {
        grpc_power_cmd_pub->publish(grpc_power_cmd);
    }
}

// Callback: When a gRPC power state message is received, convert it and publish it on the ROS topic.
void grpc_power_state_cb(power_msg::PowerStateStamped m) {
    std::lock_guard<std::mutex> lock(mutex_ros_power_state);
    grpc_power_state = m;

    // Header
    ros_power_state.header.seq = m.header().seq();
    ros_power_state.header.time.sec = m.header().stamp().sec();
    ros_power_state.header.time.nanosec = m.header().stamp().usec() * 1000;

    // Booleans
    ros_power_state.digital = m.digital();
    ros_power_state.signal = m.signal();
    ros_power_state.power = m.power();
    ros_power_state.clean = m.clean();
    ros_power_state.robot_mode = static_cast<int32_t>(m.robot_mode());

    // Voltages / Currents
    ros_power_state.v_0 = m.v_0();  ros_power_state.i_0 = m.i_0();
    ros_power_state.v_1 = m.v_1();  ros_power_state.i_1 = m.i_1();
    ros_power_state.v_2 = m.v_2();  ros_power_state.i_2 = m.i_2();
    ros_power_state.v_3 = m.v_3();  ros_power_state.i_3 = m.i_3();
    ros_power_state.v_4 = m.v_4();  ros_power_state.i_4 = m.i_4();
    ros_power_state.v_5 = m.v_5();  ros_power_state.i_5 = m.i_5();
    ros_power_state.v_6 = m.v_6();  ros_power_state.i_6 = m.i_6();
    ros_power_state.v_7 = m.v_7();  ros_power_state.i_7 = m.i_7();
    ros_power_state.v_8 = m.v_8();  ros_power_state.i_8 = m.i_8();
    ros_power_state.v_9 = m.v_9();  ros_power_state.i_9 = m.i_9();
    ros_power_state.v_10 = m.v_10(); ros_power_state.i_10 = m.i_10();
    ros_power_state.v_11 = m.v_11(); ros_power_state.i_11 = m.i_11();

    if (ros_power_state_pub) {
        ros_power_state_pub->publish(ros_power_state);
    }
}

//
// Main function
//
int main(int argc, char * argv[]) {
    // Initialize the ROS 2 system.
    rclcpp::init(argc, argv);

    // Create a ROS 2 node.
    auto node = std::make_shared<rclcpp::Node>("ros2_bridge");
    RCLCPP_INFO(node->get_logger(), "ROS2 Bridge Started");

    // Create a ROS 2 subscriber for motor command messages.
    auto ros_motor_cmd_sub = node->create_subscription<kilin_msgs::msg::MotorCmdStamped>(
        "motor/command", 10, ros_motor_cmd_cb);
    
    // Create a ROS 2 subscriber for power command messages.
    auto ros_power_cmd_sub = node->create_subscription<kilin_msgs::msg::PowerCmdStamped>(
        "power/command", 10, ros_power_cmd_cb);

    // Create a ROS 2 publisher for motor state messages.
    ros_motor_state_pub = node->create_publisher<kilin_msgs::msg::MotorStateStamped>("motor/state", 10);

    // Create a ROS 2 publisher for power state messages.
    ros_power_state_pub = node->create_publisher<kilin_msgs::msg::PowerStateStamped>("power/state", 10);

    // --- Set up the gRPC side ---
    // Create a NodeHandler for gRPC operations.
    core::NodeHandler nh_;

    const char* ep = std::getenv("CORE_MASTER_ADDR");
    std::cout << "[INFO] NodeHandler endpoint=" << (ep ? ep : "(unset)") << std::endl;

    // Subscribe to gRPC motor state messages.
    // Capture the subscriber by reference (to avoid copying the non-copyable object).
    auto &grpc_motor_state_sub = nh_.subscribe<motor_msg::MotorStateStamped>(
        "motor/state", 1000, grpc_motor_state_cb);

    // Subscribe to gRPC power state messages.
    auto &grpc_power_state_sub = nh_.subscribe<power_msg::PowerStateStamped>(
        "power/state", 1000, grpc_power_state_cb);

    // Advertise the gRPC publisher for motor command messages.
    grpc_motor_cmd_pub = &(nh_.advertise<motor_msg::MotorCmdStamped>("motor/command"));
    
    // Advertise the gRPC publisher for power command messages.
    grpc_power_cmd_pub = &(nh_.advertise<power_msg::PowerCmdStamped>("power/command"));

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
