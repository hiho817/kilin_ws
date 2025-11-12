#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <csignal>
#include <thread>

using std::placeholders::_1;

class KilinJoystickInterface : public rclcpp::Node {
public:
    KilinJoystickInterface() : Node("kilin_joystick") {
        // ------------------------------------------
        // Declare configurable parameters
        // ------------------------------------------
        Vmax_ = this->declare_parameter<double>("vmax", 0.8);      // Max linear velocity [m/s]
        Wmax_ = this->declare_parameter<double>("wmax", 1.5);      // Max angular velocity [rad/s]
        deadzone_ = this->declare_parameter<double>("deadzone", 0.15);  // Joystick deadzone

        // ------------------------------------------
        // ROS Interfaces
        // ------------------------------------------
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&KilinJoystickInterface::joy_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);

        // Register signal handler for Ctrl+C
        instance_ = this;
        std::signal(SIGINT, signal_handler);

        RCLCPP_INFO(this->get_logger(),
            "Kilin joystick interface started (Left stick → vx, vy | Right stick → ω)");
        RCLCPP_INFO(this->get_logger(),
            "Parameters: Vmax=%.2f, Wmax=%.2f, deadzone=%.2f", Vmax_, Wmax_, deadzone_);
    }

    ~KilinJoystickInterface() {
        // Ensure zero command is sent when node is destructed
        send_zero_command();
    }

    static void signal_handler(int) {
        if (instance_) {
            instance_->send_zero_command();
            RCLCPP_WARN(instance_->get_logger(),
                        "Joystick node stopped — published zero command before shutdown.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        rclcpp::shutdown();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // ------------------------------------------
        // Left stick → linear velocity
        // ------------------------------------------
        double vx = msg->axes[1];  // Forward/backward
        double vy = msg->axes[0];  // Left/right

        // ------------------------------------------
        // Right stick → angular velocity
        // ------------------------------------------
        double omega = msg->axes[3];  // Right stick horizontal axis

        // ------------------------------------------
        // Apply deadzone
        // ------------------------------------------
        auto apply_deadzone = [&](double val) {
            if (std::fabs(val) <= deadzone_) return 0.0;
            double sign = (val > 0.0) ? 1.0 : -1.0;
            // Smooth deadzone transition
            return sign * (std::fabs(val) - deadzone_) / (1.0 - deadzone_);
        };

        vx = apply_deadzone(vx);
        vy = apply_deadzone(vy);
        omega = apply_deadzone(omega);

        // ------------------------------------------
        // Normalize magnitude (avoid >1.0)
        // ------------------------------------------
        double norm = std::sqrt(vx * vx + vy * vy);
        if (norm > 1e-6 && norm > 1.0) {
            vx /= norm;
            vy /= norm;
        }

        // ------------------------------------------
        // Publish Twist command
        // ------------------------------------------
        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx * Vmax_;
        twist.linear.y = vy * Vmax_;
        twist.angular.z = omega * Wmax_;

        cmd_pub_->publish(twist);

        RCLCPP_DEBUG(this->get_logger(), "vx=%.2f, vy=%.2f, ω=%.2f",
                     twist.linear.x, twist.linear.y, twist.angular.z);
    }

    void send_zero_command() {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.linear.y = 0.0;
        stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Parameters
    double Vmax_;
    double Wmax_;
    double deadzone_;

    // Static instance for signal handling
    static KilinJoystickInterface *instance_;
};

// Static member definition
KilinJoystickInterface *KilinJoystickInterface::instance_ = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinJoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
