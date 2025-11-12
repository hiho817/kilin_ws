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
        // Subscribe to joystick topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&KilinJoystickInterface::joy_callback, this, _1));

        // Publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);

        // Register signal handler for Ctrl+C
        instance_ = this;
        std::signal(SIGINT, signal_handler);

        RCLCPP_INFO(this->get_logger(),
            "Kilin joystick interface started (Left stick → vx, vy | Right stick → ω)");
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
            // Wait briefly to ensure message delivery
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        rclcpp::shutdown();
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        const double Vmax = 0.8;       // Maximum linear velocity [m/s]
        const double Wmax = 1.5;       // Maximum angular velocity [rad/s]
        const double deadzone = 0.15;  // Joystick deadzone

        // ------------------------------------------
        // Left stick controls linear velocity
        // ------------------------------------------
        double vx = msg->axes[1];  // Forward/backward
        double vy = msg->axes[0];  // Left/right

        // ------------------------------------------
        // Right stick controls angular velocity
        // ------------------------------------------
        double omega = msg->axes[3];  // Right stick horizontal axis

        // ------------------------------------------
        // Apply deadzone
        // ------------------------------------------
        if (std::fabs(vx) < deadzone) vx = 0.0;
        if (std::fabs(vy) < deadzone) vy = 0.0;
        if (std::fabs(omega) < deadzone) omega = 0.0;

        // ------------------------------------------
        // Normalize vector magnitude (avoid >1.0)
        // ------------------------------------------
        double norm = std::sqrt(vx * vx + vy * vy);
        if (norm > 1e-6 && norm > 1.0) {
            vx /= norm;
            vy /= norm;
        }

        // ------------------------------------------
        // Publish Twist message
        // ------------------------------------------
        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx * Vmax;
        twist.linear.y = vy * Vmax;
        twist.angular.z = omega * Wmax;

        cmd_pub_->publish(twist);

        // Debug output
        RCLCPP_DEBUG(this->get_logger(), "vx=%.2f, vy=%.2f, ω=%.2f",
                     twist.linear.x, twist.linear.y, twist.angular.z);
    }

    void send_zero_command() {
        // Publish a zero velocity command
        geometry_msgs::msg::Twist stop;
        stop.linear.x = 0.0;
        stop.linear.y = 0.0;
        stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    static KilinJoystickInterface *instance_;
};

// Initialize static instance pointer
KilinJoystickInterface *KilinJoystickInterface::instance_ = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinJoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
