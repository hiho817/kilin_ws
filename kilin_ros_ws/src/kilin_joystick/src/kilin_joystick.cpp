#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>

using std::placeholders::_1;

class KilinJoystickInterface : public rclcpp::Node {
public:
    KilinJoystickInterface() : Node("kilin_joystick") {
        // Subscribe to joystick topic
        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 10, std::bind(&KilinJoystickInterface::joy_callback, this, _1));

        // Publish velocity commands
        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(),
        "Kilin joystick interface started (Left stick → vx, vy | Right stick → ω)");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        const double Vmax = 0.8;      // Max linear velocity [m/s]
        const double Wmax = 1.5;      // Max angular velocity [rad/s]
        const double deadzone = 0.1;  // Joystick deadzone

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

        double norm = std::sqrt(vx * vx + vy * vy);
        if (norm > 1e-6){
            if (norm > 1.0) {
                vx /= norm;
                vy /= norm;
            }
        }

        // ------------------------------------------
        // Publish Twist message
        // ------------------------------------------
        geometry_msgs::msg::Twist twist;
        twist.linear.x = vx * Vmax;
        twist.linear.y = vy * Vmax;
        twist.angular.z = omega * Wmax;

        cmd_pub->publish(twist);

        // Debug output
        RCLCPP_DEBUG(this->get_logger(), "vx=%.2f, vy=%.2f, ω=%.2f",
                    twist.linear.x, twist.linear.y, twist.angular.z);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinJoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
