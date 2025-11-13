#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <thread>
#include <csignal>

using std::placeholders::_1;

class KilinJoystickInterface : public rclcpp::Node {
public:
    KilinJoystickInterface() : Node("kilin_joystick") {

        // -----------------------------
        // Declare parameters
        // -----------------------------
        Vmax_ = this->declare_parameter<double>("vmax", 0.8);
        Wmax_ = this->declare_parameter<double>("wmax", 1.5);
        deadzone_ = this->declare_parameter<double>("deadzone", 0.15);

        // -----------------------------
        // ROS interfaces
        // -----------------------------
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&KilinJoystickInterface::joy_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);

        // -----------------------------
        // Timer: fixed 100 Hz output
        // -----------------------------
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),   // 10 ms = 100 Hz
            std::bind(&KilinJoystickInterface::timer_callback, this)
        );

        // Register signal handler
        instance_ = this;
        std::signal(SIGINT, signal_handler);

        RCLCPP_INFO(this->get_logger(), "Joystick node started @ 100Hz output.");
    }

    ~KilinJoystickInterface() {
        send_zero_command();
    }

    static void signal_handler(int) {
        if (instance_) {
            instance_->send_zero_command();
            RCLCPP_WARN(instance_->get_logger(), "Sent zero command before shutdown");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        rclcpp::shutdown();
    }

private:

    // --------------------------------------
    // Joystick callback (NOT publishing!)
    // Only update state
    // --------------------------------------
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        auto apply_deadzone = [&](double val) {
            if (std::fabs(val) <= deadzone_) return 0.0;
            double s = (val > 0.0) ? 1.0 : -1.0;
            return s * (std::fabs(val) - deadzone_) / (1.0 - deadzone_);
        };

        // Left stick
        latest_vx_ = apply_deadzone(msg->axes[1]);
        latest_vy_ = apply_deadzone(msg->axes[0]);

        // Right stick (ω)
        latest_omega_ = apply_deadzone(msg->axes[3]);
    }

    // --------------------------------------
    // Timer: Publish at EXACT 100 Hz
    // --------------------------------------
    void timer_callback() {
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = latest_vx_ * Vmax_;
        twist.linear.y  = latest_vy_ * Vmax_;
        twist.angular.z = latest_omega_ * Wmax_;

        cmd_pub_->publish(twist);
    }

    // Send zero command
    void send_zero_command() {
        geometry_msgs::msg::Twist stop;
        stop.linear.x = stop.linear.y = stop.angular.z = 0.0;
        cmd_pub_->publish(stop);
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double Vmax_, Wmax_, deadzone_;

    // Latest joystick state
    double latest_vx_ = 0.0;
    double latest_vy_ = 0.0;
    double latest_omega_ = 0.0;

    static KilinJoystickInterface* instance_;
};

KilinJoystickInterface* KilinJoystickInterface::instance_ = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinJoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
