#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd.hpp"
#include <cmath>
#include <map>

struct ModulePos {
    double Xi; // [m]
    double Yi; // [m]
};

class KilinCmdConverter : public rclcpp::Node {
public:
    KilinCmdConverter() : Node("kilin_cmd_converter") {
        // -------------------------------
        // Parameters
        // -------------------------------
        L_base = declare_parameter<double>("L_base", 0.48);     // wheelbase
        W_base = declare_parameter<double>("W_base", 0.4925);   // track width
        R_w    = declare_parameter<double>("R_w", 0.0525);      // wheel radius
        vmax   = declare_parameter<double>("vmax", 1.0);        // max linear velocity
        wmax   = declare_parameter<double>("wmax", 2.0);        // max angular velocity

        // Module layout (A: FL, B: FR, C: RL, D: RR)
        modules["A"] = { +0.5 * L_base, +0.5 * W_base }; // Front Left
        modules["B"] = { +0.5 * L_base, -0.5 * W_base }; // Front Right
        modules["C"] = { -0.5 * L_base, +0.5 * W_base }; // Rear Left
        modules["D"] = { -0.5 * L_base, -0.5 * W_base }; // Rear Right

        // -------------------------------
        // ROS interfaces
        // -------------------------------
        sub_cmdvel = create_subscription<geometry_msgs::msg::Twist>(
            "/kilin/cmd_vel", 10,
            std::bind(&KilinCmdConverter::cmdVelCallback, this, std::placeholders::_1));

        // Publish to UI instead of bridge
        // Topic: /kilin/motor_cmd_raw
        pub_motor = create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        RCLCPP_INFO(get_logger(),
                    "KilinCmdConverter started (L=%.2f, W=%.2f, Rw=%.3f, publishing to /kilin/motor_cmd_raw)",
                    L_base, W_base, R_w);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = std::clamp(msg->linear.x, -vmax, vmax);
        double vy = std::clamp(msg->linear.y, -vmax, vmax);
        double wz = std::clamp(msg->angular.z, -wmax, wmax);

        kilin_msgs::msg::MotorCmdStamped motor_msg;

        static uint32_t seq = 0;
        motor_msg.header.seq = seq++;
        auto now = this->get_clock()->now();
        motor_msg.header.time.sec = static_cast<int32_t>(now.seconds());
        motor_msg.header.time.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
        motor_msg.header.frame_id = "cmd_converter";

        // motor mode constants
        const int REST_MODE     = 0;
        const int POSITION_MODE = 4;
        const int VELOCITY_MODE = 5;

        // Compute each module command
        for (auto &[key, pos] : modules) {
            double Vx = vx - wz * pos.Yi;
            double Vy = vy + wz * pos.Xi;

            double steering_angle = std::atan2(Vy, Vx);
            double wheel_speed = std::sqrt(Vx * Vx + Vy * Vy) / R_w;

            // Make hub bidirectional, steering stays within ±90°
            if (steering_angle > M_PI_2) {
                steering_angle -= M_PI;
                wheel_speed = -wheel_speed;
            } else if (steering_angle < -M_PI_2) {
                steering_angle += M_PI;
                wheel_speed = -wheel_speed;
            }

            if (steering_angle < 0) {
                steering_angle += 2 * M_PI; // convert to [0, 2π]
            }

            wheel_speed = wheel_speed / (2.0 * M_PI) * 60.0 * 10.0;

            // Hip
            kilin_msgs::msg::MotorCmd hip;
            hip.motor_mode = POSITION_MODE;
            hip.kp = 180.0;
            hip.ki = 0.0;
            hip.kd = 5.0;

            // Steering
            kilin_msgs::msg::MotorCmd steering;
            steering.position = steering_angle;
            steering.motor_mode = POSITION_MODE;

            // Hub
            kilin_msgs::msg::MotorCmd hub;
            hub.velocity = wheel_speed;
            hub.motor_mode = VELOCITY_MODE;

            // Assemble LegCmd
            kilin_msgs::msg::LegCmd leg;
            leg.hip = hip;
            leg.steering = steering;
            leg.hub = hub;

            if (key == "A") motor_msg.module_a = leg;
            else if (key == "B") motor_msg.module_b = leg;
            else if (key == "C") motor_msg.module_c = leg;
            else if (key == "D") motor_msg.module_d = leg;
        }

        pub_motor->publish(motor_msg);
    }

    // Parameters
    double L_base, W_base, R_w, vmax, wmax;
    std::map<std::string, ModulePos> modules;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_motor;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinCmdConverter>());
    rclcpp::shutdown();
    return 0;
}
