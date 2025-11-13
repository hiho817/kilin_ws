#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd.hpp"
#include <cmath>
#include <map>
#include <csignal>
#include <thread>

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
        L_base = declare_parameter<double>("L_base", 0.48);
        W_base = declare_parameter<double>("W_base", 0.4925);
        R_w    = declare_parameter<double>("R_w", 0.0525);
        vmax   = declare_parameter<double>("vmax", 1.0);
        wmax   = declare_parameter<double>("wmax", 2.0);

        // Steering rate limit (rad/s)
        steering_rate_limit =
            declare_parameter<double>("steering_rate_limit", 1); // default 1.0 rad/s

        // Module layout (A: FL, B: FR, C: RL, D: RR)
        modules["A"] = { +0.5 * L_base, +0.5 * W_base };
        modules["B"] = { +0.5 * L_base, -0.5 * W_base };
        modules["C"] = { -0.5 * L_base, +0.5 * W_base };
        modules["D"] = { -0.5 * L_base, -0.5 * W_base };

        // Initialize previous steering angles
        prev_steering_angle["A"] = 0.0;
        prev_steering_angle["B"] = 0.0;
        prev_steering_angle["C"] = 0.0;
        prev_steering_angle["D"] = 0.0;

        // -------------------------------
        // ROS interfaces
        // -------------------------------
        sub_cmdvel = create_subscription<geometry_msgs::msg::Twist>(
            "/kilin/cmd_vel", 10,
            std::bind(&KilinCmdConverter::cmdVelCallback, this, std::placeholders::_1));

        pub_motor = create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        instance_ = this;
        std::signal(SIGINT, signal_handler);

        RCLCPP_INFO(get_logger(),
            "KilinCmdConverter started (steering_rate_limit=%.2f rad/s)",
            steering_rate_limit);
    }

    ~KilinCmdConverter() {
        sendZeroCommand();
    }

    static void signal_handler(int) {
        if (instance_) {
            instance_->sendZeroCommand();
            RCLCPP_WARN(instance_->get_logger(),
                        "Converter stopped — zero command sent.");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        rclcpp::shutdown();
    }

private:

    // ============================================================
    // Main callback for receiving /kilin/cmd_vel
    // ============================================================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double vx = std::clamp(msg->linear.x, -vmax, vmax);
        double vy = std::clamp(msg->linear.y, -vmax, vmax);
        double wz = std::clamp(msg->angular.z, -wmax, wmax);

        kilin_msgs::msg::MotorCmdStamped motor_msg;
        fillHeader(motor_msg);

        const int POSITION_MODE = 4;
        const int VELOCITY_MODE = 5;

        // Each module
        for (auto &[key, pos] : modules) {
            double Vx = vx - wz * pos.Yi;
            double Vy = vy + wz * pos.Xi;

            // Original steering calculation
            double steering_angle = std::atan2(Vy, Vx);
            double wheel_speed = std::sqrt(Vx * Vx + Vy * Vy) / R_w;

            // ==================================================
            // Steering Rate Limiter
            // ==================================================
            double dt = 0.01;  // converter expected at 100Hz
            double max_delta = steering_rate_limit * dt;

            double prev = prev_steering_angle[key];
            double delta = steering_angle - prev;

            if (delta > max_delta) delta = max_delta;
            if (delta < -max_delta) delta = -max_delta;

            steering_angle = prev + delta;

            // Store for next cycle
            prev_steering_angle[key] = steering_angle;

            // Keep steering within ±90°
            if (steering_angle > M_PI_2) {
                steering_angle -= M_PI;
                wheel_speed = -wheel_speed;
            } else if (steering_angle < -M_PI_2) {
                steering_angle += M_PI;
                wheel_speed = -wheel_speed;
            }
            if (steering_angle < 0)
                steering_angle += 2 * M_PI;

            // Convert wheel speed to RPM (×10 for gearbox)
            wheel_speed = wheel_speed / (2.0 * M_PI) * 60.0 * 10.0;

            // ==================================================
            // Build command msg
            // ==================================================
            kilin_msgs::msg::MotorCmd hip;
            hip.motor_mode = POSITION_MODE;
            hip.kp = 180.0;
            hip.ki = 0.0;
            hip.kd = 5.0;

            kilin_msgs::msg::MotorCmd steering;
            steering.position = steering_angle;
            steering.motor_mode = POSITION_MODE;

            kilin_msgs::msg::MotorCmd hub;
            hub.velocity = wheel_speed;
            hub.motor_mode = VELOCITY_MODE;

            // Assemble module command
            kilin_msgs::msg::LegCmd leg;
            leg.hip = hip;
            leg.steering = steering;
            leg.hub = hub;

            if (key == "A")      motor_msg.module_a = leg;
            else if (key == "B") motor_msg.module_b = leg;
            else if (key == "C") motor_msg.module_c = leg;
            else if (key == "D") motor_msg.module_d = leg;
        }

        pub_motor->publish(motor_msg);
    }

    void fillHeader(kilin_msgs::msg::MotorCmdStamped &msg) {
        static uint32_t seq = 0;
        msg.header.seq = seq++;
        auto now = this->get_clock()->now();
        msg.header.time.sec = static_cast<int32_t>(now.seconds());
        msg.header.time.nanosec =
            static_cast<uint32_t>(now.nanoseconds() % 1000000000);
        msg.header.frame_id = "cmd_converter";
    }

    void sendZeroCommand() {
        kilin_msgs::msg::MotorCmdStamped stop_msg;
        fillHeader(stop_msg);

        kilin_msgs::msg::MotorCmd zero;
        zero.motor_mode = 5;
        zero.velocity = 0.0;

        kilin_msgs::msg::LegCmd zero_leg;
        zero_leg.hip = zero;
        zero_leg.steering = zero;
        zero_leg.hub = zero;

        stop_msg.module_a = zero_leg;
        stop_msg.module_b = zero_leg;
        stop_msg.module_c = zero_leg;
        stop_msg.module_d = zero_leg;

        pub_motor->publish(stop_msg);
    }

    // -----------------------------------------------------------
    // Parameters
    // -----------------------------------------------------------
    double L_base, W_base, R_w, vmax, wmax;
    double steering_rate_limit;

    std::map<std::string, ModulePos> modules;
    std::map<std::string, double> prev_steering_angle;

    // ROS
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_motor;

    static KilinCmdConverter *instance_;
};

KilinCmdConverter *KilinCmdConverter::instance_ = nullptr;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinCmdConverter>());
    rclcpp::shutdown();
    return 0;
}
