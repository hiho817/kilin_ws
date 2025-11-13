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
    double Xi;
    double Yi;
};

class KilinCmdConverter : public rclcpp::Node {
public:
    KilinCmdConverter() : Node("kilin_cmd_converter") 
    {
        // Parameters
        L_base = declare_parameter<double>("L_base", 0.48);
        W_base = declare_parameter<double>("W_base", 0.4925);
        R_w    = declare_parameter<double>("R_w", 0.0525);
        vmax   = declare_parameter<double>("vmax", 1.0);
        wmax   = declare_parameter<double>("wmax", 2.0);

        // Steering speed limit (rad/s)
        steering_rate_limit =
            declare_parameter<double>("steering_rate_limit", 2.0);

        // Module coordinates (A: FL, B: FR, C: RL, D: RR)
        modules["A"] = { +0.5 * L_base, +0.5 * W_base };
        modules["B"] = { +0.5 * L_base, -0.5 * W_base };
        modules["C"] = { -0.5 * L_base, +0.5 * W_base };
        modules["D"] = { -0.5 * L_base, -0.5 * W_base };

        // Initialize steering memory
        prev_steering_angle["A"] = 0.0;
        prev_steering_angle["B"] = 0.0;
        prev_steering_angle["C"] = 0.0;
        prev_steering_angle["D"] = 0.0;

        // ROS interfaces
        sub_cmdvel = create_subscription<geometry_msgs::msg::Twist>(
            "/kilin/cmd_vel", 10,
            std::bind(&KilinCmdConverter::cmdVelCallback, this, std::placeholders::_1));

        pub_motor = create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        RCLCPP_INFO(get_logger(), 
            "KilinCmdConverter started (rate_limit = %.2f rad/s)", 
            steering_rate_limit);

        instance_ = this;
        std::signal(SIGINT, signal_handler);
    }

    ~KilinCmdConverter() {
        sendZeroCommand();
    }

    // Handle Ctrl+C clean shutdown
    static void signal_handler(int) {
        if (instance_) {
            instance_->sendZeroCommand();
            RCLCPP_WARN(instance_->get_logger(),
                        "Converter stopped — zero command sent.");
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        rclcpp::shutdown();
    }

private:

    // ============================================================
    // /cmd_vel → wheel + steering conversion
    // ============================================================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) 
    {
        // Clamp input
        double vx = std::clamp(msg->linear.x,  -vmax, vmax);
        double vy = std::clamp(msg->linear.y,  -vmax, vmax);
        double wz = std::clamp(msg->angular.z, -wmax, wmax);

        kilin_msgs::msg::MotorCmdStamped out_msg;
        fillHeader(out_msg);

        const int POSITION_MODE = 4;
        const int VELOCITY_MODE = 5;

        // Utility for angle wrapping
        auto wrap_to_pi = [](double a){
            while (a >  M_PI) a -= 2.0*M_PI;
            while (a < -M_PI) a += 2.0*M_PI;
            return a;
        };

        // Iterate modules A,B,C,D
        for (auto &[key, pos] : modules) {

            // Local velocities
            double Vx = vx - wz * pos.Yi;
            double Vy = vy + wz * pos.Xi;

            // Wheel speed (m/s → RPM → gearbox ×10)
            double wheel_speed = std::sqrt(Vx*Vx + Vy*Vy) / R_w;

            // Target steering direction in [-π, π]
            double raw_angle = std::atan2(Vy, Vx);

            // Previous steering angle
            double prev = wrap_to_pi(prev_steering_angle[key]);

            // =====================================================
            // Minimal steering angle logic:
            // Always move within ±90°, wheel reverses if needed.
            // =====================================================
            double diff = wrap_to_pi(raw_angle - prev);

            // If shortest rotation is > 90°, flip direction
            if (std::fabs(diff) > M_PI_2) {
                raw_angle = wrap_to_pi(raw_angle + (diff > 0 ? -M_PI : M_PI));
                wheel_speed = -wheel_speed;   // reverse wheel direction
            }

            // =====================================================
            // Steering rate-limit (dt fixed = 0.01)
            // =====================================================
            double dt = 0.01;
            double max_delta = steering_rate_limit * dt;

            double new_angle = prev + std::clamp(
                wrap_to_pi(raw_angle - prev),
                -max_delta, max_delta
            );

            new_angle = wrap_to_pi(new_angle);
            prev_steering_angle[key] = new_angle;

            // =====================================================
            // Convert final angle to FPGA format (0 ~ 2π)
            // =====================================================
            double servo_angle = new_angle;
            if (servo_angle < 0)
                servo_angle += 2.0 * M_PI;

            // Convert wheel speed to RPM ×10
            wheel_speed = wheel_speed / (2.0*M_PI) * 60.0 * 10.0;

            // -----------------------------------------------------
            // Build ROS motor command for this module
            // -----------------------------------------------------
            kilin_msgs::msg::MotorCmd hip;
            hip.motor_mode = POSITION_MODE;
            hip.kp = 180.0;
            hip.ki = 0.0;
            hip.kd = 5.0;

            kilin_msgs::msg::MotorCmd steering;
            steering.motor_mode = POSITION_MODE;
            steering.position = servo_angle;

            kilin_msgs::msg::MotorCmd hub;
            hub.motor_mode = VELOCITY_MODE;
            hub.velocity = wheel_speed;

            kilin_msgs::msg::LegCmd leg;
            leg.hip = hip;
            leg.steering = steering;
            leg.hub = hub;

            if (key == "A")      out_msg.module_a = leg;
            else if (key == "B") out_msg.module_b = leg;
            else if (key == "C") out_msg.module_c = leg;
            else if (key == "D") out_msg.module_d = leg;
        }

        pub_motor->publish(out_msg);
    }

    // Build message header
    void fillHeader(kilin_msgs::msg::MotorCmdStamped &msg) {
        static uint32_t seq = 0;
        msg.header.seq = seq++;
        auto now = this->get_clock()->now();
        msg.header.time.sec = (int32_t) now.seconds();
        msg.header.time.nanosec = (uint32_t)(now.nanoseconds() % 1000000000);
        msg.header.frame_id = "cmd_converter";
    }

    // Send all-zero velocity to stop robot safely
    void sendZeroCommand() {
        kilin_msgs::msg::MotorCmdStamped msg;
        fillHeader(msg);

        kilin_msgs::msg::MotorCmd zero;
        zero.motor_mode = 5;   // velocity mode
        zero.velocity = 0.0;

        kilin_msgs::msg::LegCmd leg;
        leg.hip = zero;
        leg.steering = zero;
        leg.hub = zero;

        msg.module_a = leg;
        msg.module_b = leg;
        msg.module_c = leg;
        msg.module_d = leg;

        pub_motor->publish(msg);
    }

    // ============================================================
    // Internal fields
    // ============================================================
    double L_base, W_base, R_w;
    double vmax, wmax;
    double steering_rate_limit;

    std::map<std::string, ModulePos> modules;
    std::map<std::string, double> prev_steering_angle;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_motor;

    static KilinCmdConverter *instance_;
};

KilinCmdConverter* KilinCmdConverter::instance_ = nullptr;


// ============================================================
// Main
// ============================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinCmdConverter>());
    rclcpp::shutdown();
    return 0;
}
