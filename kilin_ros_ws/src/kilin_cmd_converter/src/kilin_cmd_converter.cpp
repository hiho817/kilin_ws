#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd.hpp"

#include <cmath>
#include <map>
#include <csignal>
#include <thread>
#include <algorithm>
#include <atomic>
#include <mutex>

struct ModulePos {
    double Xi;
    double Yi;
};

class KilinCmdConverter : public rclcpp::Node {
public:
    KilinCmdConverter() : Node("kilin_cmd_converter") {
        // Parameters
        L_base = declare_parameter<double>("L_base", 0.48);
        W_base = declare_parameter<double>("W_base", 0.4925);
        R_w    = declare_parameter<double>("R_w", 0.0525);
        vmax   = declare_parameter<double>("vmax", 1.0);
        wmax   = declare_parameter<double>("wmax", 2.0);

        // Steering speed limit (rad/s)
        steering_rate_limit =
            declare_parameter<double>("steering_rate_limit", 2.0);

        // Debug switch: module rest mask
        enable_rest_mask_ =
            declare_parameter<bool>("enable_rest_mask", false);

        // ============================================================
        // Steering-wheel coordination (vy gating)
        //   - Use steering tracking error to scale ONLY global vy
        //   - Avoid "stop and wait" feeling by:
        //       (1) g_min floor
        //       (2) g slew-rate limiting (down slow, up fast)
        //       (3) aggregate by mean (not min)
        // ============================================================
        e_full  = declare_parameter<double>("e_full", 0.10);   // rad ~ 5.7 deg
        e_stop  = declare_parameter<double>("e_stop", 0.52);   // rad ~ 30 deg
        g_min   = declare_parameter<double>("g_min", 0.30);    // keep some lateral authority
        dg_down = declare_parameter<double>("dg_down", 1.0);   // 1/s (g decreases)
        dg_up   = declare_parameter<double>("dg_up",   2.5);   // 1/s (g increases)

        prev_g = 1.0;

        // Module coordinates (A: FL, B: FR, C: RL, D: RR)
        modules["A"] = { +0.5 * L_base, +0.5 * W_base };
        modules["B"] = { +0.5 * L_base, -0.5 * W_base };
        modules["C"] = { -0.5 * L_base, +0.5 * W_base };
        modules["D"] = { -0.5 * L_base, -0.5 * W_base };

        // Initialize steering memory (continuous, start from 0 after set-zero)
        prev_steering_angle["A"] = 0.0;
        prev_steering_angle["B"] = 0.0;
        prev_steering_angle["C"] = 0.0;
        prev_steering_angle["D"] = 0.0;

        hip_positions_["A"] = 0.0;
        hip_positions_["B"] = 0.0;
        hip_positions_["C"] = 0.0;
        hip_positions_["D"] = 0.0;

        // ROS interfaces
        sub_cmdvel = create_subscription<geometry_msgs::msg::Twist>(
            "/kilin/cmd_vel", 10,
            std::bind(&KilinCmdConverter::cmdVelCallback, this, std::placeholders::_1));

        sub_hip_position = create_subscription<std_msgs::msg::Float64MultiArray>(
            "/kilin/hip_cmd_position", 10,
            std::bind(&KilinCmdConverter::hipPositionCallback, this, std::placeholders::_1));

        // Brake request (from kilin_joystick)
        sub_brake = create_subscription<std_msgs::msg::Bool>(
            "/kilin/brake_request", 10,
            std::bind(&KilinCmdConverter::brakeCallback, this, std::placeholders::_1));

        // Module rest mask (from kilin_joystick) - only in debug mode
        if (enable_rest_mask_) {
            sub_rest_mask = create_subscription<std_msgs::msg::UInt8>(
                "/kilin/module_rest_mask", 10,
                std::bind(&KilinCmdConverter::restMaskCallback, this, std::placeholders::_1));

            RCLCPP_WARN(get_logger(), "REST MASK enabled (debug feature).");
        }

        // Hip control subscriptions
        sub_hip_mask = create_subscription<std_msgs::msg::UInt8>(
            "/kilin/hip_control_mask", 10,
            std::bind(&KilinCmdConverter::hipMaskCallback, this, std::placeholders::_1));
        
        sub_hip_torque = create_subscription<std_msgs::msg::Float64>(
            "/kilin/hip_control_torque", 10,
            std::bind(&KilinCmdConverter::hipTorqueCallback, this, std::placeholders::_1));

        pub_motor = create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        RCLCPP_INFO(get_logger(),
            "KilinCmdConverter started (rate_limit = %.2f rad/s)",
            steering_rate_limit);

        RCLCPP_INFO(get_logger(),
            "Brake modes fixed: hip_rest_mode=%d, hub_brake_mode=%d",
            HIP_REST_MODE, HUB_BRAKE_MODE);

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
    // Motor modes (FPGA definitions)
    // ============================================================
    static constexpr int HIP_REST_MODE  = 0;
    static constexpr int HUB_BRAKE_MODE = 7;
    static constexpr int POSITION_MODE  = 4;
    static constexpr int VELOCITY_MODE  = 5;
    static constexpr int TORQUE_MODE    = 6;

    // ============================================================
    // Steering safety bound (prevent cable twisting)
    // ============================================================
    static constexpr double STEER_BOUND = 2.0 * M_PI;   // ±2π

    // ============================================================
    // Hip control callbacks
    // ============================================================
    void hipMaskCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
        hip_mask.store(msg->data, std::memory_order_relaxed);
    }

    void hipTorqueCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        hip_torque.store(msg->data, std::memory_order_relaxed);
    }

    // Four position targets in the shared module order A/B/C/D = FL/FR/RL/RR.
    void hipPositionCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(get_logger(), "Hip position command has %zu values; expected 4", msg->data.size());
            return;
        }
        if (!std::all_of(msg->data.begin(), msg->data.end(),
                         [](double value) { return std::isfinite(value); })) {
            RCLCPP_WARN(get_logger(), "Hip position command contains a non-finite value");
            return;
        }
        std::lock_guard<std::mutex> lock(hip_position_mutex_);
        hip_positions_["A"] = msg->data[0];
        hip_positions_["B"] = msg->data[1];
        hip_positions_["C"] = msg->data[2];
        hip_positions_["D"] = msg->data[3];
        hip_position_command_received_.store(true, std::memory_order_release);
    }

    // ============================================================
    // Brake request callback
    // ============================================================
    void brakeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        brake_active.store(msg->data, std::memory_order_relaxed);
    }

    // ============================================================
    // Rest mask callback (debug only)
    //   - bit0=A bit1=B bit2=C bit3=D
    // ============================================================
    void restMaskCallback(const std_msgs::msg::UInt8::SharedPtr msg) {
        rest_mask.store(msg->data, std::memory_order_relaxed);
    }

    // ============================================================
    // Publish brake motor command
    // ============================================================
    void publishBrakeCommand() {
        kilin_msgs::msg::MotorCmdStamped out_msg;
        fillHeader(out_msg);

        kilin_msgs::msg::MotorCmd hip;
        hip.motor_mode = HIP_REST_MODE;

        kilin_msgs::msg::MotorCmd steering;
        steering.motor_mode = POSITION_MODE;
        steering.position = 0.0;

        kilin_msgs::msg::MotorCmd hub;
        hub.motor_mode = HUB_BRAKE_MODE;

        kilin_msgs::msg::LegCmd leg;
        leg.hip = hip;
        leg.steering = steering;
        leg.hub = hub;

        out_msg.module_a = leg;
        out_msg.module_b = leg;
        out_msg.module_c = leg;
        out_msg.module_d = leg;

        // Reset steering memory to 0 (continuous)
        prev_steering_angle["A"] = 0.0;
        prev_steering_angle["B"] = 0.0;
        prev_steering_angle["C"] = 0.0;
        prev_steering_angle["D"] = 0.0;

        pub_motor->publish(out_msg);
    }

    // ============================================================
    // Apply module rest override (debug only)
    // ============================================================
    void applyRestOverride(const std::string &key, kilin_msgs::msg::LegCmd &leg) {
        if (!enable_rest_mask_) return;

        uint8_t mask = rest_mask.load(std::memory_order_relaxed);

        bool rest_this = false;
        if (key == "A") rest_this = (mask & 0x01);
        if (key == "B") rest_this = (mask & 0x02);
        if (key == "C") rest_this = (mask & 0x04);
        if (key == "D") rest_this = (mask & 0x08);

        if (!rest_this) return;

        leg.hip.motor_mode = HIP_REST_MODE;
        leg.steering.motor_mode = HIP_REST_MODE;
        leg.hub.motor_mode = HIP_REST_MODE;

        // Keep steering memory consistent if module is forced to rest
        prev_steering_angle[key] = 0.0;
    }

    // ============================================================
    // /cmd_vel → wheel + steering conversion
    // ============================================================
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (brake_active.load(std::memory_order_relaxed)) {
            publishBrakeCommand();
            return;
        }

        // Clamp input
        double vx = std::clamp(msg->linear.x,  -vmax, vmax);
        double vy = std::clamp(msg->linear.y,  -vmax, vmax);
        double wz = std::clamp(msg->angular.z, -wmax, wmax);

        kilin_msgs::msg::MotorCmdStamped out_msg;
        fillHeader(out_msg);

        auto wrap_to_pi = [](double a){
            while (a >  M_PI) a -= 2.0*M_PI;
            while (a <= -M_PI) a += 2.0*M_PI;
            return a;
        };

        // Map any angle to [0, 2π)
        auto mod_2pi = [](double a){
            a = std::fmod(a, 2.0*M_PI);
            if (a < 0) a += 2.0*M_PI;
            return a;
        };

        // Unwrap a wrapped target ([-π,π]) to be nearest to a continuous current angle
        auto unwrap_to_near = [&](double target_wrapped, double current_cont){
            double curr_wrapped = wrap_to_pi(std::fmod(current_cont, 2.0*M_PI));
            double diff = wrap_to_pi(target_wrapped - curr_wrapped);
            return current_cont + diff;
        };

        const double dt = 0.01;

        // Pass 1: compute g
        double max_delta = steering_rate_limit * dt;

        double g_sum = 0.0;
        int n_mod = 0;

        // Iterate modules A,B,C,D (evaluate error only)
        for (auto &[key, pos] : modules) {

            // Local velocities (use ORIGINAL vy)
            double Vx = vx - wz * pos.Yi;
            double Vy = vy + wz * pos.Xi;

            // Target steering direction in [-π, π]
            double target_raw = std::atan2(Vy, Vx);

            // Previous steering command (continuous, DO NOT wrap)
            double cur = prev_steering_angle[key];

            // =====================================================
            // Minimal steering angle logic (for error evaluation):
            // Always move within ±90°, wheel reverses if needed.
            // =====================================================
            double cur_wrapped = wrap_to_pi(std::fmod(cur, 2.0*M_PI));
            double diff_wrapped = wrap_to_pi(target_raw - cur_wrapped);

            // If shortest rotation is > 90°, flip direction
            if (std::fabs(diff_wrapped) > M_PI_2) {
                target_raw = wrap_to_pi(target_raw + (diff_wrapped > 0 ? -M_PI : M_PI));
                // wheel reverse not needed here (error-only pass)
            }

            // =====================================================
            // Unwrap to continuous target (near current)
            // + Clamp to ±2π (prevent cable twisting)
            // =====================================================
            double target = unwrap_to_near(target_raw, cur);
            target = std::clamp(target, -STEER_BOUND, +STEER_BOUND);

            // =====================================================
            // Rate-limited steering command (prediction)
            // =====================================================
            double new_cont = cur + std::clamp(target - cur, -max_delta, +max_delta);
            new_cont = std::clamp(new_cont, -STEER_BOUND, +STEER_BOUND);

            // Remaining tracking error after rate limit (continuous)
            double err = std::fabs(target - new_cont);

            // Map err -> gi in [0,1]
            double gi = 0.0;
            if (err <= e_full) gi = 1.0;
            else if (err >= e_stop) gi = 0.0;
            else gi = (e_stop - err) / (e_stop - e_full);

            g_sum += gi;
            n_mod++;
        }

        // Desired g (aggregate by mean)
        double g_des = (n_mod > 0) ? (g_sum / n_mod) : 1.0;
        g_des = std::clamp(g_des, 0.0, 1.0);

        // Keep some lateral authority (avoid full stop feeling)
        g_des = std::max(g_des, g_min);

        // Slew limit g (down slower, up faster)
        double dg = g_des - prev_g;
        double dg_lim = 0.0;
        if (dg >= 0.0) dg_lim = std::min(dg, dg_up * dt);
        else           dg_lim = std::max(dg, -dg_down * dt);

        double g = std::clamp(prev_g + dg_lim, 0.0, 1.0);
        prev_g = g;

        // ============================================================
        // scale ONLY global vy (keep vx unchanged)
        //   - This prevents "stop and wait steering" behavior
        //   - Lateral motion gradually ramps in as steering aligns
        // ============================================================
        double vy_eff = g * vy;

        // Pass 2: generate final commands
        for (auto &[key, pos] : modules) {

            // Local velocities
            double Vx = vx - wz * pos.Yi;
            double Vy = vy_eff + wz * pos.Xi;

            // Wheel speed (m/s → RPM → gearbox ×10)
            double wheel_speed = std::sqrt(Vx*Vx + Vy*Vy) / R_w;

            // Target steering direction in [-π, π]
            double raw_angle = std::atan2(Vy, Vx);

            // Previous steering command (continuous, DO NOT wrap)
            double cur = prev_steering_angle[key];

            // =====================================================
            // Minimal steering angle logic:
            // Always move within ±90°, wheel reverses if needed.
            // =====================================================
            double cur_wrapped = wrap_to_pi(std::fmod(cur, 2.0*M_PI));
            double diff_wrapped = wrap_to_pi(raw_angle - cur_wrapped);

            // If shortest rotation is > 90°, flip direction
            if (std::fabs(diff_wrapped) > M_PI_2) {
                raw_angle = wrap_to_pi(raw_angle + (diff_wrapped > 0 ? -M_PI : M_PI));
                wheel_speed = -wheel_speed;
            }

            // =====================================================
            // Unwrap to continuous target (near current)
            // + Clamp to ±2π (prevent cable twisting)
            // =====================================================
            double target = unwrap_to_near(raw_angle, cur);
            target = std::clamp(target, -STEER_BOUND, +STEER_BOUND);

            // =====================================================
            // Steering rate-limit (dt fixed = 0.01)
            // =====================================================
            double new_cont = cur + std::clamp(target - cur, -max_delta, +max_delta);
            new_cont = std::clamp(new_cont, -STEER_BOUND, +STEER_BOUND);

            // Update memory (continuous)
            prev_steering_angle[key] = new_cont;

            // =====================================================
            // Convert final angle to FPGA format (0 ~ 2π)
            //   - Use mod to avoid boundary jump issue
            // =====================================================
            double servo_angle = mod_2pi(new_cont);

            // Convert wheel speed to RPM ×10
            wheel_speed = wheel_speed / (2.0*M_PI) * 60.0 * 10.0;

            // -----------------------------------------------------
            // Build ROS motor command for this module
            // -----------------------------------------------------
            kilin_msgs::msg::MotorCmd hip;
            
            // Default hip behavior: REST (as requested by user)
            uint8_t h_mask = hip_mask.load(std::memory_order_relaxed);
            bool hip_enabled = false;
            if (key == "A") hip_enabled = (h_mask & 0x01);
            if (key == "B") hip_enabled = (h_mask & 0x02);
            if (key == "C") hip_enabled = (h_mask & 0x04);
            if (key == "D") hip_enabled = (h_mask & 0x08);

            if (hip_enabled) {
                hip.motor_mode = TORQUE_MODE;
                hip.torque = (float)hip_torque.load(std::memory_order_relaxed);
            } else if (hip_position_command_received_.load(std::memory_order_acquire)) {
                std::lock_guard<std::mutex> lock(hip_position_mutex_);
                hip.motor_mode = POSITION_MODE;
                hip.position = hip_positions_[key];
                hip.kp = 350.0;
                hip.ki = 0.0;
                hip.kd = 5.0;
            } else {
                hip.motor_mode = HIP_REST_MODE;
            }

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

            // Debug feature: per-module rest override
            applyRestOverride(key, leg);

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
        zero.motor_mode = VELOCITY_MODE;
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

    // debug switch
    bool enable_rest_mask_ = false;

    // brake
    std::atomic<bool> brake_active{false};

    // module rest mask (debug)
    std::atomic<uint8_t> rest_mask{0};

    // hip control
    std::atomic<uint8_t> hip_mask{0};
    std::atomic<double> hip_torque{0.0};

    // Position commands are intentionally inactive until their first valid
    // message, preserving the upstream default of hips at rest on startup.
    std::atomic<bool> hip_position_command_received_{false};
    std::map<std::string, double> hip_positions_;
    std::mutex hip_position_mutex_;

    // vy gating
    double e_full, e_stop, g_min;
    double dg_down, dg_up;
    double prev_g;

    std::map<std::string, ModulePos> modules;

    // Steering memory:
    //   - Store as continuous angle and clamp to ±2π
    //   - Servo output is always mapped to [0, 2π)
    std::map<std::string, double> prev_steering_angle;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_brake;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_rest_mask;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_hip_mask;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_hip_torque;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_hip_position;
    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_motor;

    static KilinCmdConverter *instance_;
};

KilinCmdConverter* KilinCmdConverter::instance_ = nullptr;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinCmdConverter>());
    rclcpp::shutdown();
    return 0;
}
