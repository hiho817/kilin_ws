#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <cmath>
#include <thread>
#include <csignal>
#include <array>

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
        omega_axes_ = this->declare_parameter<int>("omega_axes", 2);
        enable_rest_mask_ = this->declare_parameter<bool>("enable_rest_mask", false);
        hip_torque_limit_ = this->declare_parameter<double>("hip_torque_limit", 10.0);

        // -----------------------------
        // ROS interfaces
        // -----------------------------
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&KilinJoystickInterface::joy_callback, this, _1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/kilin/cmd_vel", 10);

        brake_pub_ = this->create_publisher<std_msgs::msg::Bool>("/kilin/brake_request", 10);

        // Hip control mask output (Momentary)
        //   - axes[7]=1  -> bit0 (A)
        //   - axes[6]=-1 -> bit1 (B)
        //   - axes[7]=-1 -> bit2 (C)
        //   - axes[6]=1  -> bit3 (D)
        hip_mask_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/kilin/hip_control_mask", 10);

        // Hip control torque output
        //   - axes[4] -> positive torque (range 1 -> -1 maps to 0 -> 10)
        //   - axes[3] -> negative torque (range 1 -> -1 maps to 0 -> 10)
        hip_torque_pub_ = this->create_publisher<std_msgs::msg::Float64>("/kilin/hip_control_torque", 10);

        // Module rest mask output
        //   - buttons[0..3] -> modules A,B,C,D
        //   - bit0=A bit1=B bit2=C bit3=D
        if (enable_rest_mask_) {
            rest_mask_pub_ = this->create_publisher<std_msgs::msg::UInt8>("/kilin/module_rest_mask", 10);
            RCLCPP_WARN(this->get_logger(), "REST MASK enabled (debug feature).");
        }
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

        // Publish safe defaults on exit
        std_msgs::msg::Bool brake_msg;
        brake_msg.data = false;
        brake_pub_->publish(brake_msg);

        if (enable_rest_mask_ && rest_mask_pub_) {
            std_msgs::msg::UInt8 rm;
            rm.data = 0;
            rest_mask_pub_->publish(rm);   
        }

        if (hip_mask_pub_) {
            std_msgs::msg::UInt8 hm;
            hm.data = 0;
            hip_mask_pub_->publish(hm);
        }

        if (hip_torque_pub_) {
            std_msgs::msg::Float64 ht;
            ht.data = 0.0;
            hip_torque_pub_->publish(ht);
        }
    }

    static void signal_handler(int) {
        if (instance_) {
            instance_->send_zero_command();

            std_msgs::msg::Bool brake_msg;
            brake_msg.data = false;
            instance_->brake_pub_->publish(brake_msg);

            std_msgs::msg::UInt8 rm;
            rm.data = 0;
            instance_->rest_mask_pub_->publish(rm);

            std_msgs::msg::UInt8 hm;
            hm.data = 0;
            instance_->hip_mask_pub_->publish(hm);

            std_msgs::msg::Float64 ht;
            ht.data = 0.0;
            instance_->hip_torque_pub_->publish(ht);

            RCLCPP_WARN(instance_->get_logger(), "Sent zero command before shutdown");
            std::this_thread::sleep_for(std::chrono::milliseconds(300));
        }
        rclcpp::shutdown();
    }

private:
    // --------------------------------------
    // Print module rest status (only when toggled)
    // --------------------------------------
    void log_rest_status(uint8_t mask) {
        auto st = [&](uint8_t bit){ return (mask & bit) ? "REST" : "NORMAL"; };

        RCLCPP_INFO(this->get_logger(),
            "[REST MASK] A=%s  B=%s  C=%s  D=%s  (mask=0x%02X)",
            st(0x01), st(0x02), st(0x04), st(0x08), mask);
    }

    // --------------------------------------
    // Print hip control status (only when toggled)
    // --------------------------------------
    void log_hip_status(uint8_t mask) {
        auto st = [&](uint8_t bit){ return (mask & bit) ? "ENABLED" : "DISABLED"; };

        RCLCPP_INFO(this->get_logger(),
            "[HIP MASK] A=%s  B=%s  C=%s  D=%s  (mask=0x%02X)",
            st(0x01), st(0x02), st(0x04), st(0x08), mask);
    }

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

        // ---------------------------------------------------------
        // Module rest mask toggle (buttons[0..3])
        //   - press once -> toggle bit
        //   - buttons[0] -> A (bit0)
        //   - buttons[1] -> B (bit1)
        //   - buttons[2] -> C (bit2)
        //   - buttons[3] -> D (bit3)
        // ---------------------------------------------------------
        if (enable_rest_mask_) {
            bool toggled = false;

            for (int i = 0; i < 4; i++) {
                int cur = 0;
                if ((int)msg->buttons.size() > i) cur = msg->buttons[i];

                // Rising edge: 0 -> 1
                if (cur == 1 && prev_btn_[i] == 0) {
                    rest_mask_ ^= (uint8_t)(1u << i);   // toggle bit i
                    toggled = true;
                }

                prev_btn_[i] = cur;
            }

            if (toggled) {
                log_rest_status(rest_mask_);
            }
        }

        // --------------------------------------
        // Brake combo: buttons[4] + buttons[5]
        // --------------------------------------
        bool b4 = (msg->buttons.size() > 4) ? (msg->buttons[4] == 1) : false;
        bool b5 = (msg->buttons.size() > 5) ? (msg->buttons[5] == 1) : false;
        brake_active_ = (b4 && b5);

        if (brake_active_) {
            latest_vx_ = 0.0;
            latest_vy_ = 0.0;
            latest_omega_ = 0.0;
            return;
        }

        // Left stick
        if (msg->axes.size() > 1) latest_vx_ = apply_deadzone(msg->axes[1]);
        if (msg->axes.size() > 0) latest_vy_ = apply_deadzone(msg->axes[0]);

        // Right stick (ω)
        if ((int)msg->axes.size() > omega_axes_)
            latest_omega_ = apply_deadzone(msg->axes[omega_axes_]);
        else
            latest_omega_ = 0.0;

        // ---------------------------------------------------------
        // Hip control mask toggle (axes[6], axes[7])
        //   - axes[7]=1  -> A (bit0)
        //   - axes[6]=-1 -> B (bit1)
        //   - axes[7]=-1 -> C (bit2)
        //   - axes[6]=1  -> D (bit3)
        // ---------------------------------------------------------
        bool hip_toggled = false;
        if (msg->axes.size() > 7) {
            double cur7 = msg->axes[7];
            // Up (A) toggle on transition 0 -> 1
            if (cur7 == 1.0 && prev_ax7_ != 1.0) {
                hip_mask_ ^= 0x01;
                hip_toggled = true;
            }
            // Down (C) toggle on transition 0 -> -1
            if (cur7 == -1.0 && prev_ax7_ != -1.0) {
                hip_mask_ ^= 0x04;
                hip_toggled = true;
            }
            prev_ax7_ = cur7;
        }

        if (msg->axes.size() > 6) {
            double cur6 = msg->axes[6];
            // Left (B) toggle on transition 0 -> -1
            if (cur6 == -1.0 && prev_ax6_ != -1.0) {
                hip_mask_ ^= 0x02;
                hip_toggled = true;
            }
            // Right (D) toggle on transition 0 -> 1
            if (cur6 == 1.0 && prev_ax6_ != 1.0) {
                hip_mask_ ^= 0x08;
                hip_toggled = true;
            }
            prev_ax6_ = cur6;
        }

        if (hip_toggled) {
            log_hip_status(hip_mask_);
        }

        // ---------------------------------------------------------
        // Hip control torque (axes[3], axes[4])
        //   - Linear axes: 1 (unpressed) -> -1 (fully pressed)
        //   - Map each to 0 -> hip_torque_limit_
        // ---------------------------------------------------------
        double torque_pos = 0.0;
        if (msg->axes.size() > 4) torque_pos = (1.0 - msg->axes[4]) / 2.0 * hip_torque_limit_;

        double torque_neg = 0.0;
        if (msg->axes.size() > 3) torque_neg = (1.0 - msg->axes[3]) / 2.0 * hip_torque_limit_;

        latest_hip_torque_ = torque_pos - torque_neg;
    }

    // --------------------------------------
    // Timer: Publish at EXACT 100 Hz
    // --------------------------------------
    void timer_callback() {
        // cmd_vel @ 100 Hz
        geometry_msgs::msg::Twist twist;
        twist.linear.x  = latest_vx_ * Vmax_;
        twist.linear.y  = latest_vy_ * Vmax_;
        twist.angular.z = latest_omega_ * Wmax_;
        cmd_pub_->publish(twist);

        // brake_request @ 100 Hz
        std_msgs::msg::Bool brake_msg;
        brake_msg.data = brake_active_;
        brake_pub_->publish(brake_msg);

        if (enable_rest_mask_ && rest_mask_pub_) {
            // module_rest_mask @ 100 Hz
            std_msgs::msg::UInt8 rm;
            rm.data = rest_mask_;
            rest_mask_pub_->publish(rm);
        }

        if (hip_mask_pub_) {
            // hip_control_mask @ 100 Hz
            std_msgs::msg::UInt8 hm;
            hm.data = hip_mask_;
            hip_mask_pub_->publish(hm);
        }

        if (hip_torque_pub_) {
            // hip_control_torque @ 100 Hz
            std_msgs::msg::Float64 ht;
            ht.data = latest_hip_torque_;
            hip_torque_pub_->publish(ht);
        }
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
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brake_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr rest_mask_pub_;
    rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr hip_mask_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr hip_torque_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double Vmax_, Wmax_, deadzone_;
    int omega_axes_;
    double hip_torque_limit_;

    // Latest joystick state
    double latest_vx_ = 0.0;
    double latest_vy_ = 0.0;
    double latest_omega_ = 0.0;
    bool brake_active_ = false;
    bool enable_rest_mask_ = false;

    // Module rest mask (toggle)
    uint8_t rest_mask_ = 0;
    std::array<int, 4> prev_btn_{0, 0, 0, 0};

    // Hip control mask (toggle)
    uint8_t hip_mask_ = 0;
    double prev_ax6_ = 0.0;
    double prev_ax7_ = 0.0;

    // Hip control torque
    double latest_hip_torque_ = 0.0;

    static KilinJoystickInterface* instance_;
};

KilinJoystickInterface* KilinJoystickInterface::instance_ = nullptr;

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinJoystickInterface>());
    rclcpp::shutdown();
    return 0;
}
