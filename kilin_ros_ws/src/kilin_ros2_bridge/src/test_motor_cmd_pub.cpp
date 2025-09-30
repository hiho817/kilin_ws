//This node is for testing purpose only. It publishes a MotorCmdStamped message with non-zero values every second.

#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"

using namespace std::chrono_literals;

class TestMotorCmdPublisher : public rclcpp::Node {
public:
    TestMotorCmdPublisher() : Node("test_motor_cmd_pub"), count_(0) {
        pub_ = this->create_publisher<kilin_msgs::msg::MotorCmdStamped>("motor/command", 10);
        timer_ = this->create_wall_timer(1000ms, [this]() { this->tick(); });
    }

private:
    void tick() {
        kilin_msgs::msg::MotorCmdStamped msg;

        // header
        auto now = this->get_clock()->now();
        msg.header.seq = count_;
        msg.header.time.sec = static_cast<int32_t>(now.seconds());
        msg.header.time.nanosec = now.nanoseconds() % 1000000000;

        // 準備一組非 0 測試值
        kilin_msgs::msg::LegCmd leg;

        // hip
        leg.hip.position = 0.10;
        leg.hip.kp       = 1.0;
        leg.hip.ki       = 0.01;
        leg.hip.kd       = 0.05;
        leg.hip.torque   = 0.2;

        // steering
        leg.steering.position = -0.20;
        leg.steering.kp       = 0.8;
        leg.steering.ki       = 0.02;
        leg.steering.kd       = 0.04;
        leg.steering.torque   = 0.0;

        // hub
        leg.hub.position = 3.14;
        leg.hub.kp       = 0.5;
        leg.hub.ki       = 0.00;
        leg.hub.kd       = 0.03;
        leg.hub.torque   = 1.0;

        // 四個模組先都塞同一組
        msg.module_a = leg;
        msg.module_b = leg;
        msg.module_c = leg;
        msg.module_d = leg;

        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Publishing MotorCmdStamped #%d", count_++);
    }

    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestMotorCmdPublisher>());
    rclcpp::shutdown();
    return 0;
}
