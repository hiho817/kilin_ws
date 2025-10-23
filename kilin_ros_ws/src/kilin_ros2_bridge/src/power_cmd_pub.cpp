#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/power_cmd_stamped.hpp"
#include "Power.pb.h"

#include <sstream>
#include <string>
#include <iostream>
#include <chrono>

using namespace std::chrono_literals;

power_msg::ROBOTMODE parse_mode(char mode_char)
{
    switch (mode_char)
    {
        case 'R': return power_msg::REST_MODE;
        case 'M': return power_msg::MOTOR_MODE;
        case 'S': return power_msg::SET_ZERO;
        case 'H': return power_msg::HALL_CALIBRATE;
        case 'C': return power_msg::CONFIG_MODE;
        default:
            std::cout << "Invalid mode character.\n";
            return power_msg::REST_MODE;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("power_cmd_publisher");
    auto pub = node->create_publisher<kilin_msgs::msg::PowerCmdStamped>("power/command", 10);

    int seq = 0;
    bool digital = false, signal = false, power = false, clean = false, trigger = false, steering_cali = false;
    power_msg::ROBOTMODE robot_mode = power_msg::REST_MODE;

    rclcpp::Rate rate(1000);

    while (rclcpp::ok())
    {
        std::string line;
        std::cout << "Enter command (e.g. P D 1 or F M R): ";
        std::getline(std::cin, line);
        std::istringstream iss(line);
        std::string cmd, subcmd;
        iss >> cmd >> subcmd;

        if (!cmd.empty()) cmd[0] = toupper(cmd[0]);
        if (!subcmd.empty()) subcmd[0] = toupper(subcmd[0]);

        if (cmd == "P" && (subcmd == "D" || subcmd == "S" || subcmd == "P" || subcmd == "C" || subcmd == "T" || subcmd == "A"))
        {
            int val;
            iss >> val;
            if (subcmd == "D") digital = (val != 0);
            if (subcmd == "S") signal = (val != 0);
            if (subcmd == "P") power = (val != 0);
            if (subcmd == "C") clean = (val != 0);
            if (subcmd == "T") trigger = (val != 0);
            if (subcmd == "A") steering_cali = (val != 0);
        }
        else if (cmd == "F" && subcmd == "M")
        {
            char mode_char;
            if (!(iss >> mode_char))
            {
                std::cout << "Missing mode character. Usage: F M <R|M|S|H|C>\n";
                continue;
            }
            mode_char = toupper(mode_char);
            robot_mode = parse_mode(mode_char);
        }
        else if (cmd == "H")
        {
            std::cout << "Usage:\n"
                      << "  P D|S|P|C|T|A <0|1>  - Set digital/signal/power/clean/trigger/steering_cali state\n"
                      << "  F M <R|M|S|H|C>      - Set robot mode\n";
            continue;
        }
        else
        {
            std::cout << "Invalid command.\n";
            continue;
        }

        kilin_msgs::msg::PowerCmdStamped msg;

        // 🔹 Header 與 motor_cmd 寫法一致
        auto now = node->get_clock()->now();
        msg.header.seq = seq++;
        msg.header.time.sec = static_cast<int32_t>(now.seconds());
        msg.header.time.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);

        // 🔹 Payload
        msg.digital = digital;
        msg.signal = signal;
        msg.power = power;
        msg.clean = clean;
        msg.trigger = trigger;
        msg.steering_cali = steering_cali;
        msg.robot_mode = static_cast<int32_t>(robot_mode);

        pub->publish(msg);

        RCLCPP_INFO(node->get_logger(),
            "Published PowerCmd: D=%d S=%d P=%d C=%d T=%d A=%d Mode=%d",
            msg.digital, msg.signal, msg.power, msg.clean, msg.trigger, msg.steering_cali, msg.robot_mode);

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
