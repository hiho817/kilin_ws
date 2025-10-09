#include <rclcpp/rclcpp.hpp>
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"

#include <sstream>
#include <string>
#include <iostream>
#include <chrono>
#include <cctype>

using namespace std::chrono_literals;
using kilin_msgs::msg::MotorCmdStamped;
using kilin_msgs::msg::LegCmd;
using kilin_msgs::msg::MotorCmd;

LegCmd leg_a, leg_b, leg_c, leg_d;

// ----------- 工具函式 -----------
MotorCmd* getMotor(LegCmd& leg, const std::string& motor_name)
{
    std::string name = motor_name;
    for (auto &c : name) c = toupper(c);
    if (name == "HIP") return &leg.hip;
    if (name == "STREEING" || name == "STEERING") return &leg.steering;
    if (name == "HUB") return &leg.hub;
    return nullptr;
}

LegCmd* getModule(char module_char)
{
    switch (toupper(module_char)) {
        case 'A': return &leg_a;
        case 'B': return &leg_b;
        case 'C': return &leg_c;
        case 'D': return &leg_d;
        default: return nullptr;
    }
}

void printHelp()
{
    std::cout << "Usage:\n";
    std::cout << "  SET <Module A-D> <Motor HIP|STREEING|HUB> <Pos> <Kp> <Ki> <Kd> <Torque> <Velocity>\n";
    std::cout << "  Example: SET A HIP 1.0 20.0 0.1 0.2 3.0 0.5\n";
    std::cout << "  Type SEND to publish current config\n";
    std::cout << "  Type SHOW to view current config\n";
    std::cout << "  Type EXIT to quit\n";
}

void printMotorCmd(const std::string& label, const MotorCmd& m)
{
    std::cout << "  " << label
              << " => pos: " << m.position
              << ", kp: " << m.kp
              << ", ki: " << m.ki
              << ", kd: " << m.kd
              << ", torque: " << m.torque
              << ", velocity: " << m.velocity << "\n";
}

void showCurrentConfig()
{
    std::cout << "\n--- Current Command Configuration ---\n";
    std::cout << "[Module A]\n";
    printMotorCmd("HIP", leg_a.hip);
    printMotorCmd("STREEING", leg_a.steering);
    printMotorCmd("HUB", leg_a.hub);

    std::cout << "[Module B]\n";
    printMotorCmd("HIP", leg_b.hip);
    printMotorCmd("STREEING", leg_b.steering);
    printMotorCmd("HUB", leg_b.hub);

    std::cout << "[Module C]\n";
    printMotorCmd("HIP", leg_c.hip);
    printMotorCmd("STREEING", leg_c.steering);
    printMotorCmd("HUB", leg_c.hub);

    std::cout << "[Module D]\n";
    printMotorCmd("HIP", leg_d.hip);
    printMotorCmd("STREEING", leg_d.steering);
    printMotorCmd("HUB", leg_d.hub);
    std::cout << "--------------------------------------\n";
}

// ----------- 主程式 -----------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motor_cmd_pub_ui");
    auto pub = node->create_publisher<MotorCmdStamped>("/motor/command", 10);

    int seq = 0;
    printHelp();

    while (rclcpp::ok()) {
        std::cout << "\n>> ";
        std::string line;
        std::getline(std::cin, line);
        if (line.empty()) continue;

        std::istringstream iss(line);
        std::string cmd;
        iss >> cmd;
        for (auto& c : cmd) c = toupper(c);

        if (cmd == "EXIT") {
            break;
        } else if (cmd == "HELP") {
            printHelp();
        } else if (cmd == "SHOW") {
            showCurrentConfig();
        } else if (cmd == "SET") {
            char module_char;
            std::string motor_name;
            double pos, kp, ki, kd, torque, velocity;

            if (!(iss >> module_char >> motor_name >> pos >> kp >> ki >> kd >> torque >> velocity)) {
                std::cout << "Invalid format. Use: SET A HIP 1.0 20.0 0.1 0.2 3.0 0.5\n";
                continue;
            }

            auto* leg = getModule(module_char);
            if (!leg) {
                std::cout << "Invalid module. Use A, B, C, or D.\n";
                continue;
            }

            auto* motor = getMotor(*leg, motor_name);
            if (!motor) {
                std::cout << "Invalid motor name. Use HIP, STREEING, or HUB.\n";
                continue;
            }

            motor->position = pos;
            motor->kp = kp;
            motor->ki = ki;
            motor->kd = kd;
            motor->torque = torque;
            motor->velocity = velocity;

            std::cout << "Updated " << (char)toupper(module_char) << " " << motor_name
                      << " to pos=" << pos << ", kp=" << kp << ", ki=" << ki
                      << ", kd=" << kd << ", torque=" << torque << ", velocity=" << velocity << "\n";
        } else if (cmd == "SEND") {
            MotorCmdStamped msg;
            msg.header.seq = seq++;
            auto now = node->now();
            msg.header.time.sec = static_cast<int32_t>(now.seconds());
            msg.header.time.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1000000000);
            msg.header.frame_id = "motor_ui";

            msg.module_a = leg_a;
            msg.module_b = leg_b;
            msg.module_c = leg_c;
            msg.module_d = leg_d;

            pub->publish(msg);
            std::cout << "Published MotorCmdStamped.\n";
        } else {
            std::cout << "Unknown command. Type HELP for usage.\n";
        }
    }

    std::cout << "Exiting Motor Command Publisher.\n";
    rclcpp::shutdown();
    return 0;
}
