#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct GaitPoint {
    double time;

    double a_hip_pos, a_hub_vel, a_hub_mode;
    double b_hip_pos, b_hub_vel, b_hub_mode;
    double c_hip_pos, c_hub_vel, c_hub_mode;
    double d_hip_pos, d_hub_vel, d_hub_mode;
};

class KilinGaitPlayer : public rclcpp::Node {
public:
    KilinGaitPlayer() : Node("kilin_csv_control")
    {
        this->declare_parameter<std::string>("csv_path", "gait.csv");
        this->declare_parameter<double>("rate_hz", 200.0);
        this->declare_parameter<double>("delay_start_sec", 3.0);

        csv_path = this->get_parameter("csv_path").as_string();
        rate_hz = this->get_parameter("rate_hz").as_double();
        delay_start_sec = this->get_parameter("delay_start_sec").as_double();

        if (!loadCSV(csv_path)) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV file: %s", csv_path.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_motor = this->create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        start_walltime = this->get_clock()->now();

        RCLCPP_INFO(get_logger(), "CSV playback starting in %.1f seconds...", delay_start_sec);

        timer = this->create_wall_timer(
            std::chrono::microseconds((int)(1e6 / rate_hz)),
            std::bind(&KilinGaitPlayer::update, this)
        );
    }

private:
    // ============================================================
    // CSV Loader
    // ============================================================
    bool loadCSV(const std::string &path)
    {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        getline(file, line); // header

        while (getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> nums;

            while (getline(ss, cell, ',')) {
                nums.push_back(std::stod(cell));
            }

            if (nums.size() != 13) {
                RCLCPP_ERROR(get_logger(),
                    "CSV row has %zu columns; expected 13!", nums.size());
                return false;
            }

            int k = 0;
            GaitPoint p;

            p.time = nums[k++];

            p.a_hip_pos = nums[k++];
            p.a_hub_vel = nums[k++];
            p.a_hub_mode = nums[k++];

            p.b_hip_pos = nums[k++];
            p.b_hub_vel = nums[k++];
            p.b_hub_mode = nums[k++];

            p.c_hip_pos = nums[k++];
            p.c_hub_vel = nums[k++];
            p.c_hub_mode = nums[k++];

            p.d_hip_pos = nums[k++];
            p.d_hub_vel = nums[k++];
            p.d_hub_mode = nums[k++];

            gait.push_back(p);
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu gait points from CSV.", gait.size());
        return true;
    }

    // ============================================================
    // Helpers: interpolation and ZOH
    // ============================================================
    double lerp(double t, double t0, double t1, double v0, double v1) {
        if (t1 == t0) return v0;
        double u = (t - t0) / (t1 - t0);
        return v0 + u * (v1 - v0);
    }

    double zoh(double v0) {
        return v0;
    }

    // ============================================================
    // Main Control Loop
    // ============================================================
    void update()
    {
        rclcpp::Time now = this->get_clock()->now();
        double wall_elapsed = (now - start_walltime).seconds();

        // --------------------------
        // Delay start
        // --------------------------
        if (wall_elapsed < delay_start_sec) {
            return;
        }

        if (!started) {
            RCLCPP_INFO(get_logger(), "CSV playback started.");
            playback_start_time = now;
            started = true;
        }

        double t = (now - playback_start_time).seconds();

        // --------------------------
        // Check if finished
        // --------------------------
        if (t > gait.back().time) {
            RCLCPP_INFO(get_logger(), "CSV playback finished. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // --------------------------
        // Find segment (p0, p1)
        // --------------------------
        GaitPoint p0 = gait.front();
        GaitPoint p1 = gait.front();

        for (size_t i = 0; i < gait.size() - 1; i++) {
            if (t >= gait[i].time && t <= gait[i+1].time) {
                p0 = gait[i];
                p1 = gait[i+1];
                break;
            }
        }

        // Build message
        kilin_msgs::msg::MotorCmdStamped msg;
        fillHeader(msg);

        msg.module_a = buildLegCmd(t, p0, p1,
                                   p0.a_hip_pos, p1.a_hip_pos,
                                   p0.a_hub_vel, p1.a_hub_vel,
                                   p0.a_hub_mode);

        msg.module_b = buildLegCmd(t, p0, p1,
                                   p0.b_hip_pos, p1.b_hip_pos,
                                   p0.b_hub_vel, p1.b_hub_vel,
                                   p0.b_hub_mode);

        msg.module_c = buildLegCmd(t, p0, p1,
                                   p0.c_hip_pos, p1.c_hip_pos,
                                   p0.c_hub_vel, p1.c_hub_vel,
                                   p0.c_hub_mode);

        msg.module_d = buildLegCmd(t, p0, p1,
                                   p0.d_hip_pos, p1.d_hip_pos,
                                   p0.d_hub_vel, p1.d_hub_vel,
                                   p0.d_hub_mode);

        pub_motor->publish(msg);
    }

    // ============================================================
    // LegCmd builder
    // ============================================================
    kilin_msgs::msg::LegCmd buildLegCmd(
        double t,
        const GaitPoint &p0, const GaitPoint &p1,
        double hip0, double hip1,
        double hub_v0, double hub_v1,
        double hub_mode0
    ) {
        kilin_msgs::msg::LegCmd leg;

        // Hip interpolation
        leg.hip.position = lerp(t, p0.time, p1.time, hip0, hip1);
        leg.hip.motor_mode = 4;  // POSITION
        leg.hip.kp = 180.0;
        leg.hip.ki = 0.0;
        leg.hip.kd = 5.0;

        // Steering fixed
        leg.steering.position = 0.0;
        leg.steering.motor_mode = 4;

        // Hub ZOH
        leg.hub.velocity = zoh(hub_v0);
        leg.hub.motor_mode = (int)hub_mode0;

        return leg;
    }

    // ============================================================
    // Header
    // ============================================================
    void fillHeader(kilin_msgs::msg::MotorCmdStamped &msg)
    {
        static uint32_t seq = 0;
        msg.header.seq = seq++;
        auto now = this->get_clock()->now();
        msg.header.time.sec = (int32_t) now.seconds();
        msg.header.time.nanosec = now.nanoseconds() % 1000000000;
        msg.header.frame_id = "kilin_csv_control";
    }

    // ============================================================
    // Members
    // ============================================================
    std::vector<GaitPoint> gait;

    rclcpp::Publisher<kilin_msgs::msg::MotorCmdStamped>::SharedPtr pub_motor;
    rclcpp::TimerBase::SharedPtr timer;

    std::string csv_path;

    double rate_hz;
    double delay_start_sec;

    rclcpp::Time start_walltime;
    rclcpp::Time playback_start_time;
    bool started = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinGaitPlayer>());
    rclcpp::shutdown();
    return 0;
}
