#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/leg_cmd.hpp"
#include "kilin_msgs/msg/motor_cmd.hpp"
#include "kilin_msgs/msg/trigger_stamped.hpp"

#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <atomic>
#include <csignal>

// Trigger GPIO (libgpiod)
#include <gpiod.h>
#include <mutex>

static std::atomic_bool g_sigint_requested{false};

static void sigintHandler(int) {
    g_sigint_requested.store(true);
}

struct GaitPoint {
    double time;

    double a_hip_pos, a_steer_pos, a_hub_vel, a_hub_mode;
    double b_hip_pos, b_steer_pos, b_hub_vel, b_hub_mode;
    double c_hip_pos, c_steer_pos, c_hub_vel, c_hub_mode;
    double d_hip_pos, d_steer_pos, d_hub_vel, d_hub_mode;
};

class KilinGaitPlayer : public rclcpp::Node {
public:
    KilinGaitPlayer() : Node("kilin_csv_control") {
        this->declare_parameter<std::string>("csv_path", "gait.csv");
        this->declare_parameter<double>("rate_hz", 200.0);
        this->declare_parameter<double>("delay_start_sec", 3.0);

        // Trigger GPIO params (ACTIVE-LOW: LOW=ON, HIGH=OFF)
        this->declare_parameter<std::string>("trigger_chip", "/dev/gpiochip0");
        this->declare_parameter<int>("trigger_line", 112);

        csv_path = this->get_parameter("csv_path").as_string();
        rate_hz = this->get_parameter("rate_hz").as_double();
        delay_start_sec = this->get_parameter("delay_start_sec").as_double();

        trigger_chipname_ = this->get_parameter("trigger_chip").as_string();
        trigger_line_offset_ = (unsigned int)this->get_parameter("trigger_line").as_int();

        if (!loadCSV(csv_path)) {
            RCLCPP_ERROR(get_logger(), "Failed to load CSV file: %s", csv_path.c_str());
            rclcpp::shutdown();
            return;
        }

        pub_motor = this->create_publisher<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
        
        pub_trigger = this->create_publisher<kilin_msgs::msg::TriggerStamped>(
            "/kilin/trigger", qos);

        start_time_ = this->get_clock()->now();

        RCLCPP_INFO(get_logger(), "CSV playback starting in %.1f seconds...", delay_start_sec);
        RCLCPP_INFO(get_logger(), "Note: If use_sim_time:=true, playback will sync with /clock from Isaac Sim.");

        // Use rclcpp::create_timer instead of create_wall_timer
        // When use_sim_time:=true, it follows /clock (sim time)
        // When use_sim_time:=false, it follows system time (real robot)
        timer = rclcpp::create_timer(
            this,
            this->get_clock(),
            std::chrono::microseconds((int)(1e6 / rate_hz)),
            std::bind(&KilinGaitPlayer::update, this)
        );
    }

    ~KilinGaitPlayer() override {
        // Make sure trigger goes back to safe state even if node stops early
        triggerOff();
        cleanupTriggerGPIO();
    }

private:
    void publishTriggerCommand(bool enable) {
        if (!pub_trigger) {
            return;
        }

        kilin_msgs::msg::TriggerStamped trigger_msg;
        trigger_msg.enable = enable;
        pub_trigger->publish(trigger_msg);
    }

    void publishTriggerFalseOnce(const char *reason) {
        if (trigger_false_sent_) {
            return;
        }

        publishTriggerCommand(false);
        trigger_false_sent_ = true;
        RCLCPP_INFO(get_logger(), "Published trigger=false (%s).", reason);
    }

    // ============================================================
    // Trigger GPIO (ACTIVE-LOW)
    // ============================================================
    bool initTriggerGPIO() {
        std::lock_guard<std::mutex> lk(trigger_mtx_);

        if (trigger_inited_) {
            return true;
        }

        trigger_chip_ = gpiod_chip_open(trigger_chipname_.c_str());
        if (!trigger_chip_) {
            RCLCPP_ERROR(get_logger(), "Failed to open %s", trigger_chipname_.c_str());
            return false;
        }

        trigger_line_ = gpiod_chip_get_line(trigger_chip_, trigger_line_offset_);
        if (!trigger_line_) {
            RCLCPP_ERROR(get_logger(), "Failed to get GPIO line %u", trigger_line_offset_);
            gpiod_chip_close(trigger_chip_);
            trigger_chip_ = nullptr;
            return false;
        }

        // request as output, default HIGH (safe: trigger OFF)
        if (gpiod_line_request_output(trigger_line_, "kilin_csv_trigger", 1) < 0) {
            RCLCPP_ERROR(get_logger(), "Failed to request GPIO line as output");
            gpiod_chip_close(trigger_chip_);
            trigger_chip_ = nullptr;
            trigger_line_ = nullptr;
            return false;
        }

        trigger_inited_ = true;
        trigger_is_on_ = false;

        RCLCPP_INFO(get_logger(),
            "Trigger GPIO ready. ACTIVE-LOW (LOW=ON, HIGH=OFF). chip=%s line=%u",
            trigger_chipname_.c_str(), trigger_line_offset_);

        return true;
    }

    void triggerOn() {
        if (!initTriggerGPIO()) {
            RCLCPP_WARN(get_logger(), "Trigger ON skipped (GPIO not ready).");
            return;
        }

        std::lock_guard<std::mutex> lk(trigger_mtx_);

        if (trigger_is_on_) {
            return;
        }

        // ACTIVE-LOW: LOW = trigger ON
        gpiod_line_set_value(trigger_line_, 0);
        trigger_is_on_ = true;
        RCLCPP_INFO(get_logger(), "Trigger ON (GPIO LOW).");
    }

    void triggerOff() {
        if (!trigger_inited_) return;
        std::lock_guard<std::mutex> lk(trigger_mtx_);
        gpiod_line_set_value(trigger_line_, 1); // HIGH = OFF
        trigger_is_on_ = false;
        RCLCPP_INFO(get_logger(), "Trigger OFF (GPIO HIGH).");
    }

    void cleanupTriggerGPIO() {
        std::lock_guard<std::mutex> lk(trigger_mtx_);

        if (trigger_line_) {
            // safe state
            gpiod_line_set_value(trigger_line_, 1);
            gpiod_line_release(trigger_line_);
            trigger_line_ = nullptr;
        }
        if (trigger_chip_) {
            gpiod_chip_close(trigger_chip_);
            trigger_chip_ = nullptr;
        }
        trigger_inited_ = false;
        trigger_is_on_ = false;
    }

    // ============================================================
    // CSV Loader
    // ============================================================
    bool loadCSV(const std::string &path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            return false;
        }

        std::string line;
        getline(file, line); // header

        bool csv_has_steer = false;
        while (getline(file, line)) {
            std::stringstream ss(line);
            std::string cell;
            std::vector<double> nums;

            while (getline(ss, cell, ',')) {
                nums.push_back(std::stod(cell));
            }

            if (nums.size() != 13 && nums.size() != 17) {
                RCLCPP_ERROR(get_logger(),
                    "CSV row has %zu columns; expected 13 (no steering) or 17 (with steering)!", nums.size());
                return false;
            }

            int k = 0;
            GaitPoint p;
            bool has_steer = (nums.size() == 17);
            csv_has_steer = has_steer;

            p.time = nums[k++];

            // Module A
            p.a_hip_pos = nums[k++];
            if (has_steer) p.a_steer_pos = nums[k++]; else p.a_steer_pos = 0.0;
            p.a_hub_vel = nums[k++];
            p.a_hub_mode = nums[k++];

            // Module B
            p.b_hip_pos = nums[k++];
            if (has_steer) p.b_steer_pos = nums[k++]; else p.b_steer_pos = 0.0;
            p.b_hub_vel = nums[k++];
            p.b_hub_mode = nums[k++];

            // Module C
            p.c_hip_pos = nums[k++];
            if (has_steer) p.c_steer_pos = nums[k++]; else p.c_steer_pos = 0.0;
            p.c_hub_vel = nums[k++];
            p.c_hub_mode = nums[k++];

            // Module D
            p.d_hip_pos = nums[k++];
            if (has_steer) p.d_steer_pos = nums[k++]; else p.d_steer_pos = 0.0;
            p.d_hub_vel = nums[k++];
            p.d_hub_mode = nums[k++];

            gait.push_back(p);
        }

        size_t cols = gait.empty() ? 0 : (csv_has_steer ? 17 : 13);
        RCLCPP_INFO(get_logger(), "Loaded %zu gait points from CSV. (Columns: %zu)", gait.size(), cols);
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
    void update() {
        rclcpp::Time now = this->get_clock()->now();

        if (g_sigint_requested.load()) {
            RCLCPP_WARN(get_logger(), "SIGINT received. Sending trigger=false then shutting down.");
            triggerOff();
            publishTriggerFalseOnce("SIGINT");
            rclcpp::shutdown();
            return;
        }

        // Check whether time is valid (in use_sim_time mode, /clock may not be published yet)
        if (now.seconds() == 0.0) {
            RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 2000,
                "Waiting for valid clock... (Is /clock being published?)");
            return;
        }

        double elapsed = (now - start_time_).seconds();

        // --------------------------
        // Delay start
        // --------------------------
        if (elapsed < delay_start_sec) {
            return;
        }

        if (!started) {
            RCLCPP_INFO(get_logger(), "CSV playback started.");
            playback_start_time = now;
            started = true;

            // Trigger ON right when playback starts (keep ON until finished)
            triggerOn();
            publishTriggerCommand(true);
        }

        double t = (now - playback_start_time).seconds();

        // --------------------------
        // Check if finished
        // --------------------------
        if (t > gait.back().time) {
            // Turn trigger OFF only after finishing the last point
            triggerOff();
            publishTriggerFalseOnce("playback complete");

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
                                   p0.a_steer_pos, p1.a_steer_pos,
                                   p0.a_hub_vel,
                                   p0.a_hub_mode);

        msg.module_b = buildLegCmd(t, p0, p1,
                                   p0.b_hip_pos, p1.b_hip_pos,
                                   p0.b_steer_pos, p1.b_steer_pos,
                                   p0.b_hub_vel,
                                   p0.b_hub_mode);

        msg.module_c = buildLegCmd(t, p0, p1,
                                   p0.c_hip_pos, p1.c_hip_pos,
                                   p0.c_steer_pos, p1.c_steer_pos,
                                   p0.c_hub_vel,
                                   p0.c_hub_mode);

        msg.module_d = buildLegCmd(t, p0, p1,
                                   p0.d_hip_pos, p1.d_hip_pos,
                                   p0.d_steer_pos, p1.d_steer_pos,
                                   p0.d_hub_vel,
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
        double steer0, double steer1,
        double hub_v0,
        double hub_mode0,
        bool is_flip = false
    ) {
        kilin_msgs::msg::LegCmd leg;

        // Hip interpolation
        double hip_deg = lerp(t, p0.time, p1.time, hip0, hip1);
        if (is_flip) {
            hip_deg = -hip_deg;
        }
        double hip_rad = hip_deg * M_PI / 180.0;
        leg.hip.position = hip_rad;
        leg.hip.motor_mode = 4;  // POSITION
        leg.hip.kp = 350.0;
        leg.hip.ki = 0.0;
        leg.hip.kd = 5.0;

        // Steering interpolation
        double steer_deg = lerp(t, p0.time, p1.time, steer0, steer1);
        double steer_rad = steer_deg * M_PI / 180.0;
        leg.steering.position = steer_rad;
        leg.steering.motor_mode = 4;

        // Hub ZOH
        leg.hub.velocity = zoh(hub_v0);
        leg.hub.motor_mode = (int)hub_mode0;

        return leg;
    }

    // ============================================================
    // Header
    // ============================================================
    void fillHeader(kilin_msgs::msg::MotorCmdStamped &msg) {
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
    rclcpp::Publisher<kilin_msgs::msg::TriggerStamped>::SharedPtr pub_trigger;
    rclcpp::TimerBase::SharedPtr timer;

    std::string csv_path;

    double rate_hz;
    double delay_start_sec;

    rclcpp::Time start_time_;
    rclcpp::Time playback_start_time;
    bool started = false;

    // Trigger GPIO members
    std::string trigger_chipname_;
    unsigned int trigger_line_offset_ = 112;

    gpiod_chip* trigger_chip_ = nullptr;
    gpiod_line* trigger_line_ = nullptr;

    std::mutex trigger_mtx_;
    bool trigger_inited_ = false;
    bool trigger_is_on_ = false;
    bool trigger_false_sent_ = false;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGINT, sigintHandler);
    auto node = std::make_shared<KilinGaitPlayer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
