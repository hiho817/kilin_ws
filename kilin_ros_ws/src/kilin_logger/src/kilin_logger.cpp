#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/motor_state_stamped.hpp"
#include "kilin_msgs/msg/power_state_stamped.hpp"

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <array>

namespace fs = std::filesystem;

class KilinLogger : public rclcpp::Node {
public:
    KilinLogger() : Node("kilin_logger") {
        // ============================================================
        // Parameters
        // ============================================================
        this->declare_parameter<std::string>("motor_topic", "/motor/state");
        this->declare_parameter<std::string>("power_topic", "/power/state");
        this->declare_parameter<int>("flush_every_n", 20);      // 0: flush every row
        this->declare_parameter<int>("qos_depth", 50);

        // logging path / naming
        // - log_dir: if empty => auto resolve to <kilin_ws>/logs
        // - csv_name: provided by launch/UI
        // - daily_folder: logs/YYYY-MM-DD/
        // - add_suffix_if_exists: avoid overwrite within same day
        this->declare_parameter<std::string>("log_dir", "");
        this->declare_parameter<std::string>("csv_name", "motor_power_state.csv");
        this->declare_parameter<bool>("daily_folder", true);
        this->declare_parameter<bool>("add_suffix_if_exists", true);

        motor_topic   = this->get_parameter("motor_topic").as_string();
        power_topic   = this->get_parameter("power_topic").as_string();
        flush_every_n = this->get_parameter("flush_every_n").as_int();
        qos_depth     = this->get_parameter("qos_depth").as_int();

        log_dir              = this->get_parameter("log_dir").as_string();
        csv_name             = this->get_parameter("csv_name").as_string();
        daily_folder         = this->get_parameter("daily_folder").as_bool();
        add_suffix_if_exists = this->get_parameter("add_suffix_if_exists").as_bool();

        // ============================================================
        // Resolve log directory (KILIN_ws/logs)
        // ============================================================
        // Default:
        //   <KILIN_ws>/logs
        //
        // We derive it from __FILE__:
        //   .../<KILIN_ws>/kilin_ros_ws/src/kilin_logger/src/kilin_logger.cpp
        // Find ancestor folder named "kilin_ros_ws" then go to its parent = <KILIN_ws>
        fs::path base_logs_dir;
        if (log_dir.empty()) {
            base_logs_dir = getKilinWsLogsDir_();
        } else {
            base_logs_dir = fs::path(log_dir);     // allow override (absolute or relative)
        }

        if (daily_folder) {
            base_logs_dir /= makeDateFolder_();    // logs/YYYY-MM-DD
        }

        fs::create_directories(base_logs_dir);

        fs::path out_path = base_logs_dir / csv_name;
        if (add_suffix_if_exists) {
            out_path = makeUniquePath_(out_path);
        }

        csv_path = out_path.string();

        RCLCPP_INFO(get_logger(), "Log dir: %s", base_logs_dir.string().c_str());
        RCLCPP_INFO(get_logger(), "CSV: %s", csv_path.c_str());

        // ============================================================
        // Open CSV file
        // ============================================================
        file.open(csv_path, std::ios::out);
        if (!file.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open CSV file!");
            rclcpp::shutdown();
            return;
        }

        // ============================================================
        // Write CSV header
        // ============================================================
        writeHeader_();
        file.flush();

        // ============================================================
        // QoS
        // ============================================================
        auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort();

        // ============================================================
        // Subscribers
        // ============================================================
        sub_motor = this->create_subscription<kilin_msgs::msg::MotorStateStamped>(
            motor_topic, qos, std::bind(&KilinLogger::onMotorState_, this, std::placeholders::_1));

        sub_power = this->create_subscription<kilin_msgs::msg::PowerStateStamped>(
            power_topic, qos, std::bind(&KilinLogger::onPowerState_, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "kilin_logger started.");
    }

    ~KilinLogger() override {
        closeFile_();
    }

private:
    // ============================================================
    // Utils
    // ============================================================
    static double toTimeSec_(int32_t sec, uint32_t nsec) {
        return static_cast<double>(sec) + static_cast<double>(nsec) * 1e-9;
    }

    std::string makeDateFolder_() {
        std::time_t t = std::time(nullptr);
        std::tm tm{};
        localtime_r(&t, &tm);

        std::ostringstream oss;
        oss << std::put_time(&tm, "%Y-%m-%d");
        return oss.str();
    }

    fs::path makeUniquePath_(const fs::path &p) {
        if (!fs::exists(p)) return p;

        fs::path dir = p.parent_path();
        std::string stem = p.stem().string();
        std::string ext  = p.extension().string();

        for (int k = 1; k < 10000; ++k) {
            std::ostringstream oss;
            oss << stem << "_" << std::setw(3) << std::setfill('0') << k << ext;
            fs::path cand = dir / oss.str();
            if (!fs::exists(cand)) return cand;
        }
        return dir / (stem + "_overflow" + ext);
    }

    fs::path getKilinWsLogsDir_() {
        // Expect:
        //   .../kilin_ws/kilin_ros_ws/src/kilin_logger/src/kilin_logger.cpp
        // We search for folder named "kilin_ws"
        fs::path this_file(__FILE__);

        std::error_code ec;
        fs::path p = fs::absolute(this_file, ec);
        if (ec) {
            p = this_file; // fallback
        }

        fs::path cur = p.parent_path();
        while (!cur.empty()) {
            if (cur.filename() == "kilin_ws") {
                return cur / "logs";
            }
            cur = cur.parent_path();
        }

        // Fallback: current working directory / logs
        return fs::current_path() / "logs";
    }


    void closeFile_() {
        if (closed) return;
        closed = true;

        try { file.flush(); } catch (...) {}
        try { file.close(); } catch (...) {}

        RCLCPP_INFO(get_logger(), "Saved CSV: %s", csv_path.c_str());
    }

    // ============================================================
    // CSV Header
    // ============================================================
    void writeHeader_() {
        // time (motor)
        file << "motor_seq,motor_sec,motor_nanosec,motor_time_sec";

        // motor fields
        const std::vector<std::string> modules = {"a","b","c","d"};
        const std::vector<std::string> joints  = {"hip","steering","hub"};
        const std::vector<std::string> fields  = {"pos","vel","tor","mode","error"};

        for (const auto &m : modules) {
            for (const auto &j : joints) {
                for (const auto &x : fields) {
                    file << ",m" << m << "_" << j << "_" << x;
                }
            }
        }

        // power 12 pairs
        for (int k = 0; k < 12; ++k) {
            file << ",v_" << k << ",i_" << k;
        }

        // power meta
        file << ",power_seq,power_time_sec,power_age_sec";
        file << "\n";
    }

    // ============================================================
    // Power cache
    // ============================================================
    void onPowerState_(const kilin_msgs::msg::PowerStateStamped::SharedPtr msg) {
        if (closed) return;

        last_power_valid = true;

        last_power_seq  = msg->header.seq;
        last_power_tsec = toTimeSec_(msg->header.time.sec, msg->header.time.nanosec);

        last_vi[0]  = msg->v_0;   last_vi[1]  = msg->i_0;
        last_vi[2]  = msg->v_1;   last_vi[3]  = msg->i_1;
        last_vi[4]  = msg->v_2;   last_vi[5]  = msg->i_2;
        last_vi[6]  = msg->v_3;   last_vi[7]  = msg->i_3;
        last_vi[8]  = msg->v_4;   last_vi[9]  = msg->i_4;
        last_vi[10] = msg->v_5;   last_vi[11] = msg->i_5;
        last_vi[12] = msg->v_6;   last_vi[13] = msg->i_6;
        last_vi[14] = msg->v_7;   last_vi[15] = msg->i_7;
        last_vi[16] = msg->v_8;   last_vi[17] = msg->i_8;
        last_vi[18] = msg->v_9;   last_vi[19] = msg->i_9;
        last_vi[20] = msg->v_10;  last_vi[21] = msg->i_10;
        last_vi[22] = msg->v_11;  last_vi[23] = msg->i_11;
    }

    // ============================================================
    // Main logging (triggered by motor/state)
    // ============================================================
    void onMotorState_(const kilin_msgs::msg::MotorStateStamped::SharedPtr msg) {
        if (closed) return;

        // ----------------------------
        // Time (motor)
        // ----------------------------
        const auto m_seq  = msg->header.seq;
        const auto m_sec  = msg->header.time.sec;
        const auto m_nsec = msg->header.time.nanosec;
        const double m_t  = toTimeSec_(m_sec, m_nsec);

        file << m_seq << "," << m_sec << "," << m_nsec << ",";
        file << std::fixed << std::setprecision(9) << m_t;

        // ----------------------------
        // Motor flatten
        // ----------------------------
        auto append_state = [&](const auto &s) {
            file << "," << s.position
                 << "," << s.velocity
                 << "," << s.torque
                 << "," << static_cast<int>(s.motor_mode)
                 << "," << static_cast<int>(s.error_code);
        };

        const auto &A = msg->module_a;
        const auto &B = msg->module_b;
        const auto &C = msg->module_c;
        const auto &D = msg->module_d;

        append_state(A.hip);      append_state(A.steering);      append_state(A.hub);
        append_state(B.hip);      append_state(B.steering);      append_state(B.hub);
        append_state(C.hip);      append_state(C.steering);      append_state(C.hub);
        append_state(D.hip);      append_state(D.steering);      append_state(D.hub);

        // ----------------------------
        // Append latest power (cached)
        // ----------------------------
        if (last_power_valid) {
            for (int i = 0; i < 24; ++i) {
                file << "," << last_vi[i];
            }
            const double age = m_t - last_power_tsec;
            file << "," << last_power_seq
                 << "," << std::fixed << std::setprecision(9) << last_power_tsec
                 << "," << std::fixed << std::setprecision(9) << age;
        } else {
            for (int i = 0; i < 24; ++i) {
                file << ",";
            }
            file << ",,,"; // power_seq, power_time_sec, power_age_sec
        }

        file << "\n";

        motor_count++;
        if (flush_every_n <= 0 || (motor_count % flush_every_n == 0)) {
            file.flush();
        }
    }

    // ============================================================
    // Members
    // ============================================================
    std::string motor_topic;
    std::string power_topic;
    int flush_every_n = 20;
    int qos_depth = 50;

    // log settings
    std::string log_dir;
    std::string csv_name;
    bool daily_folder = true;
    bool add_suffix_if_exists = true;

    std::string csv_path;
    std::ofstream file;

    uint64_t motor_count = 0;
    bool closed = false;

    rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr sub_motor;
    rclcpp::Subscription<kilin_msgs::msg::PowerStateStamped>::SharedPtr sub_power;

    // power cache
    bool last_power_valid = false;
    uint32_t last_power_seq = 0;
    double last_power_tsec = 0.0;
    std::array<double, 24> last_vi{}; // [v0,i0,v1,i1,...,v11,i11]
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinLogger>());
    rclcpp::shutdown();
    return 0;
}
