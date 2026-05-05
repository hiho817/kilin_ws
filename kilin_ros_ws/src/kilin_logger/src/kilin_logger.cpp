#include "rclcpp/rclcpp.hpp"
#include "kilin_msgs/msg/motor_cmd_stamped.hpp"
#include "kilin_msgs/msg/motor_state_stamped.hpp"
#include "kilin_msgs/msg/power_state_stamped.hpp"
#include "kilin_msgs/msg/trigger_stamped.hpp"

#include <deque>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <filesystem>
#include <ctime>
#include <array>
#include <atomic>
#include <unordered_map>
#include <chrono>

namespace fs = std::filesystem;

class KilinLogger : public rclcpp::Node {
public:
    KilinLogger() : Node("kilin_logger") {
        // ============================================================
        // Parameters
        // ============================================================
        this->declare_parameter<std::string>("motor_cmd_topic", "/motor/command");
        this->declare_parameter<std::string>("motor_topic", "/motor/state");
        this->declare_parameter<std::string>("power_topic", "/power/state");
        this->declare_parameter<int>("flush_every_n", 20);      // 0: flush every row
        this->declare_parameter<int>("qos_depth", 50);

        // trigger control
        // - use_trigger=false: start logging immediately (original behavior)
        // - use_trigger=true : wait for /kilin/trigger enable=true to start logging
        //                      and enable=false to stop logging after stop_grace_sec
        this->declare_parameter<bool>("use_trigger", false);
        this->declare_parameter<std::string>("trigger_topic", "/kilin/trigger");
        this->declare_parameter<double>("shutdown_delay_sec", 3.0);
        this->declare_parameter<double>("stop_grace_sec", 0.5);

        // logging path / naming
        // - log_dir: if empty => auto resolve to <kilin_ws>/logs
        // - csv_name: provided by launch/UI
        // - daily_folder: logs/YYYY-MM-DD/
        // - add_suffix_if_exists: avoid overwrite within same day
        this->declare_parameter<std::string>("log_dir", "");
        this->declare_parameter<std::string>("csv_name", "motor_power_state.csv");
        this->declare_parameter<bool>("daily_folder", true);
        this->declare_parameter<bool>("add_suffix_if_exists", true);

        // seq-join policy
        // - pair_timeout_sec: wait up to this long for matching power(seq) after motor(seq) arrives
        //   if timeout, log motor only (leave power columns empty)
        this->declare_parameter<double>("pair_timeout_sec", 0.5);
        this->declare_parameter<int>("cmd_buffer_size", 200);

        motor_cmd_topic = this->get_parameter("motor_cmd_topic").as_string();
        motor_topic   = this->get_parameter("motor_topic").as_string();
        power_topic   = this->get_parameter("power_topic").as_string();
        flush_every_n = this->get_parameter("flush_every_n").as_int();
        qos_depth     = this->get_parameter("qos_depth").as_int();

        use_trigger_        = this->get_parameter("use_trigger").as_bool();
        trigger_topic_      = this->get_parameter("trigger_topic").as_string();
        shutdown_delay_sec_ = this->get_parameter("shutdown_delay_sec").as_double();
        stop_grace_sec_     = this->get_parameter("stop_grace_sec").as_double();

        log_dir              = this->get_parameter("log_dir").as_string();
        csv_name             = this->get_parameter("csv_name").as_string();
        daily_folder         = this->get_parameter("daily_folder").as_bool();
        add_suffix_if_exists = this->get_parameter("add_suffix_if_exists").as_bool();

        pair_timeout_sec_ = this->get_parameter("pair_timeout_sec").as_double();
        cmd_buffer_size_ = this->get_parameter("cmd_buffer_size").as_int();

        // ============================================================
        // Resolve log directory (KILIN_ws/logs)
        // ============================================================
        // Default:
        //   <KILIN_ws>/logs
        //
        // We derive it from __FILE__:
        //   .../<kilin_ws>/kilin_ros_ws/src/kilin_logger/src/kilin_logger.cpp
        // Find ancestor folder named "kilin_ws"
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
        // Note:
        // - Even in trigger mode, we open the file and write header first.
        // - Data rows will be gated by logging_enabled_.
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
        // Initial logging state
        // ============================================================
        if (use_trigger_) {
            logging_enabled_ = false;
            RCLCPP_INFO(get_logger(), "Mode: TRIGGER (waiting for %s enable=true)", trigger_topic_.c_str());
        } else {
            logging_enabled_ = true;
            RCLCPP_INFO(get_logger(), "Mode: DIRECT (start logging immediately)");
        }

        // ============================================================
        // QoS
        // ============================================================
        // For high-rate states, best_effort can reduce latency and avoid backpressure.
        // If you need strict completeness, change to reliable().
        auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth)).best_effort();

        // Trigger is a state signal: must be reliable + transient_local to avoid missing
        // the last enable state (ROS1 latched-like).
        auto trig_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

        // ============================================================
        // Subscribers
        // ============================================================
        sub_motor_cmd = this->create_subscription<kilin_msgs::msg::MotorCmdStamped>(
            motor_cmd_topic, qos, std::bind(&KilinLogger::onMotorCmd_, this, std::placeholders::_1));

        sub_motor = this->create_subscription<kilin_msgs::msg::MotorStateStamped>(
            motor_topic, qos, std::bind(&KilinLogger::onMotorState_, this, std::placeholders::_1));

        sub_power = this->create_subscription<kilin_msgs::msg::PowerStateStamped>(
            power_topic, qos, std::bind(&KilinLogger::onPowerState_, this, std::placeholders::_1));

        if (use_trigger_) {
            sub_trigger = this->create_subscription<kilin_msgs::msg::TriggerStamped>(
                trigger_topic_, trig_qos, std::bind(&KilinLogger::onTrigger_, this, std::placeholders::_1));
        }

        // ============================================================
        // Pairing cleanup timer (timeout flush)
        // ============================================================
        // We periodically:
        // - Flush motor-only rows if power(seq) didn't arrive within pair_timeout_sec_
        // - Drop stale power pending entries to cap memory usage
        cleanup_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&KilinLogger::onCleanupTimer_, this)
        );

        RCLCPP_INFO(
            get_logger(),
            "kilin_logger started. pair_timeout_sec=%.3f cmd_buffer_size=%d stop_grace_sec=%.3f",
            pair_timeout_sec_, cmd_buffer_size_, stop_grace_sec_);
    }

    ~KilinLogger() override {
        closeFile_();
    }

private:
    // ============================================================
    // Types for seq-join buffering
    // ============================================================
    struct PendingMotor {
        kilin_msgs::msg::MotorStateStamped msg;
        std::chrono::steady_clock::time_point t_arrival;
    };

    struct PendingCmd {
        kilin_msgs::msg::MotorCmdStamped msg;
        double tsec = 0.0;
    };

    struct PendingPower {
        uint32_t seq = 0;
        double tsec = 0.0;
        std::array<double, 24> vi{};
        std::chrono::steady_clock::time_point t_arrival;
    };

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

        // Clear pending buffers (avoid any future writes)
        cmd_pending_.clear();
        motor_pending_.clear();
        power_pending_.clear();

        try { file.flush(); } catch (...) {}
        try { file.close(); } catch (...) {}

        RCLCPP_INFO(get_logger(), "Saved CSV: %s", csv_path.c_str());
    }

    // ============================================================
    // Trigger callback (for gating)
    // ============================================================
    void onTrigger_(const kilin_msgs::msg::TriggerStamped::SharedPtr msg) {
        if (closed) return;

        if (msg->enable) {
            // START logging
            if (stopping_) {
                stopping_ = false;
                if (stop_timer_) {
                    stop_timer_->cancel();
                }

                RCLCPP_INFO(get_logger(), "Trigger enable=true -> CANCEL stop grace period");
            }

            if (!logging_enabled_) {
                logging_enabled_ = true;

                // Clear buffers when (re)starting to avoid mixing old pending data
                cmd_pending_.clear();
                motor_pending_.clear();
                power_pending_.clear();

                RCLCPP_INFO(get_logger(), "Trigger enable=true -> START logging");
            }
        } else {
            // STOP logging
            if (logging_enabled_ && !stopping_) {
                stopping_ = true;

                RCLCPP_INFO(
                    get_logger(),
                    "Trigger enable=false -> KEEP logging for %.3f sec, then close file and shutdown in %.1f sec",
                    stop_grace_sec_, shutdown_delay_sec_);

                // Do not close file immediately.
                // Keep logging for stop_grace_sec_ to collect late motor/state from sbRIO.
                scheduleStop_(stop_grace_sec_);
            }
        }
    }

    void scheduleStop_(double delay_sec) {
        auto delay_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(delay_sec));

        stop_timer_ = this->create_wall_timer(
            delay_ns,
            [this]() {
                if (closed) return;

                if (stop_timer_) {
                    stop_timer_->cancel();
                }

                RCLCPP_INFO(this->get_logger(), "Stop grace period done -> flush pending rows and close file");

                // Flush pending motor rows before closing.
                flushAllPending_();

                logging_enabled_ = false;
                stopping_ = false;

                closeFile_();

                // shutdown after delay
                scheduleShutdown_(shutdown_delay_sec_);
            }
        );
    }

    void scheduleShutdown_(double delay_sec) {
        if (shutdown_scheduled_) return;
        shutdown_scheduled_ = true;

        auto delay_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(delay_sec));

        shutdown_timer_ = this->create_wall_timer(
            delay_ns,
            [this]() {
                RCLCPP_INFO(this->get_logger(), "Shutdown now.");
                rclcpp::shutdown();
            }
        );
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

        // command fields are appended to preserve the original CSV prefix.
        file << ",cmd_seq,cmd_sec,cmd_nanosec,cmd_time_sec,cmd_state_dt_sec";

        const std::vector<std::string> cmd_fields = {"pos_cmd","tor_cmd","vel_cmd","mode_cmd"};
        for (const auto &m : modules) {
            for (const auto &j : joints) {
                for (const auto &x : cmd_fields) {
                    file << ",m" << m << "_" << j << "_" << x;
                }
            }
        }
        file << "\n";
    }

    // ============================================================
    // Command buffer
    // ============================================================
    void onMotorCmd_(const kilin_msgs::msg::MotorCmdStamped::SharedPtr msg) {
        if (closed) return;
        if (!logging_enabled_) return;

        PendingCmd c;
        c.msg = *msg;
        c.tsec = toTimeSec_(msg->header.time.sec, msg->header.time.nanosec);
        cmd_pending_.push_back(c);

        trimCommandBuffer_();
    }

    // ============================================================
    // Power pending (buffer by seq)
    // ============================================================
    void onPowerState_(const kilin_msgs::msg::PowerStateStamped::SharedPtr msg) {
        if (closed) return;

        // Gate by trigger
        if (!logging_enabled_) return;

        PendingPower p;
        p.seq = msg->header.seq;
        p.tsec = toTimeSec_(msg->header.time.sec, msg->header.time.nanosec);
        p.t_arrival = std::chrono::steady_clock::now();

        p.vi[0]  = msg->v_0;   p.vi[1]  = msg->i_0;
        p.vi[2]  = msg->v_1;   p.vi[3]  = msg->i_1;
        p.vi[4]  = msg->v_2;   p.vi[5]  = msg->i_2;
        p.vi[6]  = msg->v_3;   p.vi[7]  = msg->i_3;
        p.vi[8]  = msg->v_4;   p.vi[9]  = msg->i_4;
        p.vi[10] = msg->v_5;   p.vi[11] = msg->i_5;
        p.vi[12] = msg->v_6;   p.vi[13] = msg->i_6;
        p.vi[14] = msg->v_7;   p.vi[15] = msg->i_7;
        p.vi[16] = msg->v_8;   p.vi[17] = msg->i_8;
        p.vi[18] = msg->v_9;   p.vi[19] = msg->i_9;
        p.vi[20] = msg->v_10;  p.vi[21] = msg->i_10;
        p.vi[22] = msg->v_11;  p.vi[23] = msg->i_11;

        power_pending_[p.seq] = p;

        // Try flush if matching motor(seq) already arrived
        tryFlushPair_(p.seq);
    }

    // ============================================================
    // Motor pending (triggered by motor/state)
    // ============================================================
    void onMotorState_(const kilin_msgs::msg::MotorStateStamped::SharedPtr msg) {
        if (closed) return;

        // Gate by trigger
        if (!logging_enabled_) return;

        PendingMotor m;
        m.msg = *msg;
        m.t_arrival = std::chrono::steady_clock::now();
        motor_pending_[msg->header.seq] = m;

        // Try flush if matching power(seq) already arrived
        tryFlushPair_(msg->header.seq);
    }

    // ============================================================
    // Flush a paired row (same seq)
    // ============================================================
    void tryFlushPair_(uint32_t seq) {
        if (closed) return;
        if (!logging_enabled_) return;

        auto itM = motor_pending_.find(seq);
        auto itP = power_pending_.find(seq);
        if (itM == motor_pending_.end() || itP == power_pending_.end()) return;

        writeRow_(itM->second.msg, /*has_power=*/true, &itP->second);

        motor_pending_.erase(itM);
        power_pending_.erase(itP);
    }

    // ============================================================
    // Timeout flush: motor-only if power not available within pair_timeout_sec_
    // ============================================================
    void onCleanupTimer_() {
        if (closed) return;
        if (!logging_enabled_) return;

        const auto now = std::chrono::steady_clock::now();
        const auto timeout = std::chrono::duration<double>(pair_timeout_sec_);

        // ----------------------------
        // 1) motor-only flush on timeout
        // ----------------------------
        // Note: erase while iterating
        for (auto it = motor_pending_.begin(); it != motor_pending_.end(); ) {
            const auto age = now - it->second.t_arrival;
            if (age >= timeout) {
                // Timeout: log motor only, do not log power
                writeRow_(it->second.msg, /*has_power=*/false, nullptr);
                it = motor_pending_.erase(it);
            } else {
                ++it;
            }
        }

        // ----------------------------
        // 2) drop stale power pending (no matching motor arrived)
        // ----------------------------
        for (auto it = power_pending_.begin(); it != power_pending_.end(); ) {
            const auto age = now - it->second.t_arrival;
            if (age >= timeout) {
                it = power_pending_.erase(it);
            } else {
                ++it;
            }
        }

        trimCommandBuffer_();
    }

    // ============================================================
    // Flush all remaining pending rows before close
    // ============================================================
    void flushAllPending_() {
        // ----------------------------
        // 1) flush currently paired rows first
        // ----------------------------
        std::vector<uint32_t> paired_seqs;
        paired_seqs.reserve(motor_pending_.size());

        for (const auto &kv : motor_pending_) {
            const uint32_t seq = kv.first;
            if (power_pending_.find(seq) != power_pending_.end()) {
                paired_seqs.push_back(seq);
            }
        }

        for (const auto seq : paired_seqs) {
            auto itM = motor_pending_.find(seq);
            auto itP = power_pending_.find(seq);

            if (itM != motor_pending_.end() && itP != power_pending_.end()) {
                writeRow_(itM->second.msg, /*has_power=*/true, &itP->second);
                motor_pending_.erase(itM);
                power_pending_.erase(itP);
            }
        }

        // ----------------------------
        // 2) flush remaining motor-only rows
        // ----------------------------
        for (auto &kv : motor_pending_) {
            writeRow_(kv.second.msg, /*has_power=*/false, nullptr);
        }
        motor_pending_.clear();

        // ----------------------------
        // 3) drop power-only rows
        // ----------------------------
        power_pending_.clear();

        try { file.flush(); } catch (...) {}
    }

    void trimCommandBuffer_() {
        if (cmd_buffer_size_ <= 0) {
            cmd_pending_.clear();
            return;
        }

        while (static_cast<int>(cmd_pending_.size()) > cmd_buffer_size_) {
            cmd_pending_.pop_front();
        }
    }

    bool popMatchingCmd_(PendingCmd &out_cmd) {
        if (cmd_pending_.empty()) {
            return false;
        }

        out_cmd = cmd_pending_.front();
        cmd_pending_.pop_front();
        return true;
    }

    // ============================================================
    // Write one CSV row
    // - has_power=true : append power from PendingPower
    // - has_power=false: leave power columns empty
    // ============================================================
    void writeRow_(const kilin_msgs::msg::MotorStateStamped &m,
                   bool has_power,
                   const PendingPower *p) {
        const double m_t  = toTimeSec_(m.header.time.sec, m.header.time.nanosec);
        PendingCmd c;
        const bool has_cmd = popMatchingCmd_(c);

        // ----------------------------
        // Time (motor)
        // ----------------------------
        const auto m_seq  = m.header.seq;
        const auto m_sec  = m.header.time.sec;
        const auto m_nsec = m.header.time.nanosec;

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

        const auto &A = m.module_a;
        const auto &B = m.module_b;
        const auto &C = m.module_c;
        const auto &D = m.module_d;

        append_state(A.hip);      append_state(A.steering);      append_state(A.hub);
        append_state(B.hip);      append_state(B.steering);      append_state(B.hub);
        append_state(C.hip);      append_state(C.steering);      append_state(C.hub);
        append_state(D.hip);      append_state(D.steering);      append_state(D.hub);

        // ----------------------------
        // Append power (paired by seq) or leave empty
        // ----------------------------
        if (has_power && p != nullptr) {
            for (int i = 0; i < 24; ++i) {
                file << "," << p->vi[i];
            }
            const double age = m_t - p->tsec;
            file << "," << p->seq
                 << "," << std::fixed << std::setprecision(9) << p->tsec
                 << "," << std::fixed << std::setprecision(9) << age;
        } else {
            // 24 power numbers
            for (int i = 0; i < 24; ++i) {
                file << ",";
            }
            // power_seq, power_time_sec, power_age_sec
            file << ",,,";
        }

        if (has_cmd) {
            const auto c_seq = c.msg.header.seq;
            const auto c_sec = c.msg.header.time.sec;
            const auto c_nsec = c.msg.header.time.nanosec;
            const double c_t = c.tsec;
            const double dt = m_t - c_t;

            file << "," << c_seq << "," << c_sec << "," << c_nsec << ",";
            file << std::fixed << std::setprecision(9) << c_t << ",";
            file << std::fixed << std::setprecision(9) << dt;

            auto append_cmd = [&](const auto &s) {
                file << "," << s.position
                     << "," << s.torque
                     << "," << s.velocity
                     << "," << static_cast<int>(s.motor_mode);
            };

            const auto &A_cmd = c.msg.module_a;
            const auto &B_cmd = c.msg.module_b;
            const auto &C_cmd = c.msg.module_c;
            const auto &D_cmd = c.msg.module_d;

            append_cmd(A_cmd.hip);      append_cmd(A_cmd.steering);      append_cmd(A_cmd.hub);
            append_cmd(B_cmd.hip);      append_cmd(B_cmd.steering);      append_cmd(B_cmd.hub);
            append_cmd(C_cmd.hip);      append_cmd(C_cmd.steering);      append_cmd(C_cmd.hub);
            append_cmd(D_cmd.hip);      append_cmd(D_cmd.steering);      append_cmd(D_cmd.hub);
        } else {
            file << ",,,,";
            for (int i = 0; i < 48; ++i) {
                file << ",";
            }
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
    std::string motor_cmd_topic;
    std::string motor_topic;
    std::string power_topic;
    int flush_every_n = 20;
    int qos_depth = 50;

    // trigger control
    bool use_trigger_ = false;
    std::string trigger_topic_;
    double shutdown_delay_sec_ = 3.0;
    double stop_grace_sec_ = 0.5;

    std::atomic<bool> logging_enabled_{false};
    bool stopping_ = false;
    bool shutdown_scheduled_ = false;
    rclcpp::TimerBase::SharedPtr stop_timer_;
    rclcpp::TimerBase::SharedPtr shutdown_timer_;

    // seq-join policy
    double pair_timeout_sec_ = 0.5;
    int cmd_buffer_size_ = 200;

    // log settings
    std::string log_dir;
    std::string csv_name;
    bool daily_folder = true;
    bool add_suffix_if_exists = true;

    std::string csv_path;
    std::ofstream file;

    uint64_t motor_count = 0;
    bool closed = false;

    rclcpp::Subscription<kilin_msgs::msg::MotorCmdStamped>::SharedPtr sub_motor_cmd;
    rclcpp::Subscription<kilin_msgs::msg::MotorStateStamped>::SharedPtr sub_motor;
    rclcpp::Subscription<kilin_msgs::msg::PowerStateStamped>::SharedPtr sub_power;
    rclcpp::Subscription<kilin_msgs::msg::TriggerStamped>::SharedPtr sub_trigger;

    // pairing buffers
    std::deque<PendingCmd> cmd_pending_;
    std::unordered_map<uint32_t, PendingMotor> motor_pending_;
    std::unordered_map<uint32_t, PendingPower> power_pending_;

    // cleanup timer
    rclcpp::TimerBase::SharedPtr cleanup_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KilinLogger>());
    rclcpp::shutdown();
    return 0;
}