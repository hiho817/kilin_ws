#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <kilin_msgs/msg/motor_cmd_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <kilin_msgs/msg/motor_state_stamped.hpp>

#include <algorithm>
#include <cmath>
#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <ctime>

namespace fs = std::filesystem;

// 定義單位轉換常數
// 你的程式邏輯: velocity = wheel_speed_rad_s / (2pi) * 60 * 10
// 這裡做逆向轉換: rad_s = velocity / 10 / 60 * 2pi
static constexpr double RPM10_TO_RADS = (1.0 / 10.0) * (2.0 * M_PI / 60.0);

class IsaacConverter : public rclcpp::Node {
public:
    IsaacConverter() : Node("isaac_converter") {
        // ============================================================
        // Parameters
        // ============================================================
        this->declare_parameter<std::string>("log_dir", "");
        this->declare_parameter<std::string>("csv_name", "isaac_sim_data.csv");
        this->declare_parameter<bool>("daily_folder", true);
        this->declare_parameter<bool>("add_suffix_if_exists", true);
        this->declare_parameter<int>("flush_every_n", 20);
        this->declare_parameter<bool>("enable_logging", true);

        log_dir_ = this->get_parameter("log_dir").as_string();
        csv_name_ = this->get_parameter("csv_name").as_string();
        daily_folder_ = this->get_parameter("daily_folder").as_bool();
        add_suffix_if_exists_ = this->get_parameter("add_suffix_if_exists").as_bool();
        flush_every_n_ = this->get_parameter("flush_every_n").as_int();
        enable_logging_ = this->get_parameter("enable_logging").as_bool();

        // 1. 訂閱你的機器人指令 Topic
        sub_motor_cmd_ = this->create_subscription<kilin_msgs::msg::MotorCmdStamped>(
            "/kilin/motor_cmd_raw", 10,
            std::bind(&IsaacConverter::topic_callback, this, std::placeholders::_1));

        // 2. 發布 Wheel Velocity Command (4個輪子的速度)
        pub_wheel_velocity_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/isaac/wheel_velocity_cmd", 10);

        // 3. 發布 Position Command (8個關節: hip + steering)
        pub_position_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/isaac/position_cmd", 10);

        // Isaac JointState is translated back into the same feedback contract
        // exposed by the real Kilin ROS bridge.
        pub_motor_state_ = this->create_publisher<kilin_msgs::msg::MotorStateStamped>(
            "/motor/state", 10);

        // 4. 訂閱 Isaac Sim 回傳的感測器資料
        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            std::bind(&IsaacConverter::imu_callback, this, std::placeholders::_1));

        sub_joint_state_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/kilin_joint_states", 10,
            std::bind(&IsaacConverter::joint_state_callback, this, std::placeholders::_1));

        // Initialize previous angles (for minimal path)
        prev_steering_["FL"] = 0.0;
        prev_steering_["FR"] = 0.0;
        prev_steering_["RL"] = 0.0;
        prev_steering_["RR"] = 0.0;

        prev_hip_["FL"] = 0.0;
        prev_hip_["FR"] = 0.0;
        prev_hip_["RL"] = 0.0;
        prev_hip_["RR"] = 0.0;

        // ============================================================
        // Setup CSV logging
        // ============================================================
        if (enable_logging_) {
            setupCsvLogging_();
        }

        RCLCPP_INFO(this->get_logger(), "Isaac Converter Node Started with IMU & JointState monitoring.");
        RCLCPP_INFO(this->get_logger(), "Publishing to /isaac/wheel_velocity_cmd (4 wheels - VELOCITY)");
        RCLCPP_INFO(this->get_logger(), "Publishing to /isaac/position_cmd (8 joints: hip + steering - POSITION)");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /imu and /kilin_joint_states");
        RCLCPP_INFO(this->get_logger(), "Publishing simulated feedback on /motor/state");
    }

    ~IsaacConverter() {
        closeCsvFile_();
    }

private:
    // ============================================================
    // CSV Logging Helper Functions
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
        std::string ext = p.extension().string();
        for (int k = 1; k < 10000; ++k) {
            std::ostringstream oss;
            oss << stem << "_" << std::setw(3) << std::setfill('0') << k << ext;
            fs::path cand = dir / oss.str();
            if (!fs::exists(cand)) return cand;
        }
        return dir / (stem + "_overflow" + ext);
    }

    fs::path getKilinWsLogsDir_() {
        fs::path this_file(__FILE__);
        std::error_code ec;
        fs::path p = fs::absolute(this_file, ec);
        if (ec) p = this_file;
        
        fs::path cur = p.parent_path();
        while (!cur.empty()) {
            if (cur.filename() == "kilin_ws") {
                return cur / "logs";
            }
            cur = cur.parent_path();
        }
        return fs::current_path() / "logs";
    }

    void setupCsvLogging_() {
        fs::path base_logs_dir;
        if (log_dir_.empty()) {
            base_logs_dir = getKilinWsLogsDir_();
        } else {
            base_logs_dir = fs::path(log_dir_);
        }

        if (daily_folder_) {
            base_logs_dir /= makeDateFolder_();
        }

        fs::create_directories(base_logs_dir);

        fs::path out_path = base_logs_dir / csv_name_;
        if (add_suffix_if_exists_) {
            out_path = makeUniquePath_(out_path);
        }

        csv_path_ = out_path.string();

        csv_file_.open(csv_path_, std::ios::out);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open CSV file: %s", csv_path_.c_str());
            enable_logging_ = false;
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CSV logging enabled: %s", csv_path_.c_str());
        
        // Write CSV header
        writeCsvHeader_();
        csv_file_.flush();
    }

    void writeCsvHeader_() {
        // Timestamp
        csv_file_ << "seq,time_sec";
        
        // IMU data (orientation, angular_velocity, linear_acceleration)
        csv_file_ << ",imu_orient_x,imu_orient_y,imu_orient_z,imu_orient_w";
        csv_file_ << ",imu_angular_vel_x,imu_angular_vel_y,imu_angular_vel_z";
        csv_file_ << ",imu_linear_acc_x,imu_linear_acc_y,imu_linear_acc_z";
        
        // Joint states (assuming typical joint names for 4-wheel swerve drive)
        // Position and velocity are converted from radians to degrees
        csv_file_ << ",num_joints";
        for (int i = 0; i < 16; ++i) {  // Max expected joints (4 modules * 3 joints)
            csv_file_ << ",joint_" << i << "_name"
                     << ",joint_" << i << "_pos_deg"
                     << ",joint_" << i << "_vel_deg_s"
                     << ",joint_" << i << "_eff";
        }
        
        csv_file_ << "\n";
    }

    void closeCsvFile_() {
        if (csv_closed_) return;
        csv_closed_ = true;

        if (csv_file_.is_open()) {
            try { csv_file_.flush(); } catch (...) {}
            try { csv_file_.close(); } catch (...) {}
            RCLCPP_INFO(this->get_logger(), "Saved CSV: %s", csv_path_.c_str());
        }
    }

    void logToCsv_() {
        if (!enable_logging_ || !csv_file_.is_open()) return;
        if (!imu_received_ || !joint_state_received_) return;  // Wait for both

        // Use IMU timestamp as the main timestamp
        csv_file_ << log_seq_++ << ","
                  << std::fixed << std::setprecision(9) << toTimeSec_(last_imu_sec_, last_imu_nsec_);

        // IMU data
        csv_file_ << "," << last_imu_orient_x_ << "," << last_imu_orient_y_ 
                  << "," << last_imu_orient_z_ << "," << last_imu_orient_w_;
        csv_file_ << "," << last_imu_ang_vel_x_ << "," << last_imu_ang_vel_y_ 
                  << "," << last_imu_ang_vel_z_;
        csv_file_ << "," << last_imu_lin_acc_x_ << "," << last_imu_lin_acc_y_ 
                  << "," << last_imu_lin_acc_z_;

        // Joint states
        csv_file_ << "," << last_joint_names_.size();
        for (size_t i = 0; i < 16; ++i) {
            if (i < last_joint_names_.size()) {
                // Convert position from radians to degrees
                double pos_deg = last_joint_positions_[i] * 180.0 / M_PI;
                double vel_deg = last_joint_velocities_.size() > i ? last_joint_velocities_[i] * 180.0 / M_PI : 0.0;
                double eff = last_joint_efforts_.size() > i ? last_joint_efforts_[i] : 0.0;
                csv_file_ << "," << last_joint_names_[i]
                         << "," << pos_deg
                         << "," << vel_deg
                         << "," << eff;
            } else {
                csv_file_ << ",,,,";  // Empty fields for unused joints
            }
        }

        csv_file_ << "\n";

        log_count_++;
        if (flush_every_n_ <= 0 || (log_count_ % flush_every_n_ == 0)) {
            csv_file_.flush();
        }
    }

    // IMU Callback
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Store IMU data for logging
        last_imu_sec_ = msg->header.stamp.sec;
        last_imu_nsec_ = msg->header.stamp.nanosec;
        last_imu_orient_x_ = msg->orientation.x;
        last_imu_orient_y_ = msg->orientation.y;
        last_imu_orient_z_ = msg->orientation.z;
        last_imu_orient_w_ = msg->orientation.w;
        last_imu_ang_vel_x_ = msg->angular_velocity.x;
        last_imu_ang_vel_y_ = msg->angular_velocity.y;
        last_imu_ang_vel_z_ = msg->angular_velocity.z;
        last_imu_lin_acc_x_ = msg->linear_acceleration.x;
        last_imu_lin_acc_y_ = msg->linear_acceleration.y;
        last_imu_lin_acc_z_ = msg->linear_acceleration.z;
        imu_received_ = true;

        // Log to CSV
        logToCsv_();

        // 使用 THROTTLE 每 1000 毫秒 (1秒) 印一次 Log，避免洗版
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "IMU Orientation: [x: %.2f, y: %.2f, z: %.2f, w: %.2f]",
            msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    }

    // JointState Callback
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Store joint state data for logging
        last_joint_names_ = msg->name;
        last_joint_positions_ = msg->position;
        last_joint_velocities_ = msg->velocity;
        last_joint_efforts_ = msg->effort;
        joint_state_received_ = true;

        publish_motor_state_(*msg);

        if (!msg->position.empty()) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "JointState Received: %zu joints. First joint [%s] pos: %.2f",
                msg->name.size(), msg->name[0].c_str(), msg->position[0]);
        }
    }

    void publish_motor_state_(const sensor_msgs::msg::JointState& joint_state) {
        const auto position_for = [&joint_state](const std::string& name, double& value) {
            const auto found = std::find(joint_state.name.begin(), joint_state.name.end(), name);
            if (found == joint_state.name.end()) {
                return false;
            }
            const auto index = static_cast<std::size_t>(std::distance(joint_state.name.begin(), found));
            if (index >= joint_state.position.size()) {
                return false;
            }
            value = joint_state.position[index];
            return std::isfinite(value);
        };

        double fl_hip = 0.0;
        double fr_hip = 0.0;
        double rl_hip = 0.0;
        double rr_hip = 0.0;
        if (!position_for("FL_hip", fl_hip) || !position_for("FR_hip", fr_hip) ||
            !position_for("RL_hip", rl_hip) || !position_for("RR_hip", rr_hip)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "JointState lacks one or more required hip names; not publishing /motor/state");
            return;
        }

        kilin_msgs::msg::MotorStateStamped state;
        state.header.seq = static_cast<int32_t>(++motor_state_seq_);
        const auto now_ns = this->get_clock()->now().nanoseconds();
        state.header.time.sec = static_cast<int32_t>(now_ns / 1000000000LL);
        state.header.time.nanosec = static_cast<uint32_t>(now_ns % 1000000000LL);
        state.header.frame_id = "isaac_sim";
        // The shared module convention is A/B/C/D = FL/FR/RL/RR.
        state.module_a.hip.position = fl_hip;
        state.module_b.hip.position = fr_hip;
        state.module_c.hip.position = rl_hip;
        state.module_d.hip.position = rr_hip;
        // position_diff is measured by the physical low-level motor system for
        // backlash monitoring. Isaac has no corresponding model, so its ideal
        // articulation feedback explicitly reports zero rather than inventing
        // a command-tracking error with different semantics.
        state.module_a.hip.position_diff = 0.0;
        state.module_b.hip.position_diff = 0.0;
        state.module_c.hip.position_diff = 0.0;
        state.module_d.hip.position_diff = 0.0;
        state.module_a.steering.position_diff = 0.0;
        state.module_b.steering.position_diff = 0.0;
        state.module_c.steering.position_diff = 0.0;
        state.module_d.steering.position_diff = 0.0;
        state.module_a.hub.position_diff = 0.0;
        state.module_b.hub.position_diff = 0.0;
        state.module_c.hub.position_diff = 0.0;
        state.module_d.hub.position_diff = 0.0;
        pub_motor_state_->publish(state);
    }

    // Wrap angle to [-π, π]
    double wrap_to_pi(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Convert steering angle with minimal path logic
    // Input: raw angle (0~2π from kilin_cmd_converter)
    // Output: angle in [-π, π] taking shortest path from previous position
    double steering_convert(double raw_angle, const std::string& module) {
        // Convert 0~2π to -π~π
        double angle = wrap_to_pi(raw_angle);
        
        // Get previous angle for this module
        double prev = prev_steering_[module];
        
        // Calculate difference (shortest path)
        double diff = wrap_to_pi(angle - prev);
        
        // Calculate new target using minimal rotation
        double target = wrap_to_pi(prev + diff);
        
        // Update previous angle
        prev_steering_[module] = target;
        
        return target;
    }

    // Convert hip angle with minimal path logic (keep original range)
    // Input: raw angle from kilin_cmd_converter
    // Output: angle taking shortest path from previous position
    double hip_convert(double raw_angle, const std::string& module) {
        // Get previous angle for this module
        double prev = prev_hip_[module];
        
        // Calculate difference (shortest path) - assuming continuous rotation
        double diff = wrap_to_pi(raw_angle - prev);
        
        // Calculate new target using minimal rotation
        double target = prev + diff;
        
        // Update previous angle
        prev_hip_[module] = target;
        
        return target;
    }

    void topic_callback(const kilin_msgs::msg::MotorCmdStamped::SharedPtr msg) {
        // ============================================================
        // 🟢 發布 Wheel Velocity Command (4 個輪子 - VELOCITY MSG)
        // ============================================================
        auto wheel_vel_msg = std_msgs::msg::Float64MultiArray();
        wheel_vel_msg.data = {
            msg->module_a.hub.velocity * RPM10_TO_RADS,  // FL_wheel
            msg->module_b.hub.velocity * RPM10_TO_RADS,  // FR_wheel
            msg->module_c.hub.velocity * RPM10_TO_RADS,  // RL_wheel
            msg->module_d.hub.velocity * RPM10_TO_RADS   // RR_wheel
        };
        pub_wheel_velocity_->publish(wheel_vel_msg);

        // ============================================================
        // 🔵 發布 Position Command (8 個關節 - POSITION MSG)
        // Hip: minimal path (keep original range)
        // Steering: converted to [-π, π] with minimal path
        // ============================================================
        auto position_msg = std_msgs::msg::Float64MultiArray();
        position_msg.data = {
            hip_convert(msg->module_a.hip.position, "FL"),           // FL_hip
            steering_convert(msg->module_a.steering.position, "FL"), // FL_steering
            hip_convert(msg->module_b.hip.position, "FR"),           // FR_hip 
            steering_convert(msg->module_b.steering.position, "FR"), // FR_steering
            hip_convert(msg->module_c.hip.position, "RL"),           // RL_hip
            steering_convert(msg->module_c.steering.position, "RL"), // RL_steering
            hip_convert(msg->module_d.hip.position, "RR"),           // RR_hip 
            steering_convert(msg->module_d.steering.position, "RR")  // RR_steering
        };
        pub_position_cmd_->publish(position_msg);
    }

    rclcpp::Subscription<kilin_msgs::msg::MotorCmdStamped>::SharedPtr sub_motor_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_wheel_velocity_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_position_cmd_;
    rclcpp::Publisher<kilin_msgs::msg::MotorStateStamped>::SharedPtr pub_motor_state_;
    
    // Sensor subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state_;
    
    // Track previous angles for minimal path calculation
    std::map<std::string, double> prev_steering_;
    std::map<std::string, double> prev_hip_;

    // ============================================================
    // CSV Logging Members
    // ============================================================
    std::string log_dir_;
    std::string csv_name_;
    bool daily_folder_ = true;
    bool add_suffix_if_exists_ = true;
    int flush_every_n_ = 20;
    bool enable_logging_ = true;

    std::string csv_path_;
    std::ofstream csv_file_;
    bool csv_closed_ = false;
    uint64_t log_count_ = 0;
    uint64_t log_seq_ = 0;
    uint64_t motor_state_seq_ = 0;

    // IMU data cache
    bool imu_received_ = false;
    int32_t last_imu_sec_ = 0;
    uint32_t last_imu_nsec_ = 0;
    double last_imu_orient_x_ = 0.0;
    double last_imu_orient_y_ = 0.0;
    double last_imu_orient_z_ = 0.0;
    double last_imu_orient_w_ = 0.0;
    double last_imu_ang_vel_x_ = 0.0;
    double last_imu_ang_vel_y_ = 0.0;
    double last_imu_ang_vel_z_ = 0.0;
    double last_imu_lin_acc_x_ = 0.0;
    double last_imu_lin_acc_y_ = 0.0;
    double last_imu_lin_acc_z_ = 0.0;

    // Joint state data cache
    bool joint_state_received_ = false;
    std::vector<std::string> last_joint_names_;
    std::vector<double> last_joint_positions_;
    std::vector<double> last_joint_velocities_;
    std::vector<double> last_joint_efforts_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IsaacConverter>());
    rclcpp::shutdown();
    return 0;
}
