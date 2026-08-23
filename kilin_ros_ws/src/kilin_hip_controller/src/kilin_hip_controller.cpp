#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "httplib.h"

#include <thread>
#include <mutex>
#include <map>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
using json = nlohmann::json;

class HipController : public rclcpp::Node {
public:
    HipController() : Node("kilin_hip_controller") {
        // Parameters
        web_port_ = declare_parameter<int>("web_port", 8080);
        
        // Get resources_dir from parameter (passed by launch file with get_package_share_directory)
        // This is the correct way via ROS
        resources_dir_ = declare_parameter<std::string>("resources_dir", 
            "/home/r14522829/kilin_ws/install/kilin_hip_controller/share/kilin_hip_controller/resources");
        
        RCLCPP_INFO(this->get_logger(), "Resources directory: %s", resources_dir_.c_str());
        
        // Publisher for hip position commands (Float64MultiArray with 4 values: A, B, C, D)
        pub_hip_cmd_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/kilin/hip_cmd_position", 10);
        
        // Initialize hip positions
        hip_positions_[0] = 0.0;  // A (FL)
        hip_positions_[1] = 0.0;  // B (FR)
        hip_positions_[2] = 0.0;  // C (RL)
        hip_positions_[3] = 0.0;  // D (RR)
        
        // Start HTTP server in a separate thread
        http_thread_ = std::thread(&HipController::runHttpServer, this);
        
        RCLCPP_INFO(this->get_logger(), 
                    "Hip Controller started. Web UI available at http://localhost:%d", 
                    web_port_);
    }

    ~HipController() {
        if (http_thread_.joinable()) {
            http_thread_.join();
        }
    }

private:
    // ============================================================
    // HTTP Server
    // ============================================================
    void runHttpServer() {
        httplib::Server svr;
        
        RCLCPP_INFO(this->get_logger(), "Resources directory: %s", resources_dir_.c_str());
        
        // Verify resources directory exists
        if (!fs::exists(resources_dir_)) {
            RCLCPP_ERROR(this->get_logger(), "Resources directory does not exist: %s", resources_dir_.c_str());
        }
        
        // Health check endpoint
        svr.Get("/api/health", [this](const httplib::Request&, httplib::Response& res) {
            res.set_content(R"({"status":"ok"})", "application/json");
        });
        
        // Set hip position endpoint
        svr.Post("/api/hip/set", [this](const httplib::Request& req, httplib::Response& res) {
            try {
                auto data = json::parse(req.body);
                std::string module = data["module"];
                double position = data["position"];
                
                // Map module to index
                int idx = -1;
                if (module == "A") idx = 0;
                else if (module == "B") idx = 1;
                else if (module == "C") idx = 2;
                else if (module == "D") idx = 3;
                
                if (idx < 0) {
                    res.status = 400;
                    res.set_content(R"({"error":"Invalid module"})", "application/json");
                    return;
                }
                
                // Update position
                {
                    std::lock_guard<std::mutex> lock(hip_mutex_);
                    hip_positions_[idx] = position;
                }
                
                // Publish command
                publishHipCommand();
                
                res.set_content(R"({"status":"ok"})", "application/json");
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error in /api/hip/set: %s", e.what());
                res.status = 400;
                res.set_content(R"({"error":"Invalid request"})", "application/json");
            }
        });
        
        // Get current hip positions
        svr.Get("/api/hip/get", [this](const httplib::Request&, httplib::Response& res) {
            std::lock_guard<std::mutex> lock(hip_mutex_);
            json data;
            data["A"] = hip_positions_[0];
            data["B"] = hip_positions_[1];
            data["C"] = hip_positions_[2];
            data["D"] = hip_positions_[3];
            res.set_content(data.dump(), "application/json");
        });
        
        // Serve static files - root path
        svr.Get("/", [this](const httplib::Request&, httplib::Response& res) {
            serveFile(res, "index.html", "text/html");
        });
        
        // Catch-all handler for static files
        svr.Get(R"(.*)", [this](const httplib::Request& req, httplib::Response& res) {
            std::string path = req.path;
            
            // Skip API routes
            if (path.find("/api/") == 0) {
                res.status = 404;
                res.set_content("Not Found", "text/plain");
                return;
            }
            
            // Determine content type
            std::string content_type = "application/octet-stream";
            if (path.find(".html") != std::string::npos) content_type = "text/html";
            else if (path.find(".css") != std::string::npos) content_type = "text/css";
            else if (path.find(".js") != std::string::npos) content_type = "text/javascript";
            else if (path.find(".png") != std::string::npos) content_type = "image/png";
            else if (path.find(".jpg") != std::string::npos) content_type = "image/jpeg";
            else if (path.find(".svg") != std::string::npos) content_type = "image/svg+xml";
            
            // Remove leading slash for filename
            std::string filename = path;
            if (!filename.empty() && filename[0] == '/') {
                filename = filename.substr(1);
            }
            
            serveFile(res, filename, content_type);
        });
        
        RCLCPP_INFO(this->get_logger(), "Starting HTTP server on port %d", web_port_);
        RCLCPP_INFO(this->get_logger(), "Web UI available at http://localhost:%d", web_port_);
        svr.listen("0.0.0.0", web_port_);
    }

    // ============================================================
    // Serve static file
    // ============================================================
    void serveFile(httplib::Response& res, const std::string& filename, const std::string& content_type) {
        fs::path file_path = fs::path(resources_dir_) / filename;
        
        RCLCPP_DEBUG(this->get_logger(), "Attempting to serve: %s (full path: %s)", filename.c_str(), file_path.c_str());
        
        if (!fs::exists(file_path)) {
            RCLCPP_WARN(this->get_logger(), "File not found: %s", file_path.c_str());
            res.status = 404;
            res.set_content("File not found: " + file_path.string(), "text/plain");
            return;
        }
        
        if (!fs::is_regular_file(file_path)) {
            RCLCPP_WARN(this->get_logger(), "Path is not a file: %s", file_path.c_str());
            res.status = 404;
            res.set_content("Not a file", "text/plain");
            return;
        }
        
        std::ifstream file(file_path, std::ios::binary);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Error opening file: %s", file_path.c_str());
            res.status = 500;
            res.set_content("Error reading file", "text/plain");
            return;
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        res.set_content(buffer.str(), content_type);
        RCLCPP_DEBUG(this->get_logger(), "Successfully served: %s (%zu bytes)", filename.c_str(), buffer.str().size());
    }

    // ============================================================
    // Publish hip position command (Float64MultiArray)
    // ============================================================
    void publishHipCommand() {
        auto msg = std_msgs::msg::Float64MultiArray();
        
        std::lock_guard<std::mutex> lock(hip_mutex_);
        msg.data = {
            hip_positions_[0],  // A (FL)
            hip_positions_[1],  // B (FR)
            hip_positions_[2],  // C (RL)
            hip_positions_[3]   // D (RR)
        };
        
        pub_hip_cmd_->publish(msg);
    }

    int web_port_;
    std::string resources_dir_;
    
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_hip_cmd_;
    
    std::array<double, 4> hip_positions_;
    std::mutex hip_mutex_;
    
    std::thread http_thread_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HipController>());
    rclcpp::shutdown();
    return 0;
}

