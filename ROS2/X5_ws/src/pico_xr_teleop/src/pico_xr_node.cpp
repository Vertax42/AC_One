#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <atomic>
#include <array>
#include <string>

// Include common_tools logging system
#include <utils_common.h>
#include <utils_logger.hpp>
#include <utils_colorprint.hpp>
#include <log4z.h>

using json = nlohmann::json;

// // 自定义立即输出的日志宏
// #define LOGI(msg) do { \
//     std::cout << "[INFO] " << msg << std::endl; \
//     std::cout.flush(); \
// } while(0)

// #define LOGFMTI(fmt, ...) do { \
//     char buffer[1024]; \
//     snprintf(buffer, sizeof(buffer), fmt, ##__VA_ARGS__); \
//     std::cout << "[INFO] " << buffer << std::endl; \
//     std::cout.flush(); \
// } while(0)

// #define LOGFMTW_IMMEDIATE(fmt, ...) do { \
//     char buffer[1024]; \
//     snprintf(buffer, sizeof(buffer), fmt, ##__VA_ARGS__); \
//     std::cout << "[WARN] " << buffer << std::endl; \
//     std::cout.flush(); \
// } while(0)

class PicoXRNode : public rclcpp::Node
{
public:
    PicoXRNode() : Node("pico_xr_node")
    {
        // Connection status flags
        server_connected_ = false;
        device_connected_ = false;
        last_data_time_ = this->get_clock()->now();
        should_stop_ = false;
        // Publishers for left controller
        left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pico_xr/left_controller/pose", 10);
        left_buttons_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
            "/pico_xr/left_controller/joy", 10);
        
        // Publishers for right controller
        right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pico_xr/right_controller/pose", 10);
        right_buttons_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
            "/pico_xr/right_controller/joy", 10);
        
        // Publisher for headset
        headset_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/pico_xr/headset/pose", 10);
        
        // Initialize controller data
        left_controller_pose_.fill(0.0);
        right_controller_pose_.fill(0.0);
        headset_pose_.fill(0.0);
        left_trigger_ = 0.0;
        right_trigger_ = 0.0;
        left_grip_ = 0.0;
        right_grip_ = 0.0;
        
        // Initialize PICO XR SDK
        int ret = PXREAInit(this, OnPXREAClientCallback, PXREAFullMask);
        if (ret != 0) {
            LOGFMTE("Failed to initialize PICO XR SDK: %d", ret);
            return;
        }
        
        LOGFMTI("PICO XR SDK initialized successfully");
        
        // Create timer to publish data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),  // 100Hz
            std::bind(&PicoXRNode::publish_data, this));
            
        // Start background thread for status monitoring
        status_thread_ = std::thread(&PicoXRNode::status_monitor_loop, this);
    }
    
    ~PicoXRNode()
    {
        // Stop the status monitoring thread
        should_stop_ = true;
        if (status_thread_.joinable()) {
            status_thread_.join();
        }
        
        PXREADeinit();
        // PICO XR SDK deinitialized
        LOGI("PICO XR SDK deinitialized");
    }

private:
    static void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
    {
        PicoXRNode* node = static_cast<PicoXRNode*>(context);
        node->handle_callback(type, status, userData);
    }
    
    void handle_callback(PXREAClientCallbackType type, int status, void* userData)
    {
        switch (type)
        {
        case PXREAServerConnect:
            server_connected_ = true;
            // PICO XR server connected
            LOGI("PICO XR server connected");
            break;
        case PXREAServerDisconnect:
            server_connected_ = false;
            device_connected_ = false;
            // PICO XR server disconnected
            LOGFMTW("PICO XR server disconnected");
            break;
        case PXREADeviceFind:
            // PICO XR device found
            LOGFMTI("PICO XR device found: %s", (const char*)userData);
            break;
        case PXREADeviceMissing:
            device_connected_ = false;
            // PICO XR device missing
            LOGFMTW("PICO XR device missing: %s", (const char*)userData);
            break;
        case PXREADeviceConnect:
            device_connected_ = (status == 0);
            // PICO XR device connected
            LOGFMTI("PICO XR device connected: %s (status: %d)", 
                          (const char*)userData, status);
            break;
        case PXREADeviceStateJson:
            last_data_time_ = this->get_clock()->now();
            // If we're receiving data, we can assume device is connected
            if (!device_connected_) {
                device_connected_ = true;
                // PICO XR device connected and receiving data
                LOGI("✅ PICO XR device connected and receiving data!");
                // Device connected, status monitor loop will exit
            }
            process_device_state(userData);
            break;
        default:
            break;
        }
    }
    
    void process_device_state(void* userData)
    {
        auto& dsj = *((PXREADevStateJson*)userData);
        
        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                
                // Process left controller
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(left_mutex_);
                        left_controller_pose_ = string_to_pose_array(left["pose"].get<std::string>());
                        left_trigger_ = left["trigger"].get<double>();
                        left_grip_ = left["grip"].get<double>();
                    }
                }
                
                // Process right controller
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(right_mutex_);
                        right_controller_pose_ = string_to_pose_array(right["pose"].get<std::string>());
                        right_trigger_ = right["trigger"].get<double>();
                        right_grip_ = right["grip"].get<double>();
                    }
                }
                
                // Process headset
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headset_mutex_);
                        headset_pose_ = string_to_pose_array(headset["pose"].get<std::string>());
                    }
                }
            }
        } catch (const json::exception& e) {
            LOGFMTE("JSON parsing error: %s", e.what());
        }
    }
    
    std::array<double, 7> string_to_pose_array(const std::string& pose_str)
    {
        std::array<double, 7> pose = {0};
        std::istringstream iss(pose_str);
        std::string token;
        int i = 0;
        
        while (std::getline(iss, token, ',') && i < 7) {
            try {
                pose[i] = std::stod(token);
                i++;
            } catch (const std::exception& e) {
                LOGFMTE("Failed to parse pose element: %s", token.c_str());
            }
        }
        
        return pose;
    }
    
    geometry_msgs::msg::PoseWithCovarianceStamped create_pose_message(const std::array<double, 7>& pose_array)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "pico_xr_frame";
        
        // Position (x, y, z)
        msg.pose.pose.position.x = pose_array[0];
        msg.pose.pose.position.y = pose_array[1];
        msg.pose.pose.position.z = pose_array[2];
        
        // Orientation (quaternion: x, y, z, w)
        msg.pose.pose.orientation.x = pose_array[3];
        msg.pose.pose.orientation.y = pose_array[4];
        msg.pose.pose.orientation.z = pose_array[5];
        msg.pose.pose.orientation.w = pose_array[6];
        
        return msg;
    }
    
    sensor_msgs::msg::Joy create_buttons_message(double trigger, double grip)
    {
        sensor_msgs::msg::Joy msg;
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "pico_xr_frame";
        
        // Add trigger and grip values as axes
        msg.axes.push_back(static_cast<float>(trigger));
        msg.axes.push_back(static_cast<float>(grip));
        
        // Add button states (trigger > 0.5 and grip > 0.5 considered pressed)
        msg.buttons.push_back(trigger > 0.5 ? 1 : 0);  // trigger button
        msg.buttons.push_back(grip > 0.5 ? 1 : 0);     // grip button
        
        return msg;
    }
    
    void check_connection_status()
    {
        auto now = this->get_clock()->now();
        auto time_since_last_data = now - last_data_time_;
        
        // Check if we haven't received data for more than 5 seconds
        if (time_since_last_data.seconds() > 5.0) {
            static auto last_warn_time = std::chrono::steady_clock::now();
            auto current_time = std::chrono::steady_clock::now();
            auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_warn_time);
            
            if (time_diff.count() >= 5) {  // Throttle to every 5 seconds
                // No data received warning
                LOGFMTW("No data received for %.1f seconds", time_since_last_data.seconds());
                last_warn_time = current_time;
            }
        }
        
        // Log connection status
        static auto last_status_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        auto time_diff = std::chrono::duration_cast<std::chrono::seconds>(current_time - last_status_time);
        
        if (time_diff.count() >= 10) {  // Throttle to every 10 seconds
            LOGFMTI("Connection Status - Server: %s, Device: %s, Last data: %.1fs ago",
                          server_connected_ ? "Connected" : "Disconnected",
                          device_connected_ ? "Connected" : "Disconnected",
                          time_since_last_data.seconds());
            last_status_time = current_time;
        }
    }
    
    void status_monitor_loop()
    {
        int waiting_count = 0;
        auto last_connection_check = std::chrono::steady_clock::now();
        auto last_waiting_print = std::chrono::steady_clock::now();
        
        while (!should_stop_) {
            auto now = std::chrono::steady_clock::now();
            
            // Check connection status every 2 seconds
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_connection_check).count() >= 2) {
                check_connection_status();
                last_connection_check = now;
            }
            
            // Print waiting status every 5 seconds
            if (std::chrono::duration_cast<std::chrono::seconds>(now - last_waiting_print).count() >= 5) {
                waiting_count++;
                
                if (!server_connected_ && !device_connected_) {
                    LOGFMTI("⏳ Waiting for PICO XR connection... (attempt %d)", waiting_count);
                    LOGI("   📋 Please ensure:");
                    LOGI("   • PICO XR device is powered on");
                    LOGI("   • Device is connected to the same network");
                    LOGI("   • PICO XR server is running");
                    LOGI("   • No firewall blocking the connection");
                } else if (server_connected_ && !device_connected_) {
                    LOGFMTI("🔗 Server connected, waiting for PICO XR device... (attempt %d)", waiting_count);
                } else if (server_connected_ && device_connected_) {
                    LOGI("✅ PICO XR device connected successfully!");
                    break;  // Exit the loop once connected
                }
                
                last_waiting_print = now;
            }
            
            // Sleep for 100ms to avoid busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    void print_waiting_status()
    {
        // This function is now handled by status_monitor_loop
    }
    
    void publish_data()
    {
        // Publish left controller data
        {
            std::lock_guard<std::mutex> lock(left_mutex_);
            auto left_pose_msg = create_pose_message(left_controller_pose_);
            auto left_buttons_msg = create_buttons_message(left_trigger_, left_grip_);
            
            left_pose_pub_->publish(left_pose_msg);
            left_buttons_pub_->publish(left_buttons_msg);
        }
        
        // Publish right controller data
        {
            std::lock_guard<std::mutex> lock(right_mutex_);
            auto right_pose_msg = create_pose_message(right_controller_pose_);
            auto right_buttons_msg = create_buttons_message(right_trigger_, right_grip_);
            
            right_pose_pub_->publish(right_pose_msg);
            right_buttons_pub_->publish(right_buttons_msg);
        }
        
        // Publish headset data
        {
            std::lock_guard<std::mutex> lock(headset_mutex_);
            auto headset_pose_msg = create_pose_message(headset_pose_);
            headset_pose_pub_->publish(headset_pose_msg);
        }
    }
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr left_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr left_buttons_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr right_buttons_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr headset_pose_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Status monitoring thread
    std::thread status_thread_;
    std::atomic<bool> should_stop_;
    
    // Connection status
    std::atomic<bool> server_connected_;
    std::atomic<bool> device_connected_;
    rclcpp::Time last_data_time_;
    
    // Data storage with mutexes
    std::mutex left_mutex_;
    std::mutex right_mutex_;
    std::mutex headset_mutex_;
    
    std::array<double, 7> left_controller_pose_;
    std::array<double, 7> right_controller_pose_;
    std::array<double, 7> headset_pose_;
    
    std::atomic<double> left_trigger_;
    std::atomic<double> right_trigger_;
    std::atomic<double> left_grip_;
    std::atomic<double> right_grip_;
};

void signalHandler(int signum) {
  rclcpp::shutdown();
}

int main(int argc, char* argv[])
{
    std::signal(SIGHUP, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);
    
    // Initialize log4z system
    zsummer::log4z::ILog4zManager::getRef().start();
    zsummer::log4z::ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_DEBUG);
    
    // 设置日志立即输出，不缓冲
    zsummer::log4z::ILog4zManager::getRef().setLoggerDisplay(LOG4Z_MAIN_LOGGER_ID, true);
    // zsummer::log4z::ILog4zManager::getRef().setLoggerOutFile(LOG4Z_MAIN_LOGGER_ID, false);  // 不输出到文件，只输出到控制台
    zsummer::log4z::ILog4zManager::getRef().setLoggerLimitsize(LOG4Z_MAIN_LOGGER_ID, 0);    // 不限制日志大小
    
        
    // Initialize program info
    printf_program("PICO XR Teleoperation Node");
    common_tools::dump_program_info_log4z("PICO XR Teleoperation Node");
    common_tools::clean_log_files(6);
    LOGI("Initializing PICO XR Node...");

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PicoXRNode>());

    rclcpp::shutdown();
    return 0;
}