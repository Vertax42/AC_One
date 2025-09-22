#include "pico_xr_teleop/pico_xr_node.hpp"

using json = nlohmann::json;

namespace pico_xr_teleop
{

PicoXRNode::PicoXRNode() : Node("pico_xr_node")
{
    // Connection status flags
    server_connected_ = false;
    device_connected_ = false;
    last_data_time_ = this->get_clock()->now();
    should_stop_ = false;
    
    // Publishers for left controller
    left_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pico_xr/left_controller/pose", 10);
    left_buttons_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
        "/pico_xr/left_controller/joy", 10);
    
    // Publishers for right controller
    right_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/pico_xr/right_controller/pose", 10);
    right_buttons_pub_ = this->create_publisher<sensor_msgs::msg::Joy>(
        "/pico_xr/right_controller/joy", 10);
    
    // Publisher for headset
    headset_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pico_xr/headset/pose", 10);
    
    // Initialize controller data
    left_controller_pose_.fill(0.0);
    right_controller_pose_.fill(0.0);
    headset_pose_.fill(0.0);
    left_trigger_ = 0.0;
    right_trigger_ = 0.0;
    left_grip_ = 0.0;
    right_grip_ = 0.0;
    
    // Initialize PICO XR SDK
    int ret = PXREAInit(this, PicoXRNode::OnPXREAClientCallback, PXREAFullMask);
    if (ret != 0) {
        RCLCPP_ERROR(this->get_logger(), "❌ Failed to initialize PICO XR SDK: %d", ret);
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ PICO XR SDK initialized successfully");
    RCLCPP_INFO(this->get_logger(), "🚀 PICO XR Teleoperation Node started successfully");
    RCLCPP_INFO(this->get_logger(), "📡 Publishing topics:");
    RCLCPP_INFO(this->get_logger(), "   • /pico_xr/left_controller/pose");
    RCLCPP_INFO(this->get_logger(), "   • /pico_xr/left_controller/joy");
    RCLCPP_INFO(this->get_logger(), "   • /pico_xr/right_controller/pose");
    RCLCPP_INFO(this->get_logger(), "   • /pico_xr/right_controller/joy");
    RCLCPP_INFO(this->get_logger(), "   • /pico_xr/headset/pose");
    RCLCPP_INFO(this->get_logger(), "⏳ Waiting for PICO XR device connection...");
    
    // Create timer to publish data
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(5),  // 200Hz
        std::bind(&PicoXRNode::publish_data, this));
        
    // Start background thread for status monitoring
    status_monitor_thread_ = std::thread(&PicoXRNode::status_monitor_loop, this);
}

PicoXRNode::~PicoXRNode()
{
    // Stop the status monitoring thread
    should_stop_ = true;
    if (status_monitor_thread_.joinable()) {
        status_monitor_thread_.join();
    }
    
    PXREADeinit();
    // PICO XR SDK deinitialized
    RCLCPP_INFO(this->get_logger(), "🔌 PICO XR SDK deinitialized");
}

void PicoXRNode::OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
    PicoXRNode* node = static_cast<PicoXRNode*>(context);
    node->handle_callback(type, status, userData);
}

void PicoXRNode::handle_callback(PXREAClientCallbackType type, int status, void* userData)
{
    (void)status; // Suppress unused parameter warning
    
    switch (type)
    {
    case PXREAServerConnect:
        server_connected_ = true;
        // PICO XR server connected
        RCLCPP_INFO(this->get_logger(), "🔗 PICO XR server connected successfully");
        break;
    case PXREAServerDisconnect:
        server_connected_ = false;
        // PICO XR server disconnected
        RCLCPP_WARN(this->get_logger(), "⚠️ PICO XR server disconnected");
        break;
    case PXREADeviceConnect:
        device_connected_ = true;
        // PICO XR device connected
        RCLCPP_INFO(this->get_logger(), "🎮 PICO XR device connected successfully");
        break;
    case PXREADeviceMissing:
        device_connected_ = false;
        // PICO XR device missing
        RCLCPP_WARN(this->get_logger(), "⚠️ PICO XR device missing");
        break;
    case PXREADeviceStateJson:
        // PICO XR data received
        device_connected_ = true;
        process_device_state(userData);
        break;
    default:
        break;
    }
}

void PicoXRNode::process_device_state(void* userData)
{
    if (!userData) return;
    
    PXREADevStateJson* state = static_cast<PXREADevStateJson*>(userData);
    
    // Update last data time
    last_data_time_ = this->get_clock()->now();
    
    try {
        json data = json::parse(state->stateJson);
        if (data.contains("value")) {
            auto value = json::parse(data["value"].get<std::string>());
            
            // Debug: Print JSON structure occasionally
            static int json_debug_counter = 0;
            if (json_debug_counter++ % 1000 == 0) {
                RCLCPP_INFO(this->get_logger(), "🔍 JSON data received, has Controller: %s, has Head: %s", 
                           value.contains("Controller") ? "✅" : "❌",
                           value.contains("Head") ? "✅" : "❌");
                           
                // Print more detailed structure
                if (value.contains("Controller")) {
                    auto controller = value["Controller"];
                    RCLCPP_INFO(this->get_logger(), "🔍 Controller has left: %s, right: %s", 
                               controller.contains("left") ? "✅" : "❌",
                               controller.contains("right") ? "✅" : "❌");
                    
                    if (controller.contains("left")) {
                        auto left = controller["left"];
                        RCLCPP_INFO(this->get_logger(), "🔍 Left has pose: %s, trigger: %s, grip: %s", 
                                   left.contains("pose") ? "✅" : "❌",
                                   left.contains("trigger") ? "✅" : "❌",
                                   left.contains("grip") ? "✅" : "❌");
                    }
                    
                    if (controller.contains("right")) {
                        auto right = controller["right"];
                        RCLCPP_INFO(this->get_logger(), "🔍 Right has pose: %s, trigger: %s, grip: %s", 
                                   right.contains("pose") ? "✅" : "❌",
                                   right.contains("trigger") ? "✅" : "❌",
                                   right.contains("grip") ? "✅" : "❌");
                    }
                }
            }
            
            // Process left controller data
            if (value["Controller"].contains("left")) {
                auto left = value["Controller"]["left"];
                if (left.contains("pose")) {
                    // Parse pose string (format: "x,y,z,qx,qy,qz,qw")
                    std::string pose_str = left["pose"].get<std::string>();
                    left_controller_pose_ = string_to_pose_array(pose_str);
                    
                    // Debug: Print pose data occasionally
                    static int left_pose_debug_counter = 0;
                    if (left_pose_debug_counter++ % 500 == 0) {
                        RCLCPP_INFO(this->get_logger(), "🔍 Left pose updated: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f,%.3f]", 
                                   left_controller_pose_[0], left_controller_pose_[1], left_controller_pose_[2],
                                   left_controller_pose_[3], left_controller_pose_[4], left_controller_pose_[5], left_controller_pose_[6]);
                    }
                }
                
                if (left.contains("trigger")) {
                    left_trigger_ = left["trigger"].get<double>();
                }
                if (left.contains("grip")) {
                    left_grip_ = left["grip"].get<double>();
                }
                
                // Log button presses
                static double last_left_trigger = 0.0;
                static double last_left_grip = 0.0;
                
                if (std::abs(left_trigger_.load() - last_left_trigger) > 0.1) {
                    RCLCPP_INFO(this->get_logger(), "🎯 Left trigger: %.2f", left_trigger_.load());
                    last_left_trigger = left_trigger_.load();
                }
                
                if (std::abs(left_grip_.load() - last_left_grip) > 0.1) {
                    RCLCPP_INFO(this->get_logger(), "🤏 Left grip: %.2f", left_grip_.load());
                    last_left_grip = left_grip_.load();
                }
            }
            
            // Process right controller data
            if (value["Controller"].contains("right")) {
                auto right = value["Controller"]["right"];
                if (right.contains("pose")) {
                    // Parse pose string (format: "x,y,z,qx,qy,qz,qw")
                    std::string pose_str = right["pose"].get<std::string>();
                    right_controller_pose_ = string_to_pose_array(pose_str);
                }
                
                if (right.contains("trigger")) {
                    right_trigger_ = right["trigger"].get<double>();
                }
                if (right.contains("grip")) {
                    right_grip_ = right["grip"].get<double>();
                }
                
                // Log button presses
                static double last_right_trigger = 0.0;
                static double last_right_grip = 0.0;
                
                if (std::abs(right_trigger_.load() - last_right_trigger) > 0.1) {
                    RCLCPP_INFO(this->get_logger(), "🎯 Right trigger: %.2f", right_trigger_.load());
                    last_right_trigger = right_trigger_.load();
                }
                
                if (std::abs(right_grip_.load() - last_right_grip) > 0.1) {
                    RCLCPP_INFO(this->get_logger(), "🤏 Right grip: %.2f", right_grip_.load());
                    last_right_grip = right_grip_.load();
                }
            }
            
            // Process headset data (note: it's "Head" not "Headset" in JSON)
            if (value.contains("Head")) {
                auto headset = value["Head"];
                if (headset.contains("pose")) {
                    // Parse pose string (format: "x,y,z,qx,qy,qz,qw")
                    std::string pose_str = headset["pose"].get<std::string>();
                    headset_pose_ = string_to_pose_array(pose_str);
                }
            }
        }
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse PICO XR data: %s", e.what());
    }
}

void PicoXRNode::publish_data()
{
    static bool left_data_received = false;
    static bool right_data_received = false;
    static bool headset_data_received = false;
    
    // Check if we have valid left controller data (not all zeros)
    bool left_valid = (left_controller_pose_[0] != 0.0 || left_controller_pose_[1] != 0.0 || left_controller_pose_[2] != 0.0 ||
                      left_controller_pose_[3] != 0.0 || left_controller_pose_[4] != 0.0 || left_controller_pose_[5] != 0.0 || left_controller_pose_[6] != 0.0);
    
    if (left_valid) {
        if (!left_data_received) {
            RCLCPP_INFO(this->get_logger(), "📡 Started publishing left controller data");
            left_data_received = true;
        }
        auto left_pose_msg = create_pose_message(left_controller_pose_, "left_controller");
        left_pose_pub_->publish(left_pose_msg);
        
        auto left_joy_msg = create_buttons_message(left_trigger_.load(), left_grip_.load(), "left_controller");
        left_buttons_pub_->publish(left_joy_msg);
    }
    
    // Check if we have valid right controller data (not all zeros)
    bool right_valid = (right_controller_pose_[0] != 0.0 || right_controller_pose_[1] != 0.0 || right_controller_pose_[2] != 0.0 ||
                       right_controller_pose_[3] != 0.0 || right_controller_pose_[4] != 0.0 || right_controller_pose_[5] != 0.0 || right_controller_pose_[6] != 0.0);
    
    if (right_valid) {
        if (!right_data_received) {
            RCLCPP_INFO(this->get_logger(), "📡 Started publishing right controller data");
            right_data_received = true;
        }
        auto right_pose_msg = create_pose_message(right_controller_pose_, "right_controller");
        right_pose_pub_->publish(right_pose_msg);
        
        auto right_joy_msg = create_buttons_message(right_trigger_.load(), right_grip_.load(), "right_controller");
        right_buttons_pub_->publish(right_joy_msg);
    }
    
    // Check if we have valid headset data (not all zeros)
    bool headset_valid = (headset_pose_[0] != 0.0 || headset_pose_[1] != 0.0 || headset_pose_[2] != 0.0 ||
                         headset_pose_[3] != 0.0 || headset_pose_[4] != 0.0 || headset_pose_[5] != 0.0 || headset_pose_[6] != 0.0);
    
    if (headset_valid) {
        if (!headset_data_received) {
            RCLCPP_INFO(this->get_logger(), "📡 Started publishing headset data");
            headset_data_received = true;
        }
        auto headset_pose_msg = create_pose_message(headset_pose_, "headset");
        headset_pose_pub_->publish(headset_pose_msg);
    }
    
    // Debug: Print data reception status every 5 seconds
    static auto last_debug_time = this->get_clock()->now();
    auto now = this->get_clock()->now();
    if ((now - last_debug_time).seconds() > 5.0) {
        RCLCPP_INFO(this->get_logger(), "🔍 Data status - Left: %s, Right: %s, Headset: %s", 
                   left_valid ? "✅" : "❌",
                   right_valid ? "✅" : "❌", 
                   headset_valid ? "✅" : "❌");
        last_debug_time = now;
    }
}

std::array<double, 7> PicoXRNode::string_to_pose_array(const std::string& poseStr)
{
    std::array<double, 7> result{0};
    std::stringstream ss(poseStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 7) {
        result[i++] = std::stod(value);
    }
    return result;
}

geometry_msgs::msg::PoseStamped PicoXRNode::create_pose_message(const std::array<double, 7>& pose, const std::string& frame_suffix)
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "pico_xr_" + frame_suffix + "_frame";
    
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];
    msg.pose.orientation.x = pose[3];
    msg.pose.orientation.y = pose[4];
    msg.pose.orientation.z = pose[5];
    msg.pose.orientation.w = pose[6];
    
    return msg;
}

sensor_msgs::msg::Joy PicoXRNode::create_buttons_message(double trigger, double grip, const std::string& controller_name)
{
    sensor_msgs::msg::Joy msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "pico_xr_" + controller_name + "_frame";
    
    // Buttons: trigger as button (pressed when > 0.9), grip as button (pressed when > 0.9)
    msg.buttons = {
        static_cast<int>(trigger > 0.9 ? 1 : 0),  // trigger button
        static_cast<int>(grip > 0.9 ? 1 : 0)      // grip button
    };
    
    // Axes: trigger value as axis[0], grip value as axis[1]
    msg.axes = {
        static_cast<float>(trigger),  // trigger axis (0.0 to 1.0)
        static_cast<float>(grip)      // grip axis (0.0 to 1.0)
    };
    
    return msg;
}

void PicoXRNode::status_monitor_loop()
{
    while (!should_stop_) {
        auto now = this->get_clock()->now();
        auto time_since_last_data = now - last_data_time_;
        
        // Check if we haven't received data for more than 5 seconds
        if (time_since_last_data.seconds() > 5.0) {
            if (device_connected_) {
                RCLCPP_WARN(this->get_logger(), "⚠️ No data received for %.1f seconds", time_since_last_data.seconds());
            }
        }
        
        // Log connection status every 10 seconds
        static auto last_status_log = now;
        if ((now - last_status_log).seconds() > 10.0) {
            RCLCPP_INFO(this->get_logger(), "📊 Status - Server: %s, Device: %s", 
                       server_connected_ ? "✅" : "❌",
                       device_connected_ ? "✅" : "❌");
            last_status_log = now;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
}

} // namespace pico_xr_teleop

// Signal handler for graceful shutdown
namespace {
void signalHandler(int signal)
{
    RCLCPP_INFO(rclcpp::get_logger("pico_xr_node"), "Received signal %d, shutting down...", signal);
    rclcpp::shutdown();
}
}

int main(int argc, char* argv[])
{
    std::signal(SIGHUP, signalHandler);
    std::signal(SIGTERM, signalHandler);
    std::signal(SIGINT, signalHandler);
    
    // Initialize log4z system
    zsummer::log4z::ILog4zManager::getRef().start();
    zsummer::log4z::ILog4zManager::getRef().setLoggerLevel(LOG4Z_MAIN_LOGGER_ID, LOG_LEVEL_INFO);
    
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
    rclcpp::spin(std::make_shared<pico_xr_teleop::PicoXRNode>());

    rclcpp::shutdown();
    return 0;
}