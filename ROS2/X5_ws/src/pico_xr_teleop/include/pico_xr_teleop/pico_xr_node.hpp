#ifndef PICO_XR_TELEOP__PICO_XR_NODE_HPP_
#define PICO_XR_TELEOP__PICO_XR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <PXREARobotSDK.h>
#include <nlohmann/json.hpp>
#include <mutex>
#include <atomic>
#include <array>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <log4z.h>
#include <utils_common.h>

namespace pico_xr_teleop
{

class PicoXRNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    PicoXRNode();

    /**
     * @brief 析构函数
     */
    ~PicoXRNode();

private:
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr left_buttons_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr right_buttons_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr headset_pose_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Status monitoring thread
    std::thread status_monitor_thread_;
    
    // Connection status flags
    std::atomic<bool> server_connected_;
    std::atomic<bool> device_connected_;
    rclcpp::Time last_data_time_;
    std::atomic<bool> should_stop_;
    
    // Controller data
    std::array<double, 7> left_controller_pose_;
    std::array<double, 7> right_controller_pose_;
    std::array<double, 7> headset_pose_;
    
    std::atomic<double> left_trigger_;
    std::atomic<double> right_trigger_;
    std::atomic<double> left_grip_;
    std::atomic<double> right_grip_;
    
    // Mutexes for thread safety
    std::mutex left_mutex_;
    std::mutex right_mutex_;
    std::mutex headset_mutex_;
    
    /**
     * @brief PXREA SDK回调函数
     * @param context 上下文
     * @param type 回调类型
     * @param status 状态
     * @param userData 用户数据
     */
    static void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData);
    
    /**
     * @brief 处理PXREA SDK回调
     * @param context 上下文
     * @param type 回调类型
     * @param status 状态
     * @param userData 用户数据
     */
    void handle_callback(PXREAClientCallbackType type, int status, void* userData);
    
    /**
     * @brief 处理设备状态JSON数据
     * @param userData 用户数据
     */
    void process_device_state(void* userData);
    
    /**
     * @brief 将字符串转换为pose数组
     * @param poseStr pose字符串
     * @return pose数组
     */
    std::array<double, 7> string_to_pose_array(const std::string& poseStr);
    
    /**
     * @brief 创建pose消息
     * @param pose_array pose数组
     * @param frame_suffix 帧后缀
     * @return PoseStamped消息
     */
    geometry_msgs::msg::PoseStamped create_pose_message(
        const std::array<double, 7>& pose_array, 
        const std::string& frame_suffix = "");
    
    /**
     * @brief 创建按钮消息
     * @param trigger trigger值
     * @param grip grip值
     * @param controller_name 控制器名称
     * @return Joy消息
     */
    sensor_msgs::msg::Joy create_buttons_message(
        double trigger, 
        double grip, 
        const std::string& controller_name);
    
    /**
     * @brief 发布数据
     */
    void publish_data();
    
    /**
     * @brief 状态监控循环
     */
    void status_monitor_loop();
};

} // namespace pico_xr_teleop

#endif // PICO_XR_TELEOP__PICO_XR_NODE_HPP_
