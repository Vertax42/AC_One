#ifndef PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_
#define PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <pico_xr_teleop/msg/pos_cmd.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <array>
#include <mutex>
#include <atomic>
#include <vector>

namespace pico_xr_teleop
{

class PicoXRTeleopNode : public rclcpp::Node
{
public:
    /**
     * @brief 构造函数
     */
    PicoXRTeleopNode();

private:
    // Publishers for arm commands - 发送给X5Controller
    rclcpp::Publisher<pico_xr_teleop::msg::PosCmd>::SharedPtr left_arm_cmd_pub_;
    rclcpp::Publisher<pico_xr_teleop::msg::PosCmd>::SharedPtr right_arm_cmd_pub_;
    
    // Subscribers for Pico XR controller data
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_joy_sub_;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables - 参考main.cpp的状态管理
    std::atomic<double> left_grip_{0.0};
    std::atomic<double> right_grip_{0.0};
    std::atomic<double> left_trigger_{0.0};
    std::atomic<double> right_trigger_{0.0};
    
    bool left_is_first_valid_pose_;
    bool right_is_first_valid_pose_;
    bool left_was_grip_pressed_;
    bool right_was_grip_pressed_;
    
    std::array<double, 7> left_controller_pose_{0};
    std::array<double, 7> right_controller_pose_{0};
    std::array<double, 7> left_previous_pose_{0};
    std::array<double, 7> right_previous_pose_{0};
    
    std::vector<double> left_target_pose_{0, 0, 0, 0, 0, 0};
    std::vector<double> right_target_pose_{0, 0, 0, 0, 0, 0};
    
    std::mutex left_mutex_;
    std::mutex right_mutex_;
    
    // Parameters
    double position_scale_;
    double rotation_scale_;
    double control_frequency_;
    
    /**
     * @brief 左控制器pose回调
     * @param msg PoseStamped消息
     */
    void left_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief 右控制器pose回调
     * @param msg PoseStamped消息
     */
    void right_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    
    /**
     * @brief 左控制器joy回调
     * @param msg Joy消息
     */
    void left_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    
    /**
     * @brief 右控制器joy回调
     * @param msg Joy消息
     */
    void right_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    
    /**
     * @brief 主控制循环
     */
    void control_loop();
    
    /**
     * @brief 控制左臂
     */
    void control_left_arm();
    
    /**
     * @brief 控制右臂
     */
    void control_right_arm();
    
    /**
     * @brief 将pose消息转换为数组
     * @param msg PoseStamped消息
     * @return pose数组
     */
    std::array<double, 7> pose_msg_to_array(const geometry_msgs::msg::PoseStamped& msg);
    
    /**
     * @brief 验证控制器pose是否有效
     * @param pose pose数组
     * @return 是否有效
     */
    bool is_valid_controller_pose(const std::array<double, 7>& pose);
    
    /**
     * @brief ARX5左臂坐标转换函数 - 预留待调试
     * @param controller_pose 控制器pose
     * @return ARX5 pose
     */
    std::vector<double> convert_controller_to_arx5_pose_left(const std::array<double, 7>& controller_pose);
    
    /**
     * @brief ARX5右臂坐标转换函数
     * @param controller_pose 控制器pose
     * @return ARX5 pose
     */
    std::vector<double> convert_controller_to_arx5_pose_right(const std::array<double, 7>& controller_pose);
    
    /**
     * @brief 计算左臂相对pose变化
     * @param current_pose 当前pose
     * @param previous_pose 前一个pose
     * @return 相对变化
     */
    std::vector<double> calculate_relative_pose_change_left(
        const std::array<double, 7>& current_pose,
        const std::array<double, 7>& previous_pose);
    
    /**
     * @brief 计算右臂相对pose变化
     * @param current_pose 当前pose
     * @param previous_pose 前一个pose
     * @return 相对变化
     */
    std::vector<double> calculate_relative_pose_change_right(
        const std::array<double, 7>& current_pose,
        const std::array<double, 7>& previous_pose);
    
    /**
     * @brief 发送左臂命令
     */
    void send_left_arm_command();
    
    /**
     * @brief 发送右臂命令
     */
    void send_right_arm_command();
    
    /**
     * @brief 欧拉角转旋转向量
     * @param eigen_euler 欧拉角
     * @return 旋转向量
     */
    std::array<double, 3> euler_to_rotation_vector(const Eigen::Vector3d& eigen_euler);
};

} // namespace pico_xr_teleop

#endif // PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_
