#ifndef PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_
#define PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <arm_control/msg/pos_cmd.hpp>
#include <arx5_arm_msg/msg/robot_status.hpp>
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
    rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr left_arm_cmd_pub_;
    rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr right_arm_cmd_pub_;
    
    // Subscribers for Pico XR controller data
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr left_joy_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr right_joy_sub_;
    
    // Subscribers for robot arm status
    rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr left_arm_status_sub_;
    rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr right_arm_status_sub_;
    
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
    
    // Current robot arm poses (from robot status)
    std::vector<double> left_current_robot_pose_{0, 0, 0, 0, 0, 0};
    std::vector<double> right_current_robot_pose_{0, 0, 0, 0, 0, 0};
    bool left_robot_status_received_{false};
    bool right_robot_status_received_{false};
    
    std::mutex left_mutex_;
    std::mutex right_mutex_;
    std::mutex left_robot_mutex_;
    std::mutex right_robot_mutex_;
    
    // Parameters
    double position_scale_;
    double rotation_scale_;
    double control_frequency_;
    double pose_safety_threshold_{0.3}; // 0.3弧度 ≈ 17.2度
    double rpy_range_limit_{1.57}; // ±1.57弧度 ≈ ±90度
    double deadband_threshold_{0.005}; // 死区阈值：5mm位置 + 0.005弧度角度
    
    
    // Safety status display control
    int safety_status_counter_{0};
    int safety_status_display_interval_{100}; // 每1秒显示一次 (100/100Hz = 1s)
    
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
     * @brief 左臂状态回调
     * @param msg RobotStatus消息
     */
    void left_arm_status_callback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg);
    
    /**
     * @brief 右臂状态回调
     * @param msg RobotStatus消息
     */
    void right_arm_status_callback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg);
    
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
    
    /**
     * @brief 检查VR控制器姿态与机械臂当前姿态的差异是否安全
     * @param vr_pose VR控制器转换后的ARX5姿态 [x,y,z,roll,pitch,yaw]
     * @param robot_pose 机械臂当前姿态 [x,y,z,roll,pitch,yaw]
     * @return 是否安全 (RPY角度差异都小于阈值)
     */
    bool is_pose_difference_safe(const std::vector<double>& vr_pose, const std::vector<double>& robot_pose);
    
    /**
     * @brief 检查RPY角度是否在安全范围内
     * @param pose 姿态 [x,y,z,roll,pitch,yaw]
     * @return 是否在安全范围内 (RPY角度都在±1.57弧度范围内)
     */
    bool is_rpy_in_safe_range(const std::vector<double>& pose);
    
    /**
     * @brief 检查并显示当前安全状态，提示用户何时可以按下grip按键
     */
    void check_and_display_safety_status();
    
};

} // namespace pico_xr_teleop

#endif // PICO_XR_TELEOP__PICO_XR_TELEOP_NODE_HPP_
