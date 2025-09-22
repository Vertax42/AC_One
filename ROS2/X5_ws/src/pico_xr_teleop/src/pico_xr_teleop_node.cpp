#include "pico_xr_teleop/pico_xr_teleop_node.hpp"

namespace pico_xr_teleop
{

PicoXRTeleopNode::PicoXRTeleopNode() : Node("pico_xr_teleop_node")
{
    // Publishers for arm commands - 发送给X5Controller
    // X5Controller订阅"eef_cmd"话题，我们为左右臂分别创建不同的话题
    left_arm_cmd_pub_ = this->create_publisher<arm_control::msg::PosCmd>("left_eef_cmd", 10);
    right_arm_cmd_pub_ = this->create_publisher<arm_control::msg::PosCmd>("right_eef_cmd", 10);
        
        // Subscribers for Pico XR controller data
        left_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pico_xr/left_controller/pose", 10,
            std::bind(&PicoXRTeleopNode::left_pose_callback, this, std::placeholders::_1));
        
        left_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/pico_xr/left_controller/joy", 10,
            std::bind(&PicoXRTeleopNode::left_joy_callback, this, std::placeholders::_1));
        
        right_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pico_xr/right_controller/pose", 10,
            std::bind(&PicoXRTeleopNode::right_pose_callback, this, std::placeholders::_1));
        
        right_joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/pico_xr/right_controller/joy", 10,
            std::bind(&PicoXRTeleopNode::right_joy_callback, this, std::placeholders::_1));
        
    // Subscribers for robot arm status - 订阅机械臂状态获取当前RPY
    left_arm_status_sub_ = this->create_subscription<arx5_arm_msg::msg::RobotStatus>(
        "arm_slave_l_status", 10,  // 根据v2_joint_control.yaml中的配置
        std::bind(&PicoXRTeleopNode::left_arm_status_callback, this, std::placeholders::_1));
        
    right_arm_status_sub_ = this->create_subscription<arx5_arm_msg::msg::RobotStatus>(
        "arm_slave_r_status", 10,  // 根据v2_joint_control.yaml中的配置
        std::bind(&PicoXRTeleopNode::right_arm_status_callback, this, std::placeholders::_1));
    // Initialize states - 参考main.cpp的状态管理
    left_grip_ = 0.0;
    right_grip_ = 0.0;
    left_trigger_ = 0.0;
    right_trigger_ = 0.0;
    left_is_first_valid_pose_ = true;
    right_is_first_valid_pose_ = true;
    left_was_grip_pressed_ = false;
    right_was_grip_pressed_ = false;
    
    // Scale factors
        position_scale_ = this->declare_parameter("position_scale", 1.2);
        rotation_scale_ = this->declare_parameter("rotation_scale", 1.0);
        
    // Control parameters - 参考main.cpp的控制参数，匹配X5Controller频率
    control_frequency_ = 100.0; // 100Hz, 匹配X5Controller的10ms周期
    
    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
        std::bind(&PicoXRTeleopNode::control_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "🎮 Pico XR Teleop Node started (ARX5 Mode)");
    RCLCPP_INFO(this->get_logger(), "📡 Publishing to X5Controller: left_arm/eef_cmd_test, right_arm/eef_cmd_test");
    RCLCPP_INFO(this->get_logger(), "📡 Subscribing to: /pico_xr/left_controller/pose, /pico_xr/right_controller/pose");
    RCLCPP_INFO(this->get_logger(), "📡 Subscribing to: /pico_xr/left_controller/joy, /pico_xr/right_controller/joy");
    RCLCPP_INFO(this->get_logger(), "📡 Subscribing to: arm_slave_l_status, arm_slave_r_status");
        RCLCPP_INFO(this->get_logger(), "🔧 Position scale: %.2f, Rotation scale: %.2f", 
                   position_scale_, rotation_scale_);
    RCLCPP_INFO(this->get_logger(), "⚡ Control frequency: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), "🛡️ Pose safety threshold: %.3f rad (%.1f°)", 
                pose_safety_threshold_, pose_safety_threshold_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "📐 RPY range limit: ±%.3f rad (±%.1f°)", 
                rpy_range_limit_, rpy_range_limit_ * 180.0 / M_PI);
    RCLCPP_INFO(this->get_logger(), "🎯 Deadband threshold: %.6f (anti-jitter)", deadband_threshold_);
}

void PicoXRTeleopNode::left_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(left_mutex_);
    left_controller_pose_ = pose_msg_to_array(*msg);
}

void PicoXRTeleopNode::right_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(right_mutex_);
    right_controller_pose_ = pose_msg_to_array(*msg);
}

void PicoXRTeleopNode::left_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() > 0) {
        left_trigger_ = msg->axes[0]; // trigger轴值，0.0-1.0范围
    }
    if (msg->axes.size() > 1) {
        left_grip_ = msg->axes[1]; // grip轴
    }
}

void PicoXRTeleopNode::right_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->axes.size() > 0) {
        right_trigger_ = msg->axes[0]; // trigger轴值，0.0-1.0范围
    }
    if (msg->axes.size() > 1) {
        right_grip_ = msg->axes[1]; // grip轴
    }
}

void PicoXRTeleopNode::left_arm_status_callback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(left_robot_mutex_);
    
    // 提取当前机械臂的位置和姿态 (假设end_pos包含[x,y,z,roll,pitch,yaw])
    if (msg->end_pos.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            left_current_robot_pose_[i] = msg->end_pos[i];
        }
        left_robot_status_received_ = true;
    }
}

void PicoXRTeleopNode::right_arm_status_callback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(right_robot_mutex_);
    
    // 提取当前机械臂的位置和姿态
    if (msg->end_pos.size() >= 6) {
        for (int i = 0; i < 6; ++i) {
            right_current_robot_pose_[i] = msg->end_pos[i];
        }
        right_robot_status_received_ = true;
        
        // 调试：偶尔打印机械臂状态
        static int status_debug_counter = 0;
        if (status_debug_counter++ % 500 == 0) { // 每5秒打印一次
            RCLCPP_INFO(this->get_logger(), "🤖 Right arm status: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f] (size=%zu)",
                       right_current_robot_pose_[0], right_current_robot_pose_[1], right_current_robot_pose_[2],
                       right_current_robot_pose_[3], right_current_robot_pose_[4], right_current_robot_pose_[5],
                       msg->end_pos.size());
            
            // 打印原始end_pos数据
            if (msg->end_pos.size() >= 6) {
                RCLCPP_INFO(this->get_logger(), "🤖 Raw end_pos: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                           msg->end_pos[0], msg->end_pos[1], msg->end_pos[2],
                           msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]);
            }
        }
    }
}

// 主控制循环 - 参考main.cpp的控制逻辑
void PicoXRTeleopNode::control_loop()
{
    // Left arm control
    control_left_arm();
    
    // Right arm control  
    control_right_arm();
    
    // 定期显示安全状态提示
    check_and_display_safety_status();
}

void PicoXRTeleopNode::control_left_arm()
{
    double current_grip = left_grip_.load();
    bool is_grip_pressed = current_grip > 0.9; // 参考main.cpp的阈值
    
    // 优化：减少锁的持有时间，直接拷贝数据
    std::array<double, 7> current_pose;
    {
        std::lock_guard<std::mutex> lock(left_mutex_);
        current_pose = left_controller_pose_;
    }
    
    // 参考main.cpp: wasGripPressed && !isGripPressed - 松开grip时重置目标位置
    if (left_was_grip_pressed_ && !is_grip_pressed) {
        // 重置目标位置为当前位置（这里应该从机械臂获取当前位置）
        // 暂时保持目标位置不变
        left_is_first_valid_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "🔘 Left grip released - Resetting target pose");
    }
    
    if (is_grip_pressed) {
        if (!is_valid_controller_pose(current_pose)) {
            left_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        if (left_is_first_valid_pose_) {
            // 🛡️ 安全检查：检查VR控制器姿态与机械臂当前姿态的差异
            std::vector<double> current_robot_pose;
            bool robot_status_available = false;
            {
                std::lock_guard<std::mutex> lock(left_robot_mutex_);
                if (left_robot_status_received_) {
                    current_robot_pose = left_current_robot_pose_;
                    robot_status_available = true;
                }
            }
            
            if (!robot_status_available) {
                RCLCPP_WARN(this->get_logger(), "🚨 Left arm robot status not available, cannot perform safety check");
                left_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 将VR控制器姿态转换为机械臂坐标系
            auto vr_arx5_pose = convert_controller_to_arx5_pose_left(current_pose);
            
            // 检查VR控制器RPY角度是否在安全范围内
            if (!is_rpy_in_safe_range(vr_arx5_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Left arm: VR controller RPY angles out of safe range! Please adjust controller orientation.");
                left_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 检查机械臂当前RPY角度是否在安全范围内
            if (!is_rpy_in_safe_range(current_robot_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Left arm: Robot current RPY angles out of safe range! Cannot start teleop.");
                left_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 检查姿态差异是否安全
            if (!is_pose_difference_safe(vr_arx5_pose, current_robot_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Left arm: VR controller pose too different from robot pose! Please align controller orientation first.");
                left_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 安全检查通过，设置基线
            left_previous_pose_ = current_pose;
            // 初始化目标姿态为当前机械臂姿态
            left_target_pose_ = current_robot_pose;
            left_is_first_valid_pose_ = false;
            RCLCPP_INFO(this->get_logger(), "🎯✅ Left controller baseline set safely");
            left_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        // 计算相对变化 - 参考main.cpp的calculateRelativePoseChangeLeft
        auto relative_change = calculate_relative_pose_change_left(current_pose, left_previous_pose_);
        
        // 抖动抑制：检查变化是否足够大
        double total_change = 0.0;
        for (int i = 0; i < 6; ++i) {
            total_change += std::abs(relative_change[i]);
        }
        
        // 死区处理：如果变化太小，忽略这次更新
        if (total_change < deadband_threshold_) {
            return; // 变化太小，不更新目标位置
        }
        
        // 应用相对变化到位置和姿态 (所有6个元素都使用相对变化)
        for (int i = 0; i < 6; ++i) {
            left_target_pose_[i] += relative_change[i];
            
            // 限制RPY角度在安全范围内
            if (i >= 3) {
                left_target_pose_[i] = std::clamp(left_target_pose_[i], -rpy_range_limit_, rpy_range_limit_);
            }
        }
        
        left_previous_pose_ = current_pose;
        
        // 发送命令
        send_left_arm_command();
    }
    
    left_was_grip_pressed_ = is_grip_pressed;
}

void PicoXRTeleopNode::control_right_arm()
{
    double current_grip = right_grip_.load();
    bool is_grip_pressed = current_grip > 0.9;
    
    std::array<double, 7> current_pose;
    {
        std::lock_guard<std::mutex> lock(right_mutex_);
        current_pose = right_controller_pose_;
    }
    
    if (right_was_grip_pressed_ && !is_grip_pressed) {
        right_is_first_valid_pose_ = true;
        RCLCPP_INFO(this->get_logger(), "🔘 Right grip released - Resetting target pose");
    }
    
    if (is_grip_pressed) {
        if (!is_valid_controller_pose(current_pose)) {
            right_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        if (right_is_first_valid_pose_) {
            // 🛡️ 安全检查：检查VR控制器姿态与机械臂当前姿态的差异
            std::vector<double> current_robot_pose;
            bool robot_status_available = false;
            {
                std::lock_guard<std::mutex> lock(right_robot_mutex_);
                if (right_robot_status_received_) {
                    current_robot_pose = right_current_robot_pose_;
                    robot_status_available = true;
                }
            }
            
            if (!robot_status_available) {
                RCLCPP_WARN(this->get_logger(), "🚨 Right arm robot status not available, cannot perform safety check");
                right_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 将VR控制器姿态转换为机械臂坐标系
            auto vr_arx5_pose = convert_controller_to_arx5_pose_right(current_pose);
            
            // 检查VR控制器RPY角度是否在安全范围内
            if (!is_rpy_in_safe_range(vr_arx5_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Right arm: VR controller RPY angles out of safe range! Please adjust controller orientation.");
                right_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 检查机械臂当前RPY角度是否在安全范围内
            if (!is_rpy_in_safe_range(current_robot_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Right arm: Robot current RPY angles out of safe range! Cannot start teleop.");
                right_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 检查姿态差异是否安全
            if (!is_pose_difference_safe(vr_arx5_pose, current_robot_pose)) {
                RCLCPP_WARN(this->get_logger(), "🚨 Right arm: VR controller pose too different from robot pose! Please align controller orientation first.");
                right_was_grip_pressed_ = is_grip_pressed;
                return;
            }
            
            // 安全检查通过，设置基线
            right_previous_pose_ = current_pose;
            // 初始化目标姿态为当前机械臂姿态
            right_target_pose_ = current_robot_pose;
            right_is_first_valid_pose_ = false;
            RCLCPP_INFO(this->get_logger(), "🎯✅ Right controller baseline set safely");
            right_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        auto relative_change = calculate_relative_pose_change_right(current_pose, right_previous_pose_);
        
        // 抖动抑制：检查变化是否足够大
        double total_change = 0.0;
        for (int i = 0; i < 6; ++i) {
            total_change += std::abs(relative_change[i]);
        }
        
        // 死区处理：如果变化太小，忽略这次更新
        if (total_change < deadband_threshold_) {
            return; // 变化太小，不更新目标位置
        }
        
        // 应用相对变化到位置和姿态 (所有6个元素都使用相对变化)
        for (int i = 0; i < 6; ++i) {
            right_target_pose_[i] += relative_change[i];
            
            // 限制RPY角度在安全范围内
            if (i >= 3) {
                right_target_pose_[i] = std::clamp(right_target_pose_[i], -rpy_range_limit_, rpy_range_limit_);
            }
        }
        
        right_previous_pose_ = current_pose;
        
        send_right_arm_command();
    } else {
        right_is_first_valid_pose_ = true;
    }
    
    right_was_grip_pressed_ = is_grip_pressed;
}

// 工具函数
std::array<double, 7> PicoXRTeleopNode::pose_msg_to_array(const geometry_msgs::msg::PoseStamped& msg)
{
    return {
        msg.pose.position.x,
        msg.pose.position.y, 
        msg.pose.position.z,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    };
}

bool PicoXRTeleopNode::is_valid_controller_pose(const std::array<double, 7>& pose)
{
    return pose[0] != 0.0; // 参考main.cpp的验证逻辑
}

// ARX5左臂坐标转换函数 - 预留待调试
std::vector<double> PicoXRTeleopNode::convert_controller_to_arx5_pose_left(const std::array<double, 7>& controller_pose)
{
    std::vector<double> arx5_pose(6);
    
    // 位置转换 - 优化：直接计算，避免重复乘法
    const double pos_scale = position_scale_;
    arx5_pose[0] = -controller_pose[2] * pos_scale;  // X: 前/后
    arx5_pose[1] = -controller_pose[0] * pos_scale;  // Y: 左/右
    arx5_pose[2] =  controller_pose[1] * pos_scale;  // Z: 上/下
    
    // 姿态转换 - 优化：直接使用四元数分量，避免创建Eigen对象
    const double w = controller_pose[6];
    const double x = controller_pose[3];
    const double y = controller_pose[4];
    const double z = controller_pose[5];
    
    // 预计算常用的平方项，减少重复计算
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;
    
    // Roll (绕X轴旋转) - 优化：减少乘法运算
    const double vr_roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x2 + y2));
    
    // Pitch (绕Y轴旋转) - 优化：直接计算sinp
    const double sinp = 2.0 * (w * y - z * x);
    const double vr_pitch = (std::abs(sinp) >= 1.0) ? 
        M_PI / 2.0 * ((sinp > 0) ? 1.0 : -1.0) : std::asin(sinp);
    
    // Yaw (绕Z轴旋转) - 优化：复用预计算的平方项
    const double vr_yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y2 + z2));
    
    // RPY映射 - 优化：直接赋值，避免中间变量
    arx5_pose[3] = -vr_yaw;   // Roll
    arx5_pose[4] = -vr_roll;  // Pitch
    arx5_pose[5] =  vr_pitch; // Yaw
    
    return arx5_pose;
}

// ARX5右臂坐标转换函数
std::vector<double> PicoXRTeleopNode::convert_controller_to_arx5_pose_right(const std::array<double, 7>& controller_pose)
{
    std::vector<double> arx5_pose(6);
    
    // 位置转换 - 优化：直接计算，避免重复乘法
    const double pos_scale = position_scale_;
    arx5_pose[0] = -controller_pose[2] * pos_scale;  // X: 前/后
    arx5_pose[1] = -controller_pose[0] * pos_scale;  // Y: 左/右
    arx5_pose[2] =  controller_pose[1] * pos_scale;  // Z: 上/下
    
    // 姿态转换 - 优化：直接使用四元数分量，避免创建Eigen对象
    const double w = controller_pose[6];
    const double x = controller_pose[3];
    const double y = controller_pose[4];
    const double z = controller_pose[5];
    
    // 预计算常用的平方项，减少重复计算
    const double x2 = x * x;
    const double y2 = y * y;
    const double z2 = z * z;
    
    // Roll (绕X轴旋转) - 优化：减少乘法运算
    const double vr_roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x2 + y2));
    
    // Pitch (绕Y轴旋转) - 优化：直接计算sinp
    const double sinp = 2.0 * (w * y - z * x);
    const double vr_pitch = (std::abs(sinp) >= 1.0) ? 
        M_PI / 2.0 * ((sinp > 0) ? 1.0 : -1.0) : std::asin(sinp);
    
    // Yaw (绕Z轴旋转) - 优化：复用预计算的平方项
    const double vr_yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y2 + z2));
    
    // RPY映射 - 优化：直接赋值，避免中间变量
    arx5_pose[3] = -vr_yaw;   // Roll
    arx5_pose[4] = -vr_roll;  // Pitch
    arx5_pose[5] =  vr_pitch; // Yaw
    
    return arx5_pose;
}

std::array<double, 3> PicoXRTeleopNode::euler_to_rotation_vector(const Eigen::Vector3d& eigen_euler)
{
    // 参考main.cpp的eulerToRobotRotVectorRight
    std::array<double, 3> euler{eigen_euler.x(), eigen_euler.y(), eigen_euler.z()};
    
    double alpha = euler[0];
    double beta = euler[1]; 
    double gamma = euler[2];
    double ca = std::cos(alpha);
    double cb = std::cos(beta);
    double cg = std::cos(gamma);
    double sa = std::sin(alpha);
    double sb = std::sin(beta);
    double sg = std::sin(gamma);
    double r11 = ca*cb;
    double r12 = ca*sb*sg - sa*cg;
    double r13 = ca*sb*cg + sa*sg;
    double r21 = sa*cb;
    double r22 = sa*sb*sg + ca*cg;
    double r23 = sa*sb*cg - ca*sg;
    double r31 = -sb;
    double r32 = cb*sg;
    double r33 = cb*cg;
    double theta = std::acos((r11 + r22 + r33 - 1.0) * 0.5);
    double sth = std::sin(theta);
    double kx = (r32-r23)/(2*sth);
    double ky = (r13-r31)/(2*sth);
    double kz = (r21-r12)/(2*sth);
    std::array<double, 3> rot_vec{0, 0, 0};
    rot_vec[0] = theta*kx;
    rot_vec[1] = theta*ky;
    rot_vec[2] = theta*kz;
    
    return rot_vec;
}

std::vector<double> PicoXRTeleopNode::calculate_relative_pose_change_left(const std::array<double, 7>& current_pose, 
                                                      const std::array<double, 7>& previous_pose)
{
    std::vector<double> relative_pose(6);
    auto current = convert_controller_to_arx5_pose_left(current_pose);
    auto previous = convert_controller_to_arx5_pose_left(previous_pose);
    
    // 计算位置和姿态的相对变化 (所有6个元素都使用相对变化)
    for (int i = 0; i < 6; ++i) {
        relative_pose[i] = current[i] - previous[i];
        
        // 处理RPY角度的跳变问题 (后3个元素是角度)
        if (i >= 3) {
            // 将角度差异限制在 [-π, π] 范围内，避免跳变
            while (relative_pose[i] > M_PI) relative_pose[i] -= 2 * M_PI;
            while (relative_pose[i] < -M_PI) relative_pose[i] += 2 * M_PI;
        }
    }
    
    return relative_pose;
}

std::vector<double> PicoXRTeleopNode::calculate_relative_pose_change_right(const std::array<double, 7>& current_pose,
                                                       const std::array<double, 7>& previous_pose)
{
    std::vector<double> relative_pose(6);
    auto current = convert_controller_to_arx5_pose_right(current_pose);
    auto previous = convert_controller_to_arx5_pose_right(previous_pose);
    
    // 计算位置和姿态的相对变化 (所有6个元素都使用相对变化)
    for (int i = 0; i < 6; ++i) {
        relative_pose[i] = current[i] - previous[i];
        
        // 处理RPY角度的跳变问题 (后3个元素是角度)
        if (i >= 3) {
            // 将角度差异限制在 [-π, π] 范围内，避免跳变
            while (relative_pose[i] > M_PI) relative_pose[i] -= 2 * M_PI;
            while (relative_pose[i] < -M_PI) relative_pose[i] += 2 * M_PI;
        }
    }
    
    return relative_pose;
}

void PicoXRTeleopNode::send_left_arm_command()
{
    auto cmd = arm_control::msg::PosCmd();
    
    // 直接使用目标位置 - X5Controller期望的格式
    cmd.x = left_target_pose_[0];
    cmd.y = left_target_pose_[1]; 
    cmd.z = left_target_pose_[2];
    cmd.roll = left_target_pose_[3];
    cmd.pitch = left_target_pose_[4];
    cmd.yaw = left_target_pose_[5];
    
    // 夹爪控制 - 参考main.cpp的trigger控制
    double current_trigger = left_trigger_.load();
    cmd.gripper = 1.0 - current_trigger; // 反向映射，类似main.cpp
    
    cmd.time_count = this->get_clock()->now().nanoseconds() / 1000000;
    
    left_arm_cmd_pub_->publish(cmd);
    
    // 日志输出
    static int left_counter = 0;
    if (left_counter++ % 200 == 0) { // 每2秒输出一次 (200/100Hz = 2s) - 优化：减少日志频率
        RCLCPP_INFO(this->get_logger(), "⬅️ Left arm: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f] grip=%.3f",
                   cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw, cmd.gripper);
    }
}

void PicoXRTeleopNode::send_right_arm_command()
{
    auto cmd = arm_control::msg::PosCmd();
    
    // 直接使用目标位置 - X5Controller期望的格式
    cmd.x = right_target_pose_[0];
    cmd.y = right_target_pose_[1];
    cmd.z = right_target_pose_[2];
    cmd.roll = right_target_pose_[3];
    cmd.pitch = right_target_pose_[4];
    cmd.yaw = right_target_pose_[5];
    
    double current_trigger = right_trigger_.load();
    cmd.gripper = 1.0 - current_trigger;
    
    cmd.time_count = this->get_clock()->now().nanoseconds() / 1000000;
    
    right_arm_cmd_pub_->publish(cmd);
    
    static int right_counter = 0;
    if (right_counter++ % 200 == 0) { // 每2秒输出一次 (200/100Hz = 2s) - 优化：减少日志频率
        RCLCPP_INFO(this->get_logger(), "➡️ Right arm: pos[%.3f,%.3f,%.3f] rot[%.3f,%.3f,%.3f] grip=%.3f",
                   cmd.x, cmd.y, cmd.z, cmd.roll, cmd.pitch, cmd.yaw, cmd.gripper);
    }
}

bool PicoXRTeleopNode::is_pose_difference_safe(const std::vector<double>& vr_pose, const std::vector<double>& robot_pose)
{
    if (vr_pose.size() < 6 || robot_pose.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "🚨 Invalid pose size for safety check");
        return false;
    }
    
    // 只检查RPY角度差异 (索引3,4,5)
    for (int i = 3; i < 6; ++i) {
        double angle_diff = vr_pose[i] - robot_pose[i];
        
        // 处理角度跳变，将差异限制在 [-π, π] 范围内
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        // 检查是否超过安全阈值
        if (std::abs(angle_diff) > pose_safety_threshold_) {
            const char* axis_names[] = {"Roll", "Pitch", "Yaw"};
            RCLCPP_WARN(this->get_logger(), 
                "🚨 %s angle difference too large: %.3f rad (%.1f°) > threshold %.3f rad (%.1f°)",
                axis_names[i-3], angle_diff, angle_diff * 180.0 / M_PI,
                pose_safety_threshold_, pose_safety_threshold_ * 180.0 / M_PI);
            return false;
        }
    }
    
    return true;
}

bool PicoXRTeleopNode::is_rpy_in_safe_range(const std::vector<double>& pose)
{
    if (pose.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "🚨 Invalid pose size for RPY range check");
        return false;
    }
    
    // 检查RPY角度是否在 [-1.57, +1.57] 范围内 (索引3,4,5)
    for (int i = 3; i < 6; ++i) {
        if (pose[i] < -rpy_range_limit_ || pose[i] > rpy_range_limit_) {
            const char* axis_names[] = {"Roll", "Pitch", "Yaw"};
            RCLCPP_WARN(this->get_logger(), 
                "🚨 %s angle out of safe range: %.3f rad (%.1f°), must be within [%.3f, %.3f] rad ([%.1f°, %.1f°])",
                axis_names[i-3], pose[i], pose[i] * 180.0 / M_PI,
                -rpy_range_limit_, rpy_range_limit_,
                -rpy_range_limit_ * 180.0 / M_PI, rpy_range_limit_ * 180.0 / M_PI);
            return false;
        }
    }
    
    return true;
}

void PicoXRTeleopNode::check_and_display_safety_status()
{
    safety_status_counter_++;
    
    // 只在指定间隔显示状态，避免日志过多
    if (safety_status_counter_ % safety_status_display_interval_ != 0) {
        return;
    }
    
    // 检查左臂安全状态
    bool left_safe = false;
    bool left_robot_available = false;
    std::vector<double> left_robot_pose;
    std::array<double, 7> left_controller_pose;
    
    {
        std::lock_guard<std::mutex> robot_lock(left_robot_mutex_);
        std::lock_guard<std::mutex> controller_lock(left_mutex_);
        if (left_robot_status_received_) {
            left_robot_pose = left_current_robot_pose_;
            left_robot_available = true;
        }
        left_controller_pose = left_controller_pose_;
    }
    
    if (left_robot_available && is_valid_controller_pose(left_controller_pose)) {
        auto left_vr_pose = convert_controller_to_arx5_pose_left(left_controller_pose);
        left_safe = is_rpy_in_safe_range(left_vr_pose) && 
                   is_rpy_in_safe_range(left_robot_pose) &&
                   is_pose_difference_safe(left_vr_pose, left_robot_pose);
    }
    
    // 检查右臂安全状态
    bool right_safe = false;
    bool right_robot_available = false;
    std::vector<double> right_robot_pose;
    std::array<double, 7> right_controller_pose;
    
    {
        std::lock_guard<std::mutex> robot_lock(right_robot_mutex_);
        std::lock_guard<std::mutex> controller_lock(right_mutex_);
        if (right_robot_status_received_) {
            right_robot_pose = right_current_robot_pose_;
            right_robot_available = true;
        }
        right_controller_pose = right_controller_pose_;
    }
    
    if (right_robot_available && is_valid_controller_pose(right_controller_pose)) {
        auto right_vr_pose = convert_controller_to_arx5_pose_right(right_controller_pose);
        right_safe = is_rpy_in_safe_range(right_vr_pose) && 
                    is_rpy_in_safe_range(right_robot_pose) &&
                    is_pose_difference_safe(right_vr_pose, right_robot_pose);
    }
    
    // 显示安全状态
    std::string left_status = left_robot_available ? 
        (left_safe ? "🟢 SAFE" : "🔴 UNSAFE") : "⚫ NO DATA";
    std::string right_status = right_robot_available ? 
        (right_safe ? "🟢 SAFE" : "🔴 UNSAFE") : "⚫ NO DATA";
    
    RCLCPP_INFO(this->get_logger(), 
        "🛡️ Safety Status - Left: %s | Right: %s | %s", 
        left_status.c_str(), 
        right_status.c_str(),
        (left_safe || right_safe) ? "✅ Ready to grip!" : "⚠️  Align controllers first");
}


} // namespace pico_xr_teleop

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pico_xr_teleop::PicoXRTeleopNode>());
    rclcpp::shutdown();
    return 0;
}