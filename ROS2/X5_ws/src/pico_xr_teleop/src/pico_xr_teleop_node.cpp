#include "pico_xr_teleop/pico_xr_teleop_node.hpp"

namespace pico_xr_teleop
{

PicoXRTeleopNode::PicoXRTeleopNode() : Node("pico_xr_teleop_node")
{
    // Publishers for arm commands - 发送给X5Controller
    // X5Controller订阅"eef_cmd"话题，我们为左右臂分别创建不同的话题
    left_arm_cmd_pub_ = this->create_publisher<pico_xr_teleop::msg::PosCmd>("left_arm/eef_cmd", 10);
    right_arm_cmd_pub_ = this->create_publisher<pico_xr_teleop::msg::PosCmd>("right_arm/eef_cmd", 10);
        
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
        position_scale_ = this->declare_parameter("position_scale", 1.0);
        rotation_scale_ = this->declare_parameter("rotation_scale", 1.0);
        
    // Control parameters - 参考main.cpp的控制参数，匹配X5Controller频率
    control_frequency_ = 100.0; // 100Hz, 匹配X5Controller的10ms周期
    
    // Timer for control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency_)),
        std::bind(&PicoXRTeleopNode::control_loop, this));
    
    RCLCPP_INFO(this->get_logger(), "🎮 Pico XR Teleop Node started (ARX5 Mode)");
    RCLCPP_INFO(this->get_logger(), "📡 Publishing to X5Controller: left_arm/eef_cmd, right_arm/eef_cmd");
    RCLCPP_INFO(this->get_logger(), "📡 Subscribing to: /pico_xr/left_controller/pose, /pico_xr/right_controller/pose");
    RCLCPP_INFO(this->get_logger(), "📡 Subscribing to: /pico_xr/left_controller/joy, /pico_xr/right_controller/joy");
        RCLCPP_INFO(this->get_logger(), "🔧 Position scale: %.2f, Rotation scale: %.2f", 
                   position_scale_, rotation_scale_);
    RCLCPP_INFO(this->get_logger(), "⚡ Control frequency: %.1f Hz", control_frequency_);
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
    if (msg->buttons.size() > 0) {
        left_trigger_ = msg->buttons[0] > 0 ? 1.0 : 0.0; // trigger按钮
    }
    if (msg->axes.size() > 1) {
        left_grip_ = msg->axes[1]; // grip轴
    }
}

void PicoXRTeleopNode::right_joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
    if (msg->buttons.size() > 0) {
        right_trigger_ = msg->buttons[0] > 0 ? 1.0 : 0.0; // trigger按钮
    }
    if (msg->axes.size() > 1) {
        right_grip_ = msg->axes[1]; // grip轴
    }
}

// 主控制循环 - 参考main.cpp的控制逻辑
void PicoXRTeleopNode::control_loop()
{
    // Left arm control
    control_left_arm();
    
    // Right arm control  
    control_right_arm();
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
            left_previous_pose_ = current_pose;
            left_is_first_valid_pose_ = false;
            RCLCPP_INFO(this->get_logger(), "🎯 Left controller baseline set");
            left_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        // 计算相对变化 - 参考main.cpp的calculateRelativePoseChangeLeft
        auto relative_change = calculate_relative_pose_change_left(current_pose, left_previous_pose_);
        
        // 应用相对变化到位置 (前3个元素)
        for (int i = 0; i < 3; ++i) {
            left_target_pose_[i] += relative_change[i];
        }
        
        // 使用绝对值作为姿态 (后3个元素)
        for (int i = 3; i < 6; ++i) {
            left_target_pose_[i] = relative_change[i];
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
            right_previous_pose_ = current_pose;
            right_is_first_valid_pose_ = false;
            RCLCPP_INFO(this->get_logger(), "🎯 Right controller baseline set");
            right_was_grip_pressed_ = is_grip_pressed;
            return;
        }
        
        auto relative_change = calculate_relative_pose_change_right(current_pose, right_previous_pose_);
        
        for (int i = 0; i < 3; ++i) {
            right_target_pose_[i] += relative_change[i];
        }
        
        for (int i = 3; i < 6; ++i) {
            right_target_pose_[i] = relative_change[i];
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
    
    // 计算位置差异 (前3个元素)
    for (int i = 0; i < 3; ++i) {
        relative_pose[i] = current[i] - previous[i];
    }
    
    // 使用绝对值作为姿态 (后3个元素)
    for (int i = 3; i < 6; ++i) {
        relative_pose[i] = current[i];
    }
    
    return relative_pose;
}

std::vector<double> PicoXRTeleopNode::calculate_relative_pose_change_right(const std::array<double, 7>& current_pose,
                                                       const std::array<double, 7>& previous_pose)
{
    std::vector<double> relative_pose(6);
    auto current = convert_controller_to_arx5_pose_right(current_pose);
    auto previous = convert_controller_to_arx5_pose_right(previous_pose);
    
    for (int i = 0; i < 3; ++i) {
        relative_pose[i] = current[i] - previous[i];
    }
    
    for (int i = 3; i < 6; ++i) {
        relative_pose[i] = current[i];
    }
    
    return relative_pose;
}

void PicoXRTeleopNode::send_left_arm_command()
{
    auto cmd = pico_xr_teleop::msg::PosCmd();
    
    // 设置绝对位置和姿态 - X5Controller期望的格式
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
    auto cmd = pico_xr_teleop::msg::PosCmd();
    
    // 设置绝对位置和姿态 - X5Controller期望的格式
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

} // namespace pico_xr_teleop

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pico_xr_teleop::PicoXRTeleopNode>());
    rclcpp::shutdown();
    return 0;
}