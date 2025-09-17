#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include <functional>

#include "arx_x5_src/interfaces/InterfacesThread.hpp"

#include "arx5_arm_msg/msg/robot_cmd.hpp"
#include "arx5_arm_msg/msg/robot_status.hpp"
#include "arm_control/msg/pos_cmd.hpp"
#include "arm_control/msg/joint_control.hpp"
#include <std_msgs/msg/int32_multi_array.hpp>
#include "arx_x5_controller/utils.hpp"

namespace arx::x5 {
class X5Controller : public rclcpp::Node {
public:
  X5Controller();
  void Cleanup() {
    RCLCPP_INFO(this->get_logger(), "Robot is shutting down...");
    SetRobotState(InterfacesThread::state::PROTECT);
    RCLCPP_INFO(this->get_logger(), "Robot is set to PROTECT state, shutting down...");
    interfaces_ptr_.reset();
  }

  void CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg);

  void PubState();

  void EefCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);

  void VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg);

  void VrPubState();

  void FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg);
  void JointControlCallback(const arm_control::msg::JointControl::SharedPtr msg);
  void ArxJoyCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
  void SmoothGoHome(
    double duration = 3.0,
    double gripper_target = 0.0,
    std::function<double(double)> interpolation_func = nullptr
  );
  
  // Robot arm state management functions
  void SetRobotState(InterfacesThread::state target_state);
  
  // State query functions
  InterfacesThread::state getCurrentState() const;
  std::string getCurrentStateString() const;
  
  // Getter for interfaces_ptr_ (for SmoothJointInterpolator)
  std::shared_ptr<InterfacesThread> getInterfacesPtr() const { return interfaces_ptr_; }
  
  // Getter for gripper_coef_ (for SmoothJointInterpolator)
  double getGripperCoef() const { return gripper_coef_; }
  
  // Torque monitoring functions
  bool checkJointTorqueLimits() const;
  std::vector<bool> getJointTorqueStatus() const;
  
private:
  // Helper function to get state name
  std::string getStateName(InterfacesThread::state state) const;
  std::shared_ptr<InterfacesThread> interfaces_ptr_;
  enum class CatchControlMode {
    kPosition,
    kTorque
  } catch_control_mode_;

  // normal & remote from master mode
  rclcpp::Publisher<arx5_arm_msg::msg::RobotStatus>::SharedPtr joint_state_publisher_;
  // eef control mode
  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr eef_state_publisher_;
  // vr mode
  rclcpp::Publisher<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_publisher_;

  // normal mode
  rclcpp::Subscription<arx5_arm_msg::msg::RobotCmd>::SharedPtr joint_state_subscriber_;
  // vr mode
  rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr vr_joint_state_subscriber_;
  // eef control mode
  rclcpp::Subscription<arm_control::msg::PosCmd>::SharedPtr eef_joint_state_subscriber_;
  // remote from master mode
  rclcpp::Subscription<arx5_arm_msg::msg::RobotStatus>::SharedPtr follow_joint_state_subscriber_;
  // joint control
  rclcpp::Subscription<arm_control::msg::JointControl>::SharedPtr joint_control_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr arx_joy_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool new_version_ = true;
  std::vector<double> go_home_positions_ = {0, 0, 0, 0, 0, 0};
  std::vector<double> gripper_limits_ = {0, 1};
  double gripper_coef_ = -3.4; // magic number for gripper pos normalization
  std::vector<double> joint_torque_limits_ = {0, 0, 0, 0, 0, 0, 0}; // joint torque limits in Nm
  std::vector<double> joint_pos_limits_ = {0, 0, 0, 0, 0, 0}; // 6 joint position limits in radians, not in use now
  
  // Torque constants for current to torque conversion
  static constexpr double torque_constant_EC_A4310_ = 1.4 * 1.4; // Nm/A
  static constexpr double torque_constant_DM_J4310_ = 0.424; // Nm/A
  
  // smooth interpolator
  std::unique_ptr<utils::SmoothJointInterpolator> smooth_interpolator_;
  
  // Current robot state tracking
  // enum state
  //  {
  //    SOFT,
  //    GO_HOME,
  //    PROTECT,
  //    G_COMPENSATION,
  //    END_CONTROL,
  //    POSITION_CONTROL
  //  };
  
  InterfacesThread::state current_state_ = InterfacesThread::state::POSITION_CONTROL;
};
} // namespace arx::x5
