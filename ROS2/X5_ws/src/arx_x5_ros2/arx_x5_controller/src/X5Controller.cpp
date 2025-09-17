#include "arx_x5_controller/X5Controller.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <csignal>
#include <thread>
#include <chrono>

// using namespace std::chrono_literals;

namespace arx::x5 {
X5Controller::X5Controller() : Node("x5_controller_node") {
  RCLCPP_INFO(this->get_logger(), "Robot arm is initializing...");
  rclcpp::on_shutdown(std::bind(&X5Controller::Cleanup, this));
  std::string arm_control_type = this->declare_parameter("arm_control_type", "normal");
  std::string package_name = "arx_x5_controller";
  std::string package_share_dir = ament_index_cpp::get_package_share_directory(package_name);

  // default 2 for x5_2025
  int end_type = this->declare_parameter("arm_end_type", 2); 
  
  // default 0 for position, 1 for torque
  catch_control_mode_ = static_cast<CatchControlMode>(this->declare_parameter("catch_control_mode", 0)); 
  
  // default arm home position {0,0,0,0,0,0}
  go_home_positions_ = this->declare_parameter<std::vector<double>>("go_home_position", std::vector<double>{0, 0, 0, 0, 0, 0});
  gripper_limits_ = this->declare_parameter<std::vector<double>>("gripper_limits", std::vector<double>{0, 1});
  joint_torque_limits_ = this->declare_parameter<std::vector<double>>("joint_torque_limits", std::vector<double>{0, 0, 0, 0, 0, 0, 0});
  // print joint torque limits
  RCLCPP_INFO(this->get_logger(), "Joint torque limits: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f]", joint_torque_limits_[0], joint_torque_limits_[1], joint_torque_limits_[2], joint_torque_limits_[3], joint_torque_limits_[4], joint_torque_limits_[5], joint_torque_limits_[6]);

  std::string urdf_path;
  if (end_type == 0)
    urdf_path = package_share_dir + "/" + "x5.urdf";
  else if (end_type == 1)
    urdf_path = package_share_dir + "/" + "x5_master.urdf";
  else {
    urdf_path = package_share_dir + "/x5_2025.urdf";
    new_version_ = true;
  }
  interfaces_ptr_ =
      std::make_shared<InterfacesThread>(urdf_path, this->declare_parameter("arm_can_id", "can0"), end_type);
  RCLCPP_INFO(this->get_logger(), "arm_control_type = %s",arm_control_type.c_str());
  interfaces_ptr_->arx_x(150, 600, 10);
  interfaces_ptr_->setHomePositions(go_home_positions_);
  interfaces_ptr_->setArmStatus(current_state_);
  
  if (arm_control_type == "normal") {
    RCLCPP_INFO(this->get_logger(), "Normal mode started");
    // create joint state publisher
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(
      this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);
    // create joint state subscriber
    joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotCmd>(
        this->declare_parameter("arm_sub_topic_name", "arm_cmd"),
        10,
        std::bind(&X5Controller::CmdCallback, this, std::placeholders::_1));
    // timer for publishing joint information
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&X5Controller::PubState, this));
  } else if (arm_control_type == "eef_control") {
    // End-effector control mode
    RCLCPP_INFO(this->get_logger(), "End-effector control mode started");
    // create joint state publisher
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(
      this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);
    // create end-effector command subscriber
    eef_joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
        this->declare_parameter("arm_sub_topic_name", "eef_cmd"),
        10,
        std::bind(&X5Controller::EefCmdCallback, this, std::placeholders::_1));
    // set initial state to END_CONTROL
    SetRobotState(InterfacesThread::state::END_CONTROL);
    // Timer for publishing joint information
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&X5Controller::PubState, this));
  } else if (arm_control_type == "vr_slave") {
    // vr control mode
    RCLCPP_INFO(this->get_logger(), "vr remote control mode started");
    // create vr joint state publisher
    vr_joint_state_publisher_ = this->create_publisher<arm_control::msg::PosCmd>(
      this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);

    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(this->declare_parameter("arm_pub_topic_name", "arm_status") + "_full", 1);
    // create vr joint state subscriber
    vr_joint_state_subscriber_ = this->create_subscription<arm_control::msg::PosCmd>(
        this->declare_parameter("arm_sub_topic_name", "ARX_VR_L"),
        10,
        std::bind(&X5Controller::VrCmdCallback, this, std::placeholders::_1));
    // timer for publishing joint information
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::VrPubState, this));
  } else if (arm_control_type == "remote_master") {
    // G_COMPENSATION mode
    RCLCPP_INFO(this->get_logger(), "remote master mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);
    SetRobotState(InterfacesThread::state::G_COMPENSATION);
    // timer for publishing joint information
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  } else if (arm_control_type == "remote_slave") {
    // default arm control state is POSITION_CONTROL
    RCLCPP_INFO(this->get_logger(), "remote slave mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);
    follow_joint_state_subscriber_ = this->create_subscription<arx5_arm_msg::msg::RobotStatus>(
        this->declare_parameter("arm_sub_topic_name", "followed_arm_topic"),
        10,  // Reduce queue size to 1 for real-time response if needed
        std::bind(&X5Controller::FollowCmdCallback, this, std::placeholders::_1));
    SetRobotState(InterfacesThread::state::POSITION_CONTROL);
    // timer for publishing joint information
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  }
  else if (arm_control_type == "joint_control_v1")
  {
    RCLCPP_INFO(this->get_logger(),"joint control v1 mode started");
    joint_state_publisher_ = this->create_publisher<arx5_arm_msg::msg::RobotStatus>(this->declare_parameter("arm_pub_topic_name", "arm_status"), 10);
    joint_control_subscriber_ = this->create_subscription<arm_control::msg::JointControl>(
        this->declare_parameter("arm_sub_topic_name", "joint_control"),
        10,
        std::bind(&X5Controller::JointControlCallback,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&X5Controller::PubState, this));
  }
  arx_joy_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>("/arx_joy",1,std::bind(&X5Controller::ArxJoyCallback,this,std::placeholders::_1));
  
  // Initialize smooth interpolator
  smooth_interpolator_ = std::make_unique<utils::SmoothJointInterpolator>(
    std::shared_ptr<void>(this, [](void*){}), this->get_logger(), 0.01, 100); // control_dt default 0.01, print_interval default 100
}

void X5Controller::CmdCallback(const arx5_arm_msg::msg::RobotCmd::SharedPtr msg) {
  double end_pos[6] = {
      msg->end_pos[0], msg->end_pos[1], msg->end_pos[2], msg->end_pos[3], msg->end_pos[4], msg->end_pos[5]};

  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(end_pos);

  interfaces_ptr_->setEndPose(transform);

  std::vector<double> joint_positions = {0, 0, 0, 0, 0, 0};

  for (int i = 0; i < 6; i++) {
    joint_positions[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(joint_positions);

  SetRobotState(static_cast<InterfacesThread::state>(msg->mode));

  interfaces_ptr_->setCatch(msg->gripper);
}

void X5Controller::ArxJoyCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
  if (msg->data[0] == 1) {
    SetRobotState(InterfacesThread::state::G_COMPENSATION);
  }
  else if (msg->data[1] == 1) {
    SetRobotState(InterfacesThread::state::GO_HOME);
  }
  else if (msg->data[2] == 1) {
    SetRobotState(InterfacesThread::state::PROTECT);
  }
  else if (msg->data.size() > 3 && msg->data[3] == 1) {
    // Test SmoothGoHome functionality
    RCLCPP_INFO(this->get_logger(), "Testing SmoothGoHome from joystick command");
    SmoothGoHome(3.0, 0.0, utils::easeInOutCubic);
  }
}

void X5Controller::PubState() {
  
  auto message = arx5_arm_msg::msg::RobotStatus();
  message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // create a vector of length 6
  std::array<double, 6> result;

  std::vector<double> xyzrpy = {0, 0, 0, 0, 0, 0};
  xyzrpy = solve::Isometry2Xyzrpy(transform);

  // fill in vectors
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  message.end_pos = result;

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  for (int i = 0; i < 7; i++) {
    if (new_version_ && i == 6) {
      // project [0, -3.4] to [0, 1] for AC One
      message.joint_pos[i] = joint_pos_vector[i] / gripper_coef_;
    } else {
      message.joint_pos[i] = joint_pos_vector[i];
    }
  }

  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  for (int i = 0; i < 7; i++) {
    message.joint_vel[i] = joint_velocities_vector[i];
  }

  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();
  for (int i = 0; i < 7; i++) {
    message.joint_cur[i] = joint_current_vector[i];
  }
  
  // Check joint torque limits (only print warnings when exceeded)
  checkJointTorqueLimits();
  
  // add current state information to the message (using frame_id field for state info)
  message.header.frame_id = getCurrentStateString();
  
  // publish message
  joint_state_publisher_->publish(message);
}

void X5Controller::VrCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg) {
  // RCLCPP_INFO(this->get_logger(), "接收到数据");
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

  interfaces_ptr_->setEndPose(transform);

  SetRobotState(InterfacesThread::state::END_CONTROL);

  double gripper = msg->gripper;
  if(new_version_)
    gripper *= (-3.4 / 5);
  interfaces_ptr_->setCatch(gripper);
}

void X5Controller::VrPubState() {
  // RCLCPP_INFO(this->get_logger(), "Publish data");
  auto message = arm_control::msg::PosCmd();
  // message.header.stamp = this->get_clock()->now();

  Eigen::Isometry3d transform = interfaces_ptr_->getEndPose();

  // extract quaternion and translation
  Eigen::Quaterniond quat(transform.rotation());
  Eigen::Vector3d translation = transform.translation();

  std::vector<double> xyzrpy = solve::Isometry2Xyzrpy(transform);

  // fill in vectors

  message.x = xyzrpy[0];
  message.y = xyzrpy[1];
  message.z = xyzrpy[2];
  message.roll = xyzrpy[3];
  message.pitch = xyzrpy[4];
  message.yaw = xyzrpy[5];
  message.quater_x = quat.x();
  message.quater_y = quat.y();
  message.quater_z = quat.z();
  message.quater_w = quat.w();

  std::vector<double> joint_pos_vector = interfaces_ptr_->getJointPositons();
  std::vector<double> joint_velocities_vector = interfaces_ptr_->getJointVelocities();
  std::vector<double> joint_current_vector = interfaces_ptr_->getJointCurrent();

  message.gripper = joint_pos_vector[6];

  // 发布消息
  vr_joint_state_publisher_->publish(message);

  //==========================================================
  auto msg = arx5_arm_msg::msg::RobotStatus();
  msg.header.stamp = this->get_clock()->now();

  // create a vector of length 6
  std::array<double, 6> result;

  // fill in vectors
  result[0] = xyzrpy[0];
  result[1] = xyzrpy[1];
  result[2] = xyzrpy[2];
  result[3] = xyzrpy[3];
  result[4] = xyzrpy[4];
  result[5] = xyzrpy[5];

  msg.end_pos = result;

  for (int i = 0; i < 7; i++) {
    msg.joint_pos[i] = joint_pos_vector[i];
  }

  for (int i = 0; i < 7; i++) {
    msg.joint_vel[i] = joint_velocities_vector[i];
  }

  for (int i = 0; i < 7; i++) {
    msg.joint_cur[i] = joint_current_vector[i];
  }
  // publish message
  joint_state_publisher_->publish(msg);
}

void X5Controller::FollowCmdCallback(const arx5_arm_msg::msg::RobotStatus::SharedPtr msg) {
  if (current_state_ != InterfacesThread::state::POSITION_CONTROL) {
    RCLCPP_WARN(this->get_logger(), "FollowCmdCallback Disabled: Robot is now in %s mode, skipping joint position control command...", getCurrentStateString().c_str());
    return;
  }
  
  // Check if smooth interpolation is running to avoid conflicts
  if (smooth_interpolator_ && smooth_interpolator_->isInterpolating()) {
    RCLCPP_WARN(this->get_logger(), "FollowCmdCallback Disabled: Smooth interpolation is running, skipping joint position control command...");
    return;
  }
  std::vector<double> target_joint_position(6, 0.0);

  for (int i = 0; i < 6; i++) {
    target_joint_position[i] = msg->joint_pos[i];
  }

  interfaces_ptr_->setJointPositions(target_joint_position);

  if (catch_control_mode_ == CatchControlMode::kPosition) {
    double pos;
    if(new_version_)
      // project [0, 1] to [0, -3.4] for AC One
      pos = msg->joint_pos[6] * gripper_coef_; 
    // pos = msg->joint_pos[6] / (-3.4);
    else
      pos = msg->joint_pos[6] * 5;
    interfaces_ptr_->setCatch(pos);
  }
  else
    interfaces_ptr_->setCatchTorque(msg->joint_cur[6]);
}

void X5Controller::EefCmdCallback(const arm_control::msg::PosCmd::SharedPtr msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received end-effector command");
  
  // Check if robot is in appropriate state for end-effector control
  if (current_state_ != InterfacesThread::state::END_CONTROL) {
    RCLCPP_WARN(this->get_logger(), "EefCmdCallback Disabled: Robot is now in %s mode, skipping end-effector control command...", getCurrentStateString().c_str());
    return;
  }
  
  // Extract position and orientation from message
  double input[6] = {msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw};
  Eigen::Isometry3d transform = solve::Xyzrpy2Isometry(input);

  // Set end-effector pose
  interfaces_ptr_->setEndPose(transform);

  // Handle gripper control
  double gripper = msg->gripper;
  if(gripper < gripper_limits_[0] || gripper > gripper_limits_[1]) {
    gripper = std::clamp(gripper, gripper_limits_[0], gripper_limits_[1]);
    RCLCPP_WARN(this->get_logger(), "Gripper value out of bounds, clamp to %.3f", gripper);
  }

  if(new_version_) {
    // Project [0, 1] to [0, -3.4] for AC One
    gripper *= gripper_coef_;
  }
  interfaces_ptr_->setCatch(gripper);
  
  RCLCPP_DEBUG(this->get_logger(), "End-effector command executed: pos=[%.3f, %.3f, %.3f], ori=[%.3f, %.3f, %.3f], gripper=%.3f",
               msg->x, msg->y, msg->z, msg->roll, msg->pitch, msg->yaw, gripper);
}

void X5Controller::JointControlCallback(const arm_control::msg::JointControl::SharedPtr msg) {
  // Check if smooth interpolation is running to avoid conflicts
  if (smooth_interpolator_ && smooth_interpolator_->isInterpolating()) {
    RCLCPP_DEBUG(this->get_logger(), "JointControlCallback Disabled: Smooth interpolation is running, skipping joint control command...");
    return;
  }
  
  std::vector<double> target_joint_position(6,0.0);
  for (int i = 0; i < 6; i++)
    target_joint_position[i] = msg->joint_pos[i];
  interfaces_ptr_->setJointPositions(target_joint_position);
  SetRobotState(InterfacesThread::state::POSITION_CONTROL);

  interfaces_ptr_->setCatch(msg->joint_pos[6]);
}

void X5Controller::SmoothGoHome(
  double duration,
  double gripper_target,
  std::function<double(double)> interpolation_func
) {
  RCLCPP_INFO(this->get_logger(), "Starting smooth go home...");
  
  // Check if robot is in appropriate state for smooth go home
  if (current_state_ != InterfacesThread::state::POSITION_CONTROL) {
    RCLCPP_WARN(this->get_logger(), "SmoothGoHome: Robot is now in %s mode, switching to POSITION_CONTROL mode first", 
                getCurrentStateString().c_str());
    
    // Switch to POSITION_CONTROL mode
    SetRobotState(InterfacesThread::state::POSITION_CONTROL);
    
    // Set current position as target to minimize sudden movement after state change
    if (interfaces_ptr_) {
      std::vector<double> current_joint_positions = interfaces_ptr_->getJointPositons();
      if (current_joint_positions.size() >= 7) {
        std::vector<double> current_poses(6);
        for (int i = 0; i < 6; ++i) {
          current_poses[i] = current_joint_positions[i];
        }
        double current_gripper_pos = current_joint_positions[6];
        interfaces_ptr_->setJointPositions(current_poses);
        interfaces_ptr_->setCatch(current_gripper_pos);
        RCLCPP_INFO(this->get_logger(), "Current position synchronized to minimize jitter");
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Starting smooth interpolation with gentle acceleration");
  }
  
  if (smooth_interpolator_ && !smooth_interpolator_->isInterpolating()) {
    // use default interpolation function if none provided
    if (!interpolation_func) {
      interpolation_func = utils::easeInOutCubic;
    }
    
    // execute smooth interpolation with configurable parameters
    bool success = smooth_interpolator_->interpolate(
      go_home_positions_, // home position
      duration,  // configurable duration
      gripper_target,  // configurable gripper target position
      interpolation_func  // configurable interpolation function
    );
    
    if (success) {
      RCLCPP_INFO(this->get_logger(), "Smooth go home completed!");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Smooth go home failed!");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Interpolator not initialized or currently interpolating, skipping this go home");
  }
}

void X5Controller::SetRobotState(InterfacesThread::state target_state) {
  // get state name for logging
  std::string state_name = getStateName(target_state);
  
  // Check if state is changing
  if (current_state_ == target_state) {
    RCLCPP_DEBUG(this->get_logger(), "Robot already in %s mode", state_name.c_str());
    return;
  }
  
  // log state transition
  std::string current_state_name = getCurrentStateString();
  RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s", 
              current_state_name.c_str(), state_name.c_str());
  
  if (interfaces_ptr_) {
    // set arm status to target state
    interfaces_ptr_->setArmStatus(target_state);
    RCLCPP_INFO(this->get_logger(), "Robot state has been changed from %s to %s mode", current_state_name.c_str(), state_name.c_str());
    current_state_ = target_state;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Cannot change robot state to %s - interfaces_ptr_ is null!", 
                 state_name.c_str());
  }
}

InterfacesThread::state X5Controller::getCurrentState() const {
  return current_state_;
}

std::string X5Controller::getCurrentStateString() const {
  return getStateName(current_state_);
}

std::string X5Controller::getStateName(InterfacesThread::state state) const {
  switch (state) {
    case InterfacesThread::state::SOFT:
      return "SOFT";
    case InterfacesThread::state::GO_HOME:
      return "GO_HOME";
    case InterfacesThread::state::PROTECT:
      return "PROTECT";
    case InterfacesThread::state::G_COMPENSATION:
      return "G_COMPENSATION";
    case InterfacesThread::state::END_CONTROL:
      return "END_CONTROL";
    case InterfacesThread::state::POSITION_CONTROL:
      return "POSITION_CONTROL";
    default:
      return "UNKNOWN";
  }
}

bool X5Controller::checkJointTorqueLimits() const {
  if (!interfaces_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot check torque limits - interfaces_ptr_ is null!");
    return false;
  }
  
  // Get current joint currents
  std::vector<double> joint_currents = interfaces_ptr_->getJointCurrent();
  
  if (joint_currents.size() != 7) {
    RCLCPP_ERROR(this->get_logger(), "Invalid joint current data size: %zu, expected 7", joint_currents.size());
    return false;
  }
  
  if (joint_torque_limits_.size() != 7) {
    RCLCPP_ERROR(this->get_logger(), "Invalid torque limits size: %zu, expected 7", joint_torque_limits_.size());
    return false;
  }
  
  bool all_within_limits = true;
  double actual_torque;
  for (int i = 0; i < 7; ++i) {
    // Convert current to actual torque using appropriate torque constant
    if (i < 3) {
      // Joints 0-2: EC-A4310 motors
      actual_torque = std::abs(joint_currents[i]) * torque_constant_EC_A4310_;
    } else {
      // Joints 3-6: DM-J4310 motors (including gripper)
      actual_torque = std::abs(joint_currents[i]) * torque_constant_DM_J4310_;
    }

    // Check torque limit - only print warnings when exceeded
    if (actual_torque > joint_torque_limits_[i]) {
      RCLCPP_WARN(this->get_logger(), "Joint %d torque limit exceeded: %.3f Nm > %.3f Nm (current: %.3f A, abs: %.3f A)", 
                  i, actual_torque, joint_torque_limits_[i], joint_currents[i], std::abs(joint_currents[i]));
      all_within_limits = false;
    }
  }
  
  return all_within_limits;
}

std::vector<bool> X5Controller::getJointTorqueStatus() const {
  std::vector<bool> status(7, false);
  
  if (!interfaces_ptr_) {
    RCLCPP_ERROR(this->get_logger(), "Cannot check torque status - interfaces_ptr_ is null!");
    return status;
  }
  
  // Get current joint currents
  std::vector<double> joint_currents = interfaces_ptr_->getJointCurrent();
  
  if (joint_currents.size() != 7 || joint_torque_limits_.size() != 7) {
    RCLCPP_ERROR(this->get_logger(), "Invalid data size for torque status check");
    return status;
  }
  
  double actual_torque; 
  for (int i = 0; i < 7; ++i) {
    // Convert current to actual torque using appropriate torque constant
    if (i < 3) {
      // Joints 0-2: EC-A4310 motors
      actual_torque = std::abs(joint_currents[i]) * torque_constant_EC_A4310_;
    } else {
      // Joints 3-6: DM-J4310 motors (including gripper)
      actual_torque = std::abs(joint_currents[i]) * torque_constant_DM_J4310_;
    }
    
    status[i] = (actual_torque <= joint_torque_limits_[i]);
  }
  return status;
}
}

void signalHandler(int signum) {
  rclcpp::shutdown();
}

int main(int argc, char *argv[]) {
  std::signal(SIGHUP, signalHandler);
  std::signal(SIGTERM, signalHandler);
  std::signal(SIGINT, signalHandler);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arx::x5::X5Controller>());
  rclcpp::shutdown();
  return 0;
}
