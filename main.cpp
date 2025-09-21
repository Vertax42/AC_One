#define _USE_MATH_DEFINES
#include <cmath>
#include <stddef.h>
#include <iostream>
#include <PXREARobotSDK.h>
#include <chrono>
#include <thread>
#include <string>
#include <array>
#include <nlohmann/json.hpp>
#include <sstream>
#include <mutex>
#include <atomic>
#include <iomanip>
#include <Eigen/Geometry>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/robotiq_gripper.h>
#include "dynamixel_sdk.h"  // Dynamixel SDK header
#include <csignal>

using json = nlohmann::json;

// Dynamixel SDK namespace
using namespace dynamixel;

// Dynamixel motor parameters
#define MOTOR_ID 3
#define BAUDRATE 4500000
#define DEVICE_NAME "/dev/ttyUSB0"//"COM3"  // Change this to your port name

// Protocol version
#define PROTOCOL_VERSION 2.0

// Control table address
#define ADDR_LED_RED 65
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132

// Dynamixel motor IDs
#define YAW_MOTOR_ID 3
#define PITCH_MOTOR_ID 1

// Dynamixel position constants
#define YAW_CENTER 1521
#define PITCH_CENTER 2753
#define DYNAMIXEL_DEGREE_PER_UNIT 0.0879

// Global variables
std::atomic<bool> running{true};
std::atomic<bool> dynamixel_running{true};
std::mutex dynamixel_mutex;
std::mutex coutMutex;
PortHandler* portHandler = nullptr;
PacketHandler* packetHandler = nullptr;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received.\n";
    
    running = false;
    dynamixel_running = false;
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::exit(signum);
}

int32_t mapYawToDynamixelPosition(double yaw) {
    // Map yaw from [-90, 90] to Dynamixel position
    int32_t position = YAW_CENTER + static_cast<int32_t>(yaw / DYNAMIXEL_DEGREE_PER_UNIT);
    std::cout << "Yaw Dynamixel position: " << position << std::endl;
    return position;
}

// Function to map pitch angle to Dynamixel position
int32_t mapPitchToDynamixelPosition(double pitch) {
    // Map pitch from [-50, 50] to Dynamixel position
    
    if (pitch > 50) pitch = 50;
    if (pitch < -50) pitch = -50;
    
    int32_t position = PITCH_CENTER - static_cast<int32_t>(pitch / DYNAMIXEL_DEGREE_PER_UNIT);
    std::cout << "Pitch Dynamixel position: " << position << std::endl;
    return position;
}

std::vector<double> degreesToRadians(const std::vector<double>& degrees);
std::array<double, 3> quaternionToEuler(double qx, double qy, double qz, double qw);
std::vector<double> convertControllerToUR5PoseLeft(const std::array<double, 7>& controllerPose);
std::vector<double> convertControllerToUR5PoseRight(const std::array<double, 7>& controllerPose);
std::vector<double> calculateRelativePoseChangeLeft(const std::array<double, 7>& currentPose, 
                                              const std::array<double, 7>& previousPose);
std::vector<double> calculateRelativePoseChangeRight(const std::array<double, 7>& currentPose, 
                                              const std::array<double, 7>& previousPose);
bool isValidControllerPoseLeft(const std::array<double, 7>& pose);
bool isValidControllerPoseRight(const std::array<double, 7>& pose);
void leftUR5Control();
void rightUR5Control();
void leftConnectionMonitor();
void dynamixelControl();

std::array<double, 7> LeftControllerPose{0};
std::array<double, 7> RightControllerPose{0};
std::array<double, 7> HeadsetPose{0};
std::mutex leftPoseMutex;
std::mutex rightPoseMutex;
std::mutex headsetPoseMutex;

// UR5 robot IP addresses
const std::string LEFT_ROBOT_IP = "192.168.50.55"; 
const std::string RIGHT_ROBOT_IP = "192.168.50.195";

constexpr double DEG2RAD = M_PI / 180.0;
const double SERVO_TIME = 0.017;        // 17ms (60Hz)
const double LOOKAHEAD_TIME = 0.1;      // 100ms look ahead
const double SERVO_GAIN = 300;          // Servo gain
const double MAX_VELOCITY = 0.5;        // 0.5 m/s
const double MAX_ACCELERATION = 1.0;    // 1.0 m/s^2

std::atomic<double> LeftTrigger{0.0};
std::atomic<double> RightTrigger{0.0};
std::atomic<double> LeftGrip{0.0};
std::atomic<double> RightGrip{0.0};

const float GRIPPER_FORCE = 0.5f;
const float GRIPPER_SPEED = 1.0f;

std::atomic<bool> leftConnectionOK{true};
std::mutex debugMutex;

const std::vector<double> LEFT_INITIAL_JOINT_DEG = {165.26, -47.50, 118.93, -38.96, 87.51, 149.56};
const std::vector<double> RIGHT_INITIAL_JOINT_DEG = {193.53, -164.17, -114.02, 58.01, 101.87, -138.40};

const std::vector<double> LEFT_INITIAL_JOINT = degreesToRadians(LEFT_INITIAL_JOINT_DEG);
const std::vector<double> RIGHT_INITIAL_JOINT = degreesToRadians(RIGHT_INITIAL_JOINT_DEG);

void dynamixelControl() {
    try {
        {
            std::lock_guard<std::mutex> lock(coutMutex);
            std::cout << "Initializing Dynamixel motor..." << std::endl;
        }

        portHandler = PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        
        if (!portHandler->openPort()) {
            std::cerr << "Failed to open the port!" << std::endl;
            return;
        }
        
        if (!portHandler->setBaudRate(BAUDRATE)) {
            std::cerr << "Failed to change the baudrate!" << std::endl;
            return;
        }

        // Enable Dynamixel Torque
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // Enable torque for yaw motor (ID 3)
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, YAW_MOTOR_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Failed to enable torque for yaw motor!" << std::endl;
            return;
        }
        // Enable torque for pitch motor (ID 1)
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, PITCH_MOTOR_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Failed to enable torque for pitch motor!" << std::endl;
            return;
        }

        // Turn on LED
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, MOTOR_ID, ADDR_LED_RED, 1, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "Failed to set red LED!" << std::endl;
            return;
        }
    
        std::cout << "Dynamixel motor initialized successfully!" << std::endl;

        // Main control loop
        double lastYaw = 0.0;
        double lastPitch = 0.0;
        //bool shouldUpdatePosition = false;
        bool shouldUpdatePosition = true;
        
        while (dynamixel_running) {
            // Get the current head orientation
            double currentYaw = 0.0;
            double currentPitch = 0.0;
            
            {
                std::lock_guard<std::mutex> lock(headsetPoseMutex);
                // Get Euler angles from quaternion
                Eigen::Quaterniond q(HeadsetPose[6], HeadsetPose[3], HeadsetPose[4], HeadsetPose[5]);
                Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 0, 1);
                currentYaw = euler[2] * 180.0 / M_PI;
                currentPitch = euler[1] * 180.0 / M_PI;
                
                if (currentYaw > 90 && currentYaw < 180) {
                    currentYaw -= 180;
                }
                if (currentYaw<-90) {currentYaw= 180+currentYaw;}
                if (currentPitch<-90) {currentPitch= -currentPitch-180;}
                if (currentPitch>90) {currentPitch= 180-currentPitch;}
            }
            
            if (shouldUpdatePosition && (std::abs(currentYaw - lastYaw) > 0.01 || std::abs(currentPitch - lastPitch) > 0.01)) {
                // Map angles to Dynamixel positions
                int32_t yawPosition = mapYawToDynamixelPosition(currentYaw);
                int32_t pitchPosition = mapPitchToDynamixelPosition(currentPitch);
                
                // Write positions to both motors
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, YAW_MOTOR_ID, ADDR_GOAL_POSITION, yawPosition, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    std::cerr << "Failed to write yaw position!" << std::endl;
                }
                
                dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, PITCH_MOTOR_ID, ADDR_GOAL_POSITION, pitchPosition, &dxl_error);
                if (dxl_comm_result != COMM_SUCCESS) {
                    std::cerr << "Failed to write pitch position!" << std::endl;
                }
                
                lastYaw = currentYaw;
                lastPitch = currentPitch;
                
                std::lock_guard<std::mutex> lock(coutMutex);
                std::cout << "Yaw: " << currentYaw << "° (Position: " << yawPosition 
                          << "), Pitch: " << currentPitch << "° (Position: " << pitchPosition << ")" << std::endl;
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // Disable torque for both motors before closing
        packetHandler->write1ByteTxRx(portHandler, YAW_MOTOR_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, PITCH_MOTOR_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        
        // Cleanup
        {
            std::lock_guard<std::mutex> lock(coutMutex);
            std::cout << "Cleaning up Dynamixel motor..." << std::endl;
        }

        // Turn off LED
        packetHandler->write1ByteTxRx(portHandler, MOTOR_ID, ADDR_LED_RED, 0, &dxl_error);

        // Close port
        portHandler->closePort();

        std::cout << "Dynamixel motor cleanup completed." << std::endl;

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(coutMutex);
        std::cerr << "Dynamixel Error: " << e.what() << std::endl;
    }
}

std::vector<double> degreesToRadians(const std::vector<double>& degrees) {
    std::vector<double> radians;
    radians.reserve(degrees.size());
    for (const double& deg : degrees) {
        radians.push_back(deg * DEG2RAD);
    }
    return radians;
}

std::array<double, 7> stringToPoseArray(const std::string& poseStr) {
    std::array<double, 7> result{0};
    std::stringstream ss(poseStr);
    std::string value;
    int i = 0;
    while (std::getline(ss, value, ',') && i < 7) {
        result[i++] = std::stod(value);
    }
    return result;
}

bool isValidControllerPoseLeft(const std::array<double, 7>& pose) {
    return pose[0] != 0.0;  // Check if x position is not 0
}

bool isValidControllerPoseRight(const std::array<double, 7>& pose) {
    return pose[0] != 0.0;  // Check if x position is not 0
}

std::array<double, 3> quaternionToEuler(double qx, double qy, double qz, double qw) {
    std::array<double, 3> euler;
    
    
    // Transformation: x to z, y to x, z to y
    double transformed_qx = qz; 
    double transformed_qy = qx;
    double transformed_qz = qy; 

    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (qw * transformed_qx + transformed_qy * transformed_qz);
    double cosr_cosp = 1 - 2 * (transformed_qx * transformed_qx + transformed_qy * transformed_qy);
    euler[0] = std::atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2 * (qw * transformed_qy - transformed_qz * transformed_qx);
    if (std::abs(sinp) >= 1)
        euler[1] = std::copysign(M_PI / 2, sinp);
    else
        euler[1] = std::asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (qw * transformed_qz + transformed_qx * transformed_qy);
    double cosy_cosp = 1 - 2 * (transformed_qy * transformed_qy + transformed_qz * transformed_qz);
    euler[2] = std::atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

std::array<double, 3> eulerToRobotRotVectorLeft(const std::array<double, 3>& euler) {

    double rx = euler[1];
    double ry = euler[2];
    double rz = euler[0];
    
    const double cos45 = 0.70710678118;
    const double sin45 = 0.70710678118;
    
    double ry_rotated = ry * cos45 + rz * sin45;
    double rz_rotated = -ry * sin45 + rz * cos45;
    
    rx = rx;
    ry = ry;
    rz = rz;
    
    double cx = std::cos(rx * 0.5);
    double sx = std::sin(rx * 0.5);
    double cy = std::cos(ry * 0.5);
    double sy = std::sin(ry * 0.5);
    double cz = std::cos(rz * 0.5);
    double sz = std::sin(rz * 0.5);
    
    double qw = cx*cy*cz - sx*sy*sz;
    double qx = sx*cy*cz + cx*sy*sz;
    double qy = cx*sy*cz - sx*cy*sz;
    double qz = cx*cy*sz + sx*sy*cz;

    std::array<double, 3> rotVec{0, 0, 0};
    double angle = 2.0 * std::acos(qw);
    
    if (std::abs(angle) > 1e-6) {
        double s = std::sqrt(1 - qw * qw);
        if (std::abs(s) > 1e-6) {
            rotVec[0] = (qx / s) * angle;
            rotVec[1] = (qy / s) * angle;
            rotVec[2] = (qz / s) * angle;
        }
    }
    
    return rotVec;
}

std::array<double, 3> eulerToRobotRotVectorRight(const Eigen::Vector3d& eigenEuler) {
    std::array<double, 3> euler{eigenEuler.x(), eigenEuler.y(), eigenEuler.z()};
    
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
    std::array<double, 3> rotVec{0, 0, 0};
    rotVec[0] = theta*kx;
    rotVec[1] = theta*ky;
    rotVec[2] = theta*kz;
    
    return rotVec;
}

std::vector<double> convertControllerToUR5PoseRight(const std::array<double, 7>& controllerPose) {
    std::vector<double> ur5Pose(6);
    
    double x = controllerPose[2];
    double y = controllerPose[0];
    double z = controllerPose[1];
    
    const double cos45 = 0.70710678118;
    const double sin45 = 0.70710678118;
    
    double y_rotated = y * cos45 - z * sin45;
    double z_rotated = y * sin45 + z * cos45;
    
    ur5Pose[0] = x;
    ur5Pose[1] = y_rotated;
    ur5Pose[2] = z_rotated;
    
    Eigen::Quaterniond q_controller(controllerPose[6], controllerPose[3], controllerPose[4], controllerPose[5]);
    Eigen::Quaterniond rotX_controller(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond q_controller_rotated = rotX_controller * q_controller;

    Eigen::Quaterniond q_robot(q_controller_rotated.w(), q_controller_rotated.y(), q_controller_rotated.x(), -q_controller_rotated.z());
    Eigen::Quaterniond rotX_robot(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond q_robot_rotated = rotX_robot * q_robot;

    Eigen::Matrix3d rotMatrix = q_robot_rotated.toRotationMatrix();
    Eigen::Vector3d euler = rotMatrix.eulerAngles(2, 1, 0);
    auto rotVec = eulerToRobotRotVectorRight(euler);
    
    ur5Pose[3] = rotVec[0];
    ur5Pose[4] = rotVec[1];
    ur5Pose[5] = rotVec[2];
    
    return ur5Pose;
}

void OnPXREAClientCallback(void* context, PXREAClientCallbackType type, int status, void* userData)
{
    switch (type)
    {
    case PXREAServerConnect:
        std::cout << "server connect" << std::endl;
        break;
    case PXREAServerDisconnect:
        std::cout << "server disconnect" << std::endl;
        break;
    case PXREADeviceFind:
        std::cout << "device find" << (const char*)userData << std::endl;
        break;
    case PXREADeviceMissing:
        std::cout << "device missing" << (const char*)userData << std::endl;
        break;
    case PXREADeviceConnect:
        std::cout << "device connect" << (const char*)userData << status << std::endl;
        break;
    case PXREADeviceStateJson:
        auto& dsj = *((PXREADevStateJson*)userData);
        {
            std::lock_guard<std::mutex> lock(coutMutex);
        }
        try {
            json data = json::parse(dsj.stateJson);
            if (data.contains("value")) {
                auto value = json::parse(data["value"].get<std::string>());
                if (value["Controller"].contains("left")) {
                    auto& left = value["Controller"]["left"];
                    {
                        std::lock_guard<std::mutex> lock(leftPoseMutex);
                        LeftControllerPose = stringToPoseArray(left["pose"].get<std::string>());
                        LeftTrigger = left["trigger"].get<double>();
                        LeftGrip = left["grip"].get<double>();
                    }
                }
                if (value["Controller"].contains("right")) {
                    auto& right = value["Controller"]["right"];
                    {
                        std::lock_guard<std::mutex> lock(rightPoseMutex);
                        RightControllerPose = stringToPoseArray(right["pose"].get<std::string>());
                        RightTrigger = right["trigger"].get<double>();
                        RightGrip = right["grip"].get<double>();
                    }
                }
                if (value.contains("Head")) {
                    auto& headset = value["Head"];
                    {
                        std::lock_guard<std::mutex> lock(headsetPoseMutex);
                        HeadsetPose = stringToPoseArray(headset["pose"].get<std::string>());
                    }
                }
            }
        } catch (const json::exception& e) {
            std::cerr << "JSON parsing error: " << e.what() << std::endl;
        }
        break;
    }
}

// Function to control left UR5
void leftUR5Control() {
    try {
        {
            std::lock_guard<std::mutex> lock(coutMutex);
            std::cout << "Connecting to left UR5 robot at " << LEFT_ROBOT_IP << "..." << std::endl;
        }
        
        ur_rtde::RTDEControlInterface left_rtde_control(LEFT_ROBOT_IP);
        ur_rtde::RTDEReceiveInterface left_rtde_receive(LEFT_ROBOT_IP);
        
        ur_rtde::RobotiqGripper left_gripper(LEFT_ROBOT_IP);
        left_gripper.connect();
        
        if (!left_gripper.isActive()) {
            left_gripper.activate();
        }
        left_gripper.setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_NORMALIZED);
        
        left_gripper.setSpeed(GRIPPER_SPEED);
        left_gripper.setForce(GRIPPER_FORCE);
        
        left_gripper.open();
        double left_previous_trigger = 0.0;
        
        bool left_success = left_rtde_control.moveJ(LEFT_INITIAL_JOINT);
        
        std::vector<double> left_robot_pose = left_rtde_receive.getActualTCPPose();
        std::vector<double> left_target_pose = left_robot_pose;
        std::array<double, 7> left_previous_pose{0};
        bool left_is_first_valid_pose = true;
        bool wasGripPressed = false;
        while (running) {
            if (!left_rtde_receive.isConnected()) {
                left_rtde_receive.reconnect();
                std::cout << "Left RTDE receive connection lost! Reconnecting..." << std::endl;
            }
            if(!left_rtde_control.isConnected()) {
                left_rtde_control.reconnect();
                std::cout << "Left RTDE control connection lost! Reconnecting..." << std::endl;
            }
            double currentGrip = LeftGrip.load();
            bool isGripPressed = currentGrip > 0.9;
            double currentTrigger = LeftTrigger.load();
            if (std::abs(currentTrigger - left_previous_trigger) > 0.01) {
                left_gripper.move(1.0f - static_cast<float>(currentTrigger), GRIPPER_SPEED, GRIPPER_FORCE, ur_rtde::RobotiqGripper::START_MOVE);
                left_previous_trigger = currentTrigger;
            }

            if (wasGripPressed && !isGripPressed) {
                left_target_pose = left_rtde_receive.getActualTCPPose();
                std::vector<double> check_pose = left_target_pose;
                std::cout << "left_target_pose_check: " << check_pose[0] << ", " << check_pose[1] << ", " << check_pose[2] << std::endl;
                check_pose = {0, 0, 0, 0, 0, 0};
                left_is_first_valid_pose = true;
            }

            if (isGripPressed) {
                std::array<double, 7> currentPose;
                {
                    std::lock_guard<std::mutex> lock(leftPoseMutex);
                    currentPose = LeftControllerPose;
                }
                
                if (!isValidControllerPoseLeft(currentPose)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(17));
                    wasGripPressed = isGripPressed;
                    continue;
                }
                
                if (left_is_first_valid_pose) {
                    left_previous_pose = currentPose;
                    left_is_first_valid_pose = false;
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    wasGripPressed = isGripPressed;
                    continue;
                }
                
                auto relativeChange = calculateRelativePoseChangeLeft(currentPose, left_previous_pose);
                
                // Apply relative change to position (first 3 elements)
                for (int i = 0; i < 3; ++i) {
                    left_target_pose[i] += relativeChange[i];
                }
                
                // Use absolute values for orientation (last 3 elements)
                for (int i = 3; i < 6; ++i) {
                    left_target_pose[i] = relativeChange[i];
                }
                
                left_previous_pose = currentPose;
            }

            left_rtde_control.servoL(left_target_pose, MAX_VELOCITY, MAX_ACCELERATION, SERVO_TIME, LOOKAHEAD_TIME, SERVO_GAIN);

            wasGripPressed = isGripPressed;
            std::this_thread::sleep_for(std::chrono::milliseconds(17));
        }

        //cleanup
        std::cout << "Disconnecting left robot..." << std::endl;
        left_gripper.disconnect();
        left_rtde_control.stopScript();
        left_rtde_control.disconnect();
        left_rtde_receive.disconnect();
        
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(coutMutex);
        std::cerr << "Left UR5 Error: " << e.what() << std::endl;
    }
}

void rightUR5Control() {
    try {
        {
            std::lock_guard<std::mutex> lock(coutMutex);
            std::cout << "Connecting to right UR5 robot at " << RIGHT_ROBOT_IP << "..." << std::endl;
        }
        
        ur_rtde::RTDEControlInterface right_rtde_control(RIGHT_ROBOT_IP);
        ur_rtde::RTDEReceiveInterface right_rtde_receive(RIGHT_ROBOT_IP);
        
        ur_rtde::RobotiqGripper right_gripper(RIGHT_ROBOT_IP);
        right_gripper.connect();
        
        if (!right_gripper.isActive()) {
            right_gripper.activate();
        }
        right_gripper.setUnit(ur_rtde::RobotiqGripper::POSITION, ur_rtde::RobotiqGripper::UNIT_NORMALIZED);
        
        right_gripper.setSpeed(GRIPPER_SPEED);
        right_gripper.setForce(GRIPPER_FORCE);
        
        right_gripper.open();
        double right_previous_trigger = 0.0;
        
        bool right_success = right_rtde_control.moveJ(RIGHT_INITIAL_JOINT);
        
        std::vector<double> right_robot_pose = right_rtde_receive.getActualTCPPose();
        std::vector<double> right_target_pose = right_robot_pose;
        std::array<double, 7> right_previous_pose{0};
        bool right_is_first_valid_pose = true;

        while (running) {
            if (!right_rtde_receive.isConnected()) {
                right_rtde_receive.reconnect();
                std::cout << "Right RTDE receive connection lost! Reconnecting..." << std::endl;
            }
            if(!right_rtde_control.isConnected()) {
                right_rtde_control.reconnect();
                std::cout << "Right RTDE control connection lost! Reconnecting..." << std::endl;
            }
            double currentGrip = RightGrip.load();
            bool isGripPressed = currentGrip > 0.9;
            
            double currentTrigger = RightTrigger.load();
            if (std::abs(currentTrigger - right_previous_trigger) > 0.01) {
                right_gripper.move(1.0f - static_cast<float>(currentTrigger), GRIPPER_SPEED, GRIPPER_FORCE, ur_rtde::RobotiqGripper::START_MOVE);
                right_previous_trigger = currentTrigger;
            }

            if (isGripPressed) {
                std::array<double, 7> currentPose;
                {
                    std::lock_guard<std::mutex> lock(rightPoseMutex);
                    currentPose = RightControllerPose;
                }
                
                if (!isValidControllerPoseRight(currentPose)) {
                    std::this_thread::sleep_for(std::chrono::milliseconds(17));
                    continue;
                }
                
                if (right_is_first_valid_pose) {
                    right_previous_pose = currentPose;
                    right_is_first_valid_pose = false;
                    continue;
                }
                
                auto relativeChange = calculateRelativePoseChangeRight(currentPose, right_previous_pose);
                
                // Apply relative change to position (first 3 elements)
                for (int i = 0; i < 3; ++i) {
                    right_target_pose[i] += relativeChange[i];
                }
                
                // Use absolute values for orientation (last 3 elements)
                for (int i = 3; i < 6; ++i) {
                    right_target_pose[i] = relativeChange[i];
                }
                
                right_previous_pose = currentPose;
            } else {
                right_is_first_valid_pose = true;
            }

            right_rtde_control.servoL(right_target_pose, MAX_VELOCITY, MAX_ACCELERATION, SERVO_TIME, LOOKAHEAD_TIME, SERVO_GAIN);
            std::this_thread::sleep_for(std::chrono::milliseconds(17));
        }
        //cleanup
        std::cout << "Disconnecting right robot..." << std::endl;
        right_gripper.disconnect();
        right_rtde_control.stopScript();
        right_rtde_control.disconnect();
        right_rtde_receive.disconnect();
        
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(coutMutex);
        std::cerr << "Right UR5 Error: " << e.what() << std::endl;
    }
}

std::vector<double> convertControllerToUR5PoseLeft(const std::array<double, 7>& controllerPose) {
    std::vector<double> ur5Pose(6);
    
    double x = -controllerPose[2];
    double y = -controllerPose[0];
    double z = controllerPose[1];
    
    const double cos45 = 0.70710678118;
    const double sin45 = -0.70710678118; 
    
    double y_rotated = y * cos45 + z * sin45;
    double z_rotated = -y * sin45 + z * cos45;
    
    ur5Pose[0] = x;
    ur5Pose[1] = y_rotated;
    ur5Pose[2] = z_rotated;
    
    Eigen::Quaterniond quaternion_controller(controllerPose[6], controllerPose[3], controllerPose[4], controllerPose[5]);
    Eigen::Quaterniond rotX_controller(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond quaternion_controller_rotated = rotX_controller * quaternion_controller;
    Eigen::Quaterniond quaternion_robot(quaternion_controller_rotated.w(), quaternion_controller_rotated.y(), quaternion_controller_rotated.x(), -quaternion_controller_rotated.z());
    Eigen::Quaterniond rotX_robot(Eigen::AngleAxisd(M_PI/4, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond rotZ_robot(Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ()));
    Eigen::Quaterniond quaternion_robot_rotated = rotX_robot * rotZ_robot * quaternion_robot;
    Eigen::Matrix3d rotMatrix = quaternion_robot_rotated.toRotationMatrix();
    Eigen::Vector3d euler = rotMatrix.eulerAngles(2, 1, 0);
    auto rotVec = eulerToRobotRotVectorRight(euler);
    
    ur5Pose[3] = rotVec[0];
    ur5Pose[4] = rotVec[1];
    ur5Pose[5] = rotVec[2];
    
    return ur5Pose;
}

std::vector<double> calculateRelativePoseChangeLeft(const std::array<double, 7>& currentPose, 
                                              const std::array<double, 7>& previousPose) {
    std::vector<double> relativePose(6);
    auto current = convertControllerToUR5PoseLeft(currentPose);
    auto previous = convertControllerToUR5PoseLeft(previousPose);
    
    // Calculate position differences (first 3 elements)
    for (int i = 0; i < 3; ++i) {
        relativePose[i] = current[i] - previous[i];
    }
    
    // Use absolute values for orientation (last 3 elements)
    for (int i = 3; i < 6; ++i) {
        relativePose[i] = current[i];
    }
    
    return relativePose;
}

std::vector<double> calculateRelativePoseChangeRight(const std::array<double, 7>& currentPose, 
                                               const std::array<double, 7>& previousPose) {
    std::vector<double> relativePose(6);
    auto current = convertControllerToUR5PoseRight(currentPose);
    auto previous = convertControllerToUR5PoseRight(previousPose);
    
    // Calculate position differences (first 3 elements)
    for (int i = 0; i < 3; ++i) {
        relativePose[i] = current[i] - previous[i];
    }
    
    // Use absolute values for orientation (last 3 elements)
    for (int i = 3; i < 6; ++i) {
        relativePose[i] = current[i];
    }
    
    return relativePose;
}

int main(int argc, char *argv[])
{
    // Register signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    PXREAInit(NULL, OnPXREAClientCallback, PXREAFullMask);

    // Start robot control threads
    std::thread leftRobotThread(leftUR5Control);
    std::thread rightRobotThread(rightUR5Control);
    std::thread dynamixelThread(dynamixelControl);

    while(1)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // Calculate roll, pitch, yaw from quaternion
        Eigen::Quaterniond q(HeadsetPose[6], HeadsetPose[3], HeadsetPose[4], HeadsetPose[5]); // w, x, y, z
        
        
        // Get Euler angles in the new coordinate system (ZYX order)
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, -0, 1);
        
        // Convert to degrees for better readability
        double roll = euler[0] * 180.0 / M_PI;  
        double pitch = euler[1] * 180.0 / M_PI; 
        double yaw = euler[2] * 180.0 / M_PI;   
        if (yaw>90 && yaw<180) {yaw-=180;}
        if (pitch<-90) {pitch= -pitch-180;}
        if (pitch>90) {pitch= 180-pitch;}
        std::cout << "Headset Orientation (degrees) - Roll: " << std::fixed << std::setprecision(2) 
                  << roll << "° Pitch: " << pitch << "° Yaw: " << yaw << "°" << std::endl;
    }
    PXREADeinit();
    //Cleanup
    running = false;
    dynamixel_running = false;
    
    leftRobotThread.join();
    rightRobotThread.join();
    dynamixelThread.join();

    return 0;
}