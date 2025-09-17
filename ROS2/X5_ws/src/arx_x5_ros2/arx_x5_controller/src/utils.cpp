#include "arx_x5_controller/utils.hpp"
#include "arx_x5_controller/X5Controller.hpp"
#include "arx_x5_src/interfaces/InterfacesThread.hpp"
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>

namespace arx::x5::utils {

// 插值函数实现
double linear_func(double t) {
    return t;
}

double easeInOutQuad(double t) {
    t *= 2;
    if (t < 1) {
        return t * t / 2;
    } else {
        t -= 1;
        return -(t * (t - 2) - 1) / 2;
    }
}

double easeInOutCubic(double t) {
    if (t < 0.5) {
        return 4 * t * t * t;
    } else {
        t = 2 * t - 2;
        return (t * t * t + 2) / 2;
    }
}

double easeInSine(double t) {
    return 1 - std::cos(t * M_PI / 2);
}

double easeOutSine(double t) {
    return std::sin(t * M_PI / 2);
}

double easeInOutSine(double t) {
    return -(std::cos(M_PI * t) - 1) / 2;
}

// SmoothJointInterpolator 实现
SmoothJointInterpolator::SmoothJointInterpolator(
    std::shared_ptr<void> controller,
    rclcpp::Logger logger,
    double control_dt,
    int print_interval
) : controller_(controller), logger_(logger), control_dt_(control_dt), print_interval_(print_interval), is_interpolating_(false) {
}

bool SmoothJointInterpolator::interpolate(
    const std::vector<double>& target_poses,
    double duration,
    double gripper_target,
    std::function<double(double)> interpolation_func
) {
    // Controller reference is always valid (passed by reference)

    if (target_poses.size() != 6) {
        RCLCPP_ERROR(logger_, "Target poses must have 6 elements!");
        return false;
    }

    if (duration <= 0.5)
    {
        RCLCPP_ERROR(logger_, "Duration must be greater than 0.5s!");
        return false;
    }

    if (control_dt_ <= 0) {
        RCLCPP_ERROR(logger_, "Control dt must be greater than 0!");
        return false;
    }

    // calculate step number
    int step_num = static_cast<int>(duration / control_dt_);
    if (step_num <= 0) {
        RCLCPP_ERROR(logger_, "Invalid duration or control_dt!");
        return false;
    }

    if (gripper_target < 0 || gripper_target > 1) {
        RCLCPP_ERROR(logger_, "Gripper target must be between 0 and 1!");
        return false;
    }
    
    // Convert gripper_target from [0, 1] to [0, gripper_coef_] range
    if (!controller_) {
        RCLCPP_ERROR(logger_, "Controller is null!");
        return false;
    }
    
    auto* controller = static_cast<arx::x5::X5Controller*>(controller_.get());
    if (!controller) {
        RCLCPP_ERROR(logger_, "Failed to cast controller to X5Controller!");
        return false;
    }
    
    double gripper_coef = controller->getGripperCoef();
    double gripper_target_actual = gripper_target * gripper_coef;
    // get initial state
    std::vector<double> initial_poses(6);
    double initial_gripper_pos;
    if (!getCurrentJointState(initial_poses, initial_gripper_pos)) {
        RCLCPP_ERROR(logger_, "Failed to get current joint state!");
        return false;
    }

    RCLCPP_INFO(logger_, "Starting smooth interpolation: %d steps, duration: %.3fs", step_num, duration);
    RCLCPP_INFO(logger_, "Initial position: [%.3f, %.3f, %.3f]", initial_poses[0], initial_poses[1], initial_poses[2]);
    RCLCPP_INFO(logger_, "Target position: [%.3f, %.3f, %.3f]", target_poses[0], target_poses[1], target_poses[2]);
    RCLCPP_INFO(logger_, "Gripper: initial=%.3f, target=%.3f (normalized), target_actual=%.3f (raw), coef=%.3f", 
                initial_gripper_pos, gripper_target, gripper_target_actual, gripper_coef);

    is_interpolating_ = true;
    start_time_ = std::chrono::steady_clock::now();

    // Execute interpolation with gentle start
    for (int i = 0; i < step_num && is_interpolating_; ++i) {
        // Calculate interpolation parameter (0 to 1)
        double t = (step_num > 1) ? static_cast<double>(i) / (step_num - 1) : 0.0;
        
        // Apply gentle start for first 10% of movement to reduce jitter
        double gentle_t = t;
        if (t < 0.1) {
            // Use a gentler curve for the first 10% of movement
            gentle_t = t * t * (3.0 - 2.0 * t); // Smooth step function
        }
        
        double alpha = interpolation_func(gentle_t);

        // Calculate current target position
        std::vector<double> current_poses(6);
        for (int j = 0; j < 6; ++j) {
            current_poses[j] = initial_poses[j] + alpha * (target_poses[j] - initial_poses[j]);
        }
        
        double current_gripper_pos = initial_gripper_pos + alpha * (gripper_target_actual - initial_gripper_pos);

        // Set command
        if (!setJointCommand(current_poses, current_gripper_pos)) {
            RCLCPP_ERROR(logger_, "Failed to set joint command at step %d", i);
            is_interpolating_ = false;
            return false;
        }

        // Print status periodically
        if (i % print_interval_ == 0) {
            RCLCPP_DEBUG(logger_, "Step %d: target=[%.3f, %.3f, %.3f]", i, current_poses[0], current_poses[1], current_poses[2]);
        }

        // Wait for control period
        std::this_thread::sleep_for(std::chrono::duration<double>(control_dt_));
    }

    is_interpolating_ = false;
    RCLCPP_INFO(logger_, "Interpolation completed!");
    return true;
}

void SmoothJointInterpolator::setController(std::shared_ptr<void> controller) {
    controller_ = controller;
}

void SmoothJointInterpolator::setControlDt(double control_dt) {
    control_dt_ = control_dt;
}

void SmoothJointInterpolator::setPrintInterval(int print_interval) {
    print_interval_ = print_interval;
}

bool SmoothJointInterpolator::isInterpolating() const {
    return is_interpolating_;
}

void SmoothJointInterpolator::stopInterpolation() {
    is_interpolating_ = false;
}

bool SmoothJointInterpolator::getCurrentJointState(std::vector<double>& poses, double& gripper_pos) {
    try {
        // Access interfaces_ptr_ through controller pointer
        if (!controller_) {
            RCLCPP_ERROR(logger_, "Controller is null!");
            return false;
        }
        
        auto* controller = static_cast<arx::x5::X5Controller*>(controller_.get());
        if (!controller) {
            RCLCPP_ERROR(logger_, "Failed to cast controller to X5Controller!");
            return false;
        }
        
        auto interfaces_ptr = controller->getInterfacesPtr();
        
        if (!interfaces_ptr) {
            RCLCPP_ERROR(logger_, "Controller interfaces_ptr_ is null!");
            return false;
        }
        
        // Get current joint positions
        std::vector<double> joint_positions = interfaces_ptr->getJointPositons();
        
        if (joint_positions.size() >= 7) {
            poses.resize(6);
            for (int i = 0; i < 6; ++i) {
                poses[i] = joint_positions[i];
            }
            gripper_pos = joint_positions[6];
            return true;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception in getCurrentJointState: %s", e.what());
    }
    
    return false;
}

bool SmoothJointInterpolator::setJointCommand(const std::vector<double>& poses, double gripper_pos) {
    try {
        // Access interfaces_ptr_ through controller pointer
        if (!controller_) {
            RCLCPP_ERROR(logger_, "Controller is null!");
            return false;
        }
        
        auto* controller = static_cast<arx::x5::X5Controller*>(controller_.get());
        if (!controller) {
            RCLCPP_ERROR(logger_, "Failed to cast controller to X5Controller!");
            return false;
        }
        
        auto interfaces_ptr = controller->getInterfacesPtr();
        
        if (!interfaces_ptr) {
            RCLCPP_ERROR(logger_, "Controller interfaces_ptr_ is null!");
            return false;
        }
        
        // Set joint positions
        interfaces_ptr->setJointPositions(poses);
        
        // Set control mode to position control
        interfaces_ptr->setArmStatus(InterfacesThread::state::POSITION_CONTROL);
        
        // Set gripper position
        interfaces_ptr->setCatch(gripper_pos);
        
        
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(logger_, "Exception in setJointCommand: %s", e.what());
    }
    
    return false;
}

}  // namespace arx::x5::utils
