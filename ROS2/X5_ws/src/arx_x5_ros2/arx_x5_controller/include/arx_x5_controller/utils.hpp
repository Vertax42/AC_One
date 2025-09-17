#pragma once

#include <vector>
#include <functional>
#include <memory>
#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace arx::x5::utils {

    /**
     * @brief Linear interpolation function
     * @param t Interpolation parameter [0, 1]
     * @return Interpolation result
     */
double linear_func(double t);

/**
 * @brief Quadratic Bezier interpolation function (easeInOutQuad)
 * @param t Interpolation parameter [0, 1]
 * @return Interpolation result
 */
double easeInOutQuad(double t);

/**
 * @brief Cubic Bezier easing function (easeInOutCubic)
 * @param t Interpolation parameter [0, 1]
 * @return Interpolation result
 */
double easeInOutCubic(double t);

/**
 * @brief Sine easing function (ease-in)
 * Motion curve: slow-fast-slow, suitable for scenarios requiring slow startup, such as robot arm startup
 * @param t Interpolation parameter [0, 1]
 * @return Interpolation result
 */
double easeInSine(double t);

/**
 * @brief Sine easing function (ease-out)
 * Motion curve: fast-fast-slow, suitable for scenarios requiring smooth stopping, such as robot arm stopping
 * @param t Interpolation parameter [0, 1]
 * @return Interpolation result
 */
double easeOutSine(double t);

/**
 * @brief Sine easing function (ease-in-out)
 * Motion curve: slow-fast-slow, suitable for scenarios requiring smooth motion
 * @param t Interpolation parameter [0, 1]
 * @return Interpolation result
 */
double easeInOutSine(double t);

/**
 * @brief Smooth joint interpolation class
 * Used to implement smooth interpolation motion for robot arm joints
 */
class SmoothJointInterpolator {
public:
    /**
     * @brief Constructor
     * @param controller Reference to X5Controller instance
     * @param logger ROS2 logger
     * @param control_dt Control period (seconds)
     * @param print_interval Print interval (steps)
     */
    SmoothJointInterpolator(
        std::shared_ptr<void> controller,  // Shared pointer to X5Controller for safety
        rclcpp::Logger logger,
        double control_dt = 0.01,
        int print_interval = 100
    );

    /**
     * @brief Execute smooth interpolation
     * @param target_poses Target joint positions (6 joints)
     * @param duration Interpolation duration (seconds)
     * @param gripper_target Target gripper position (meters)
     * @param interpolation_func Interpolation function (default easeInOutQuad)
     * @return Whether execution was successful
     */
    bool interpolate(
        const std::vector<double>& target_poses,
        double duration,
        double gripper_target = 0.0,
        std::function<double(double)> interpolation_func = easeInOutQuad
    );

    /**
     * @brief Set controller interface
     * @param controller Reference to X5Controller instance
     */
    void setController(std::shared_ptr<void> controller);

    /**
     * @brief Set control period
     * @param control_dt Control period (seconds)
     */
    void setControlDt(double control_dt);

    /**
     * @brief Set print interval
     * @param print_interval Print interval (steps)
     */
    void setPrintInterval(int print_interval);

    /**
     * @brief Check if currently interpolating
     * @return Whether currently interpolating
     */
    bool isInterpolating() const;

    /**
     * @brief Stop interpolation
     */
    void stopInterpolation();

private:
    std::shared_ptr<void> controller_;         // Shared pointer to X5Controller instance
    rclcpp::Logger logger_;            // ROS2 logger
    double control_dt_;                // Control period
    int print_interval_;              // Print interval
    bool is_interpolating_;           // Whether currently interpolating
    std::chrono::steady_clock::time_point start_time_;  // Start time

    /**
     * @brief Get current joint state
     * @param poses Output joint positions
     * @param gripper_pos Output gripper position
     * @return Whether retrieval was successful
     */
    bool getCurrentJointState(std::vector<double>& poses, double& gripper_pos);

    /**
     * @brief Set joint command
     * @param poses Joint positions
     * @param gripper_pos Gripper position
     * @return Whether setting was successful
     */
    bool setJointCommand(const std::vector<double>& poses, double gripper_pos);
};

}  // namespace arx::x5::utils
