/**
 * @file MotorVelocity.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw) & Jason Zhou (24JasonZ@students.tas.tw)
 * @brief Accurate velocity controller for motors
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Splines/Trajectory.hpp"
namespace lib4253{

/**
 * @brief Velocity Controller class
 *
 */
class MotorVelocityController{
    double kV{0}, kA{0}, kP{0};
    public:
    /**
     * @brief Constructs the velocity controller
     *
     * @param _kV velocity gain
     * @param _kA acceleration gain
     * @param _kP proportional gain
     */
    MotorVelocityController(const double& _kV, const double& _kA, const double& _kP);

    /**
     * @brief Destructs the velocity controller
     *
     */
    ~MotorVelocityController() = default;

    /**
     * @brief Sets gain for velocity controller
     *
     * @param _kV velocity gain
     * @param _kA acceleration gain
     * @param _kP proportional gain
     */
    void setGain(const double& _kV, const double& _kA, const double& _kP);

    /**
     * @brief Calulates raw power for motor
     *
     * @param velocity stores the target velocity
     * @param acceleration stores the target acceleration
     * @param currentRPM current motor velocity
     * @return power to be fed into the motor
     */
    double calcPower(const double& velocity, const double& acceleration, const double& currentRPM);

    /**
     * @brief Calulates raw power for motor
     *
     * @param v stores the velocity and acceleration for each time step
     * @param currentRPM current motor velocity
     * @return power to be fed into the motor
     */
    double calcPower(const TrajectoryPoint& v, const double& currentRPM);
};
}