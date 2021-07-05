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
#include "main.h"
#include "lib4253/Splines/Trajectory.hpp"

namespace lib4253{

struct MotorVelocityControllerGain {
    double kV, kA, kP;
};

/**
 * @brief Velocity Controller class
 *
 */
class MotorVelocityController : public AbstractVelocityController<MotorVelocityControllerGain> {
    MotorVelocityControllerGain gain;
    public:
    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     */
    MotorVelocityController();

    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     * @param gain kV, kA, kP
     */
    MotorVelocityController(const MotorVelocityControllerGain& gain);

    /**
     * @brief Destructs the velocity controller
     *
     */
    ~MotorVelocityController() = default;

    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     * @param gain kV, kA, kP
     */
    void setGain(const MotorVelocityControllerGain& gain);

    /**
     * @brief Calulates raw power for motor
     *
     * @param velocity stores the target velocity
     * @param acceleration stores the target acceleration
     * @param currentRPM current motor velocity
     * @return power (voltage) to be fed into the motor
     */
    double calcPower(const double& velocity, const double& acceleration, const double& currentRPM) const;

    /**
     * @brief Calulates raw power for motor
     *
     * @param v stores the velocity and acceleration for each time step
     * @param currentRPM current motor velocity
     * @return power to be fed into the motor
     */
    double calcPower(const TrajectoryPoint& v, const double& currentRPM) const;
};
}

