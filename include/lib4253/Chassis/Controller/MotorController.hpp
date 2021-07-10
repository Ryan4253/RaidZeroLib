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
#include "lib4253/Utility/Units.hpp"

namespace lib4253{

struct MotorControllerGain {
    double kV{0}, kA{0}, kP{0};
};

/**
 * @brief Velocity Controller class
 *
 */
class MotorController{
    MotorControllerGain gain;
    public:
    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     */
    MotorController() = default;

    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     * @param gain kV, kA, kP
     */
    MotorController(const MotorControllerGain& gain);

    /**
     * @brief Destructs the velocity controller
     *
     */
    ~MotorController() = default;

    /**
     * @brief Construct a new Motor Velocity Controller object
     * 
     * @param gain kV, kA, kP
     */
    void setGain(const MotorControllerGain& gain);

    /**
     * @brief Calulates raw power for motor
     *
     * @param velocity stores the target velocity
     * @param acceleration stores the target acceleration
     * @param currentRPM current motor velocity
     * @return power (voltage) to be fed into the motor
     */
    double step(const okapi::QSpeed& velocity, const okapi::QAcceleration& acceleration, const okapi::QSpeed& currentSpeed) const;
};
}

