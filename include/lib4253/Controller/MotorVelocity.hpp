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

struct MotorVelocityControllerGain {
    double kV, kA, kP;
};

/**
 * @brief Velocity Controller class
 *
 */
class MotorVelocityController{
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
     * @param a velocity gain
     * @param b acceleration gain
     * @param c proportional gain
     */
    MotorVelocityController(MotorVelocityControllerGain gain);

    /**
     * @brief Sets gain for velocity controller
     *
     * @param a velocity gain
     * @param b acceleration gain
     * @param c proportional gain
     */
    void setGain(MotorVelocityControllerGain gain);

    /**
     * @brief Calulates raw power for motor
     *
     * @param v stores the velocity and acceleration for each time step
     * @param currentRPM current motor velocity
     * @return power to be fed into the motor
     */
    double calcPower(TrajectoryPoint v, double currentRPM);
};
