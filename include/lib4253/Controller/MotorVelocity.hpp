/**
 * @file MotorVelocity.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Accurate velocity controller for motors
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "main.h"


/**
 * @brief Velocity Controller class
 *
 */
class MotorVelocityController{
    double kV, kA, kP;
    public:
    /**
     * @brief Sets gain for velocity controller
     *
     * @param a velocity gain
     * @param b acceleration gain
     * @param c proportional gain
     */
    void setGain(double a, double b, double c);

    /**
     * @brief Calulates raw power for motor
     *
     * @param v stores the velocity and acceleration for each time step
     * @param currentRPM current motor velocity
     * @return power to be fed into the motor
     */
    double calcPower(TrajectoryPoint v, double currentRPM);
};
