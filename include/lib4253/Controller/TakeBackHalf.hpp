/**
 * @file TakeBackHalf.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Take back half (TBH) controller - best used with flywheels
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "main.h"

struct TBHGain {
    double gain;
};

/**
 * @brief TBH class
 *
 */
class TakeBackHalf : public AbstractVelocityController<TBHGain> {
    private:
    double error, prevError;
    double targetVel, approxVel;
    double output, tbhVal;
    TBHGain gain;
    bool firstCross;
    
    public:
    /**
     * @brief Construct a new Take Back Half object
     *
     * @param g TBH gain
     */
    TakeBackHalf(TBHGain gain);
    
    /**
     * @brief Set the gain
     *
     * @param g TBH gain
     */
    void setGain(TBHGain gain);
    
    /**
     * @brief Set the Target Velocity
     *
     * @param target tarvel velocity
     */
    void setTargetVel(double target);
    
    /**
     * @brief Sets the approximate power for the motor after crossing the target output for the first time
     *
     * @param approx approximation of the power
     */
    void setApproxVel(double approx);
    
    /**
     * @brief Initializes TBH controller
     *
     */
    void initialize();
    
    /**
     * @brief Calculates power for the motor
     *
     * @param rpm current motor velocity
     * @return modified motor velocity
     */
    double step(double val);
};  
