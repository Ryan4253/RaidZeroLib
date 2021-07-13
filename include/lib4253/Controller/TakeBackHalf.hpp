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
#include "lib4253/Utility/Math.hpp"
#include "lib4253/Controller/AbstractVelocityController.hpp"
#include <cmath>

namespace lib4253{

struct TBHGain {
    double gain, approxVel;
};

/**
 * @brief TBH class
 *
 */
class TakeBackHalf : public AbstractVelocityController<TBHGain> {
    private:
    double prevError, tbh;
    bool firstCross;
    
    public:
    /**
     * @brief Construct a new Take Back Half object
     * 
     */
    TakeBackHalf() = default;;

    /**
     * @brief Construct a new Take Back Half object
     *
     * @param g TBH gain
     */
    TakeBackHalf(const TBHGain& gain);

    /**
     * @brief Destroys the Take Back Half object
     * 
     */
    ~TakeBackHalf() = default;
    
    /**
     * @brief Initializes TBH controller
     *
     */
    void initialize() override;

    void reset() override;
    
    /**
     * @brief Calculates power for the motor
     *
     * @param rpm current motor velocity
     * @return modified motor velocity
     */
    double step(const double& val) override;
};  
}