/**
 * @file BangBang.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw) & Jason Zhou (24JasonZ@students.tas.tw)
 * @brief Bang Bang controller - simply stated, a primitive PID controller
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */


#pragma once
#include "main.h"
#include "lib4253/Controller/VelocityController.hpp"

/**
 * @brief Structure to store the BangBang gain
 * 
 */
struct BangBangGain {
    double highPower, lowPower, targetVel;
};

/**
 * @brief BangBang class
 *
 */
class BangBang : public AbstractVelocityController<BangBangGain> {
    BangBangGain gain;
    /**
     * @brief Construct a new Bang Bang object
     * 
     * @param gain highPower, lowPower, targetVel
     */
    BangBang(BangBangGain gain);

    /**
     * @brief Sets gain
     * 
     * @param gain highPower, lowPower, targetVel
     */
    void setGain(BangBangGain gain);

    /**
     * @brief Sets target velocity for Bang Bang controller
     *
     * @param t target velocity
     */
    void setTargetVel(double t);
    
    /**
     * @brief Determines the amount of power needed based on real-time velocity
     *
     * @param val actual velocity
     * @return new velocity
     */
    double step(double val);
};
