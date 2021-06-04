/**
 * @file BangBang.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Bang Bang controller - simply stated, a primitive PID controller
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */


#pragma once
#include "main.h"
namespace lib4253{

/**
 * @brief BangBang class
 *
 */
class BangBang{

    double highPower, lowPower, targetVel;
    /**
     * @brief Construct a new Bang Bang object
     *
     * @param h sets velocity to h when the actual velocity is less than the target velocity
     * @param l sets velocity to l when the actual velocity is greater than the target velocity
     * @param t target velocity
     */
    BangBang(double h, double l, double t);

    /**
     * @brief Sets target velocity for Bang Bang controller
     *
     * @param t target velocity
     */
    void setTargetVel(double t);
    
    /**
     * @brief Determines the amount of power needed based on real-time velocity
     *
     * @param v actual velocity
     * @return new velocity
     */
    double step(double v);
};
}
