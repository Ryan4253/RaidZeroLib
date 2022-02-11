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
#include "okapi/api/control/iterative/iterativeController.hpp"
#include "okapi/api/control/iterative/iterativeVelocityController.hpp"
namespace lib4253{

struct BangBangGain {
    double highPower{12000}, targetPower{6000}, lowPower{4000}, range{100};
};

/**
 * @brief BangBang class
 *
 */
class IterativeVelBangBangController : okapi::IterativeVelocityController<double, double>{
    public:
    /**
     * @brief Construct a new Bang Bang object
     * 
     * @param gain highPower, lowPower, targetVel
     */
    BangBang() = default;

    /**
     * @brief Sets gain
     * 
     * @param gain highPower, lowPower, targetVel
     */
    BangBang(const BangBangGain& gain);

    /**
     * @brief Destroys the Bang Bang object
     * 
     */
    ~BangBang() = default;
    
    void reset() override;

    void initialize() override;

    /**
     * @brief Determines the amount of power needed based on real-time velocity
     *
     * @param val error
     * @return new velocity
     */
    double step(const double& val);
};
}
