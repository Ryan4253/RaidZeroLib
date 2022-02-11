/**
 * @file Slew.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief Slew controller
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include <cmath>
#include "lib4253/Utility/Math.hpp"
#include "okapi/api/control/iterative/iterativeController.hpp"

namespace lib4253{

/**
 * @brief Slew controller class
 *
 */
class SlewController : okapi::IterativeController<double, double>{

    public:
    struct Gains{
        double accStep{0.0}, decStep{0.0};

        Gains() = default;

        ~Gains() = default;

        Gains(double accStep, double decStep);

        bool operator==(const Gains& rhs) const;

        bool operator!=(const Gains& rhs) const;
    };
    
    /**
     * @brief Construct a new Slew Controller object
     *
     */
    SlewController() = default;

    /**
     * @brief Destroys the Slew Controller object
     * 
     */
    ~SlewController() = default;
    
    /**
     * @brief Construct a new Slew Controller object
     *
     * @param accel acceleration step (how much the velocity increases everytime the robot refreshes)
     * @param decel deceleration step (^ vise versa)
     */
    SlewController(double iAccStep, double iDecStep);
        
    /**
     * @brief Construct a new Slew Controller object
     *
     * @param accel acceleration step (how much the velocity increases everytime the robot refreshes)
     * @param decel deceleration step (^ vise versa)
     */
    SlewController(const Gains& iGain);

    /**
     * @brief Resets slew controller
     * 
     */
    void reset();
    
    /**
     * @brief Calculates output power
     *
     * @param val target power
     * @return (possibly) modified power
     */
    double step(const double& val);

    protected:
    double speed{0.0};
    Gains gain;
};
}