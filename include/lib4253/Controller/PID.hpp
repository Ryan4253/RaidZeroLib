/**
 * @file PID.hpp
 * @author Ryan Liao (23RyanL@students.tas.tw)
 * @brief PID controller
 * @version 0.1
 * @date 2021-05-20
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#include "lib4253/Filter/EMA.hpp"
#include "pros/rtos.hpp"
#include <math.h>
namespace lib4253{
// class PID {
//     private:
//     double kP, kI, kD;
// };

struct PIDGain {
    double kP, kI, kD;
};

class PID : public AbstractVelocityController<PIDGain> {
    private:
    PIDGain gain;
    double error, prevError, integral, derivative;
    double maxIntegral, minDist;
    double time, prevTime;
    lib4253::EmaFilter dEMA;

    public:
    /**
     * @brief Construct a new PID object
     *
     */
    PID();

    /**
     * @brief Construct a new PID object
     * 
     * @param gain kP, kI, kD
     */
    PID(const PIDGain& gain);

    /**
     * @brief Destroys the PID object
     * 
     */
    ~PID() = default;

    /**
     * @brief Construct a new PID object
     * 
     * @param gain kP, kI, kD
     */
    void setGain(const PIDGain& gain);

    /**
     * @brief Set integral gain
     *
     * @param windup
     * @param dist
     */
    void setIGain(const double& windup, const double& dist);

    /**
     * @brief Sets gain for exponential moving average
     *
     * @param alpha EMA gain
     */
    void setEMAGain(const double& alpha);

    /**
     * @brief Initializes PID controller
     *
     */
    void initialize();

    /**
     * @brief Updates PID controller - the main meat of the PID controller
     *
     * @param val error or how far the robot's from the target location
     * @return power to the motor
     */
    double step(const double& val);

    double getError() const;
};

/**
 * @brief PID Controller class with feed forward
 *
 */
class FPID : PID{
    private:
    double kF, target;
    
    public:
    /**
     * @brief Sets Feed Forward gain
     *
     * @param f FF gain
     */
    void setFGain(const double& f);
    
    /**
     * @brief Set desired target to calculate FF
     *
     * @param t target distance
     */
    void setTarget(const double& t);
    
    /**
     * @brief Updates raw power based on FF
     *
     * @param error error or how far the robot's from the target location
     * @return updated power to the motor
     */
    double fUpdate(const double& error);
};
}