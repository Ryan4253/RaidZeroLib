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
#include "lib4253/Controller/AbstractVelocityController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include <cmath>
namespace lib4253{

struct PIDGain {
    double kP, kI, kD, maxIntegral, minDist, emaGain;
};

class PID : public AbstractVelocityController<PIDGain> {
    private:
    double prevError{0}, integral{0}, derivative{0};
    lib4253::EmaFilter derivEMA{0};
    okapi::TimeUtil timer = std::move(okapi::TimeUtilFactory::createDefault());

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

    void setPIDGain(const double& kP, const double& kI, const double& kD);

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

    double getIntegral() const;
    
    double getDerivative() const;

    /**
     * @brief Initializes PID controller
     *
     */
    void initialize() override;

    void reset() override;

    /**
     * @brief Updates PID controller - the main meat of the PID controller
     *
     * @param val error or how far the robot's from the target location
     * @return power to the motor
     */
    double step(const double& val) override;
};

struct FFPIDGain {
    double kF, kP, kI, kD, maxIntegral, minDist, emaGain;
};

/**
 * @brief PID Controller class with feed forward
 *
 */
class FFPID : public AbstractVelocityController<FFPIDGain>{
    private:
    PID pid;

    public:
    /**
     * @brief Construct a new PID object
     *
     */
    FFPID() = default;

    /**
     * @brief Construct a new PID object
     * 
     * @param gain kP, kI, kD
     */
    FFPID(const FFPIDGain& gain, const double& target);

    /**
     * @brief Destroys the PID object
     * 
     */
    ~FFPID() = default;

    void setFFGain(const double& kF);

    void setPIDGain(const double& kP, const double& kI, const double& kD);

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

    double getIntegral() const;
    
    double getDerivative() const;

    /**
     * @brief Initializes PID controller
     *
     */
    void initialize() override;

    void reset() override;

    /**
     * @brief Updates PID controller - the main meat of the PID controller
     *
     * @param val error or how far the robot's from the target location
     * @return power to the motor
     */
    double step(const double& val) override;
};
}