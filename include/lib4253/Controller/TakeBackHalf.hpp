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
#include "okapi/api/control/iterative/iterativeVelocityController.hpp"
#include "okapi/api.hpp"
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

class IterativeVelTBHController : public okapi::IterativeVelocityController<double, double>{
    public:
    struct Gains{
        double gain{0.0};
        double approxVel{0.0};

        bool operator==(const Gains& rhs) const;
        bool operator!=(const Gains& rhs) const;
    };

    IterativeVelTBHController(
        double iGain, 
        double iApproxVel, 
        std::unique_ptr<okapi::VelMath> ivelMath,
        const okapi::TimeUtil& iTimeUtil,
        std::unique_ptr<okapi::Filter> iDerivativeFilter = std::make_unique<okapi::PassthroughFilter>(),
        std::shared_ptr<okapi::Logger> iLogger = okapi::Logger::getDefaultLogger());

    IterativeVelTBHController(
        const Gains& iGains,
        std::unique_ptr<okapi::VelMath> iVelMath,
        const okapi::TimeUtil& iTimeUtil,
        std::unique_ptr<okapi::Filter> iDerivativeFilter = std::make_unique<okapi::PassthroughFilter>(),
        std::shared_ptr<okapi::Logger> iLogger = okapi::Logger::getDefaultLogger());

    /**
     * Do one iteration of the controller. Returns the reading in the range [-1, 1] unless the
     * bounds have been changed with setOutputLimits().
     *
     * @param inewReading new measurement
     * @return controller output
     */
    double step(double inewReading) override;

    /**
     * Sets the target for the controller.
     *
     * @param itarget new target velocity
     */
    void setTarget(double itarget) override;

    /**
     * Writes the value of the controller output. This method might be automatically called in another
     * thread by the controller. The range of input values is expected to be [-1, 1].
     *
     * @param ivalue the controller's output in the range [-1, 1]
     */
    void controllerSet(double ivalue) override;

    /**
     * Gets the last set target, or the default target if none was set.
     *
     * @return the last target
     */
    double getTarget() override;

    /**
     * Gets the last set target, or the default target if none was set.
     *
     * @return the last target
     */
    double getTarget() const;

    /**
     * @return The most recent value of the process variable.
     */
    double getProcessValue() const override;

    /**
     * Returns the last calculated output of the controller.
     */
    double getOutput() const override;

    /**
     * Get the upper output bound.
     *
     * @return  the upper output bound
     */
    double getMaxOutput() override;

    /**
     * Get the lower output bound.
     *
     * @return the lower output bound
     */
    double getMinOutput() override;

    /**
     * Returns the last error of the controller. Does not update when disabled.
     */
    double getError() const override;

    /**
     * Returns whether the controller has settled at the target. Determining what settling means is
     * implementation-dependent.
     *
     * If the controller is disabled, this method must return true.
     *
     * @return whether the controller is settled
     */
    bool isSettled() override;

    /**
     * Set time between loops in ms.
     *
     * @param isampleTime time between loops
     */
    void setSampleTime(okapi::QTime isampleTime) override;

    /**
     * Set controller output bounds. Default bounds are [-1, 1].
     *
     * @param imax max output
     * @param imin min output
     */
    void setOutputLimits(double imax, double imin) override;

    /**
     * Sets the (soft) limits for the target range that controllerSet() scales into. The target
     * computed by controllerSet() is scaled into the range [-itargetMin, itargetMax].
     *
     * @param itargetMax The new max target for controllerSet().
     * @param itargetMin The new min target for controllerSet().
     */
    void setControllerSetTargetLimits(double itargetMax, double itargetMin) override;

    /**
     * Resets the controller's internal state so it is similar to when it was first initialized, while
     * keeping any user-configured information.
     */
    void reset() override;

    /**
     * Changes whether the controller is off or on. Turning the controller on after it was off will
     * cause the controller to move to its last set target, unless it was reset in that time.
     */
    void flipDisable() override;

    /**
     * Sets whether the controller is off or on. Turning the controller on after it was off will
     * cause the controller to move to its last set target, unless it was reset in that time.
     *
     * @param iisDisabled whether the controller is disabled
     */
    void flipDisable(bool iisDisabled) override;

    /**
     * Returns whether the controller is currently disabled.
     *
     * @return whether the controller is currently disabled
     */
    bool isDisabled() const override;

    /**
     * Get the last set sample time.
     *
     * @return sample time
     */
    okapi::QTime getSampleTime() const override;

    /**
     * Do one iteration of velocity calculation.
     *
     * @param inewReading new measurement
     * @return filtered velocity
     */
    virtual okapi::QAngularSpeed stepVel(double inewReading);

    /**
     * Set controller gains.
     *
     * @param igains The new gains.
     */
    virtual void setGains(const Gains &igains);

    /**
     * Gets the current gains.
     *
     * @return The current gains.
     */
    Gains getGains() const;

    /**
     * Sets the number of encoder ticks per revolution. Default is 1800.
     *
     * @param tpr number of measured units per revolution
     */
    virtual void setTicksPerRev(double tpr);

    /**
     * Returns the current velocity.
     */
    virtual okapi::QAngularSpeed getVel() const;

    protected:
    std::shared_ptr<okapi::Logger> logger;
    Gains gain;
    okapi::QTime sampleTime{10 * okapi::millisecond};
    double error{0};
    double prevError{0};
    double tbh{0};
    double output{0};
    double outputMax{1};
    double outputMin{-1};
    double controllerSetTargetMax{1};
    double controllerSetTargetMin{-1};
    bool controllerIsDisabled{false};

    std::unique_ptr<okapi::VelMath> velMath;
    std::unique_ptr<okapi::Filter> derivativeFilter;
    std::unique_ptr<okapi::AbstractTimer> loopDtTimer;
    std::unique_ptr<okapi::SettledUtil> settledUtil;
};