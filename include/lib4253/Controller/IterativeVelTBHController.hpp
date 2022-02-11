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
#include "okapi/api/control/iterative/iterativeVelocityController.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include "okapi/api/filter/velMath.hpp"
#include <cmath>

namespace lib4253{
using namespace okapi;

class IterativeVelTBHController : public IterativeVelocityController<double, double>{
    public:

    IterativeVelTBHController(
        double iGain, 
        std::unique_ptr<VelMath> iVelMath,
        const TimeUtil& iTimeUtil,
        std::shared_ptr<Logger> iLogger = Logger::getDefaultLogger());

    /**
     * Do one iteration of the controller. Returns the reading in the range [-1, 1] unless the
     * bounds have been changed with setOutputLimits().
     *
     * @param iNewReading new measurement
     * @return controller output
     */
    double step(double iNewReading) override;

    /**
     * Sets the target for the controller.
     *
     * @param iTarget new target velocity
     */
    void setTarget(double iTarget) override;

    /**
     * Writes the value of the controller output. This method might be automatically called in another
     * thread by the controller. The range of input values is expected to be [-1, 1].
     *
     * @param iValue the controller's output in the range [-1, 1]
     */
    void controllerSet(double iValue) override;

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
     * @param iSampleTime time between loops
     */
    void setSampleTime(QTime iSampleTime) override;

    /**
     * Set controller output bounds. Default bounds are [-1, 1].
     *
     * @param iMax max output
     * @param iMin min output
     */
    void setOutputLimits(double iMax, double iMin) override;

    /**
     * Sets the (soft) limits for the target range that controllerSet() scales into. The target
     * computed by controllerSet() is scaled into the range [-itargetMin, itargetMax].
     *
     * @param iTargetMax The new max target for controllerSet().
     * @param iTargetMin The new min target for controllerSet().
     */
    void setControllerSetTargetLimits(double iTargetMax, double iTargetMin) override;

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
     * @param iIsDisabled whether the controller is disabled
     */
    void flipDisable(bool iIsDisabled) override;

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
    QTime getSampleTime() const override;

    /**
     * Do one iteration of velocity calculation.
     *
     * @param iNewReading new measurement
     * @return filtered velocity
     */
    virtual QAngularSpeed stepVel(double iNewReading);

    /**
     * Set controller gain.
     *
     * @param iGain The new gain.
     */
    virtual void setGains(const double iGain);

    /**
     * Gets the current gain.
     *
     * @return The current gain.
     */
    double getGains() const;

    /**
     * Sets the number of encoder ticks per revolution. Default is 1800.
     *
     * @param iTPR number of measured units per revolution
     */
    virtual void setTicksPerRev(double iTPR);

    /**
     * Returns the current velocity.
     */
    virtual QAngularSpeed getVel() const;

    protected:
    double gain{0.0};
    QTime sampleTime{10 * millisecond};
    double target{0}, error{0}, prevError{0}, tbh{0}, output{0};
    double outputMax{1}, outputMin{-1};
    double controllerSetTargetMax{1}, controllerSetTargetMin{-1};
    bool controllerIsDisabled{false};

    std::shared_ptr<Logger> logger;
    std::unique_ptr<VelMath> velMath;
    std::unique_ptr<AbstractTimer> loopDtTimer;
    std::unique_ptr<SettledUtil> settledUtil;
};
}