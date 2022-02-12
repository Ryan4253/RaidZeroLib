#pragma once
#include "lib4253/Controller/Iterative/IterativeVelBangBangController.hpp"
#include "okapi/api/control/async/asyncVelocityController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <memory>

namespace lib4253{
using namespace okapi;

class AsyncVelBangBangController : public AsyncWrapper<double, double>,
                              public AsyncVelocityController<double, double> {
    public:
    /**
   * An async velocity TBH controller.
   *
   * @param iInput The controller input.
   * @param iOutput The controller output.
   * @param iTimeUtil The TimeUtil.
   * @param iGain the tbh gain
   * @param iVelMath The VelMath used for calculating velocity.
   * @param iRatio Any external gear ratio.
   * @param iLogger the logger instance to log to
   */
    AsyncVelBangBangController(
        const std::shared_ptr<ControllerInput<double>> &iInput,
        const std::shared_ptr<ControllerOutput<double>> &iOutput,
        const TimeUtil &iTimeUtil,
        const IterativeVelBangBangController::Gains& iGain,
        std::unique_ptr<VelMath> iVelMath,
        double iRatio = 1,
        const std::shared_ptr<Logger> &iLogger = Logger::getDefaultLogger());

    ~AsyncVelBangBangController() throw()= default;


    /**
     * Set controller gains.
     *
     * @param iGains The new gains.
     */
    void setGains(const IterativeVelBangBangController::Gains iGains);

    /**
     * Gets the current gains.
     *
     * @return The current gains.
     */
    IterativeVelBangBangController::Gains getGains() const;

    protected:
    std::shared_ptr<IterativeVelBangBangController> internalController;
};
}