#pragma once
#include "RaidZeroLib/Control/Iterative/IterativeVelTBHController.hpp"
#include "okapi/api/control/async/asyncVelocityController.hpp"
#include "okapi/api/control/async/asyncWrapper.hpp"
#include "okapi/api/control/controllerInput.hpp"
#include "okapi/api/control/controllerOutput.hpp"
#include "okapi/api/util/timeUtil.hpp"
#include <memory>

namespace rz{
using namespace okapi;

class AsyncVelTBHController : public AsyncWrapper<double, double>,
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
  AsyncVelTBHController(
    const std::shared_ptr<ControllerInput<double>> &iInput,
    const std::shared_ptr<ControllerOutput<double>> &iOutput,
    const TimeUtil &iTimeUtil,
    double iGain,
    std::unique_ptr<VelMath> iVelMath,
    double iRatio = 1,
    const std::shared_ptr<Logger> &iLogger = Logger::getDefaultLogger());

  ~AsyncVelTBHController() throw()= default;

  /**
   * Set controller gains.
   *
   * @param iGains The new gains.
   */
  void setGains(const double iGains);

  /**
   * Gets the current gains.
   *
   * @return The current gains.
   */
  double getGains() const;

  protected:
  std::shared_ptr<IterativeVelTBHController> internalController;
};
}