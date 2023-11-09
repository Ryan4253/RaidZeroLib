#include "RaidZeroLib/api/Control/Async/AsyncVelTBHController.hpp"

namespace rz{
  
AsyncVelTBHController::AsyncVelTBHController(
  const std::shared_ptr<ControllerInput<double>> &iInput,
  const std::shared_ptr<ControllerOutput<double>> &iOutput,
  const TimeUtil &iTimeUtil,
  const double iGain,
  std::unique_ptr<VelMath> iVelMath,
  const double iRatio,
  const std::shared_ptr<Logger> &iLogger)
  : AsyncWrapper<double, double>(
        iInput,
        iOutput,
        std::make_shared<IterativeVelTBHController>(iGain, std::move(iVelMath), iTimeUtil), 
        iTimeUtil.getRateSupplier(),
        iRatio,
        iLogger),
        internalController(std::static_pointer_cast<IterativeVelTBHController>(controller)) {
}

void AsyncVelTBHController::setGains(const double iGain) {
  internalController->setGains(iGain);
}

double AsyncVelTBHController::getGains() const {
  return internalController->getGains();
}
} // namespace okapi