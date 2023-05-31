#include "RaidZeroLib/Control/Async/AsyncVelBangBangController.hpp"

namespace rz{
    
AsyncVelBangBangController::AsyncVelBangBangController(
    const std::shared_ptr<ControllerInput<double>> &iInput,
    const std::shared_ptr<ControllerOutput<double>> &iOutput,
    const TimeUtil &iTimeUtil,
    const IterativeVelBangBangController::Gains& iGain,
    std::unique_ptr<VelMath> iVelMath,
    const double iRatio,
    const std::shared_ptr<Logger> &iLogger)
    : AsyncWrapper<double, double>(
            iInput,
            iOutput,
            std::make_shared<IterativeVelBangBangController>(iGain, std::move(iVelMath), iTimeUtil), 
            iTimeUtil.getRateSupplier(),
            iRatio,
            iLogger),
            internalController(std::static_pointer_cast<IterativeVelBangBangController>(controller)) {}

    void AsyncVelBangBangController::setGains(const IterativeVelBangBangController::Gains iGain) {
        internalController->setGains(iGain);
    }

    IterativeVelBangBangController::Gains AsyncVelBangBangController::getGains() const {
    return internalController->getGains();
    }

}