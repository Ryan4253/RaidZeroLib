#pragma once
#include "lib4253/Chassis/Controller/MotorController.hpp" // done
#include "lib4253/Filter/EMA.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"

namespace lib4253{
class Motor : public okapi::Motor{
    private:
    std::unique_ptr<MotorController> velController {nullptr};
    std::unique_ptr<EmaFilter> velFilter {nullptr};

    public:
    Motor(const int& iport, const okapi::AbstractMotor::gearset& cartridge, const double& ia, const MotorControllerGain& constant);
    void setVelocity(const okapi::QSpeed& velocity, const okapi::QAcceleration& acceleration, const okapi::QSpeed& currentSpeed);
    double getFilteredRPM();
};
}