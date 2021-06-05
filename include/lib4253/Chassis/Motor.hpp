#pragma once
#include "lib4253/Controller/MotorVelocity.hpp"
#include "lib4253/Filter/EMA.hpp"
#include "okapi/impl/device/motor/motor.hpp"
#include "okapi/api/device/motor/abstractMotor.hpp"

namespace lib4253{
using namespace okapi;
class Motor : public okapi::Motor{
    private:
    std::unique_ptr<MotorVelocityController> velController {nullptr};
    std::unique_ptr<EmaFilter> velFilter {nullptr};

    public:
    Motor(const int& iport, const okapi::AbstractMotor::gearset& cartridge, const double& ia, const std::tuple<double, double, double>& constant);
    void setRPM(const std::int16_t& ivelocity);
    void setRPM(const std::int16_t& ivelocity, const std::int16_t& iacceleration);
    double getFilteredVelocity();
};
}