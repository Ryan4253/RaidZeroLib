#include "Motor.hpp"

namespace lib4253{
Motor::Motor(const int& iport, const okapi::AbstractMotor::gearset& cartridge, const double& ia, const MotorControllerGain& constant):
okapi::Motor((std::uint8_t)(std::abs(iport)), iport < 0, cartridge, okapi::AbstractMotor::encoderUnits::degrees)
{
    velController = std::move(std::make_unique<MotorController>(constant));
    velFilter = std::move(std::make_unique<EmaFilter>(ia));
}

void Motor::setVelocity(const okapi::QSpeed& velocity, const okapi::QAcceleration& acceleration, const okapi::QSpeed& currentSpeed){
    double power = velController->step(velocity, acceleration, currentSpeed);
    moveVoltage(power);
}

double Motor::getFilteredRPM(){
    double vel = getActualVelocity();
    return velFilter->filter(vel);
}

}