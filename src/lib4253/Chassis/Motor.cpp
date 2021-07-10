#include "lib4253/Chassis/Motor.hpp"

namespace lib4253{
Motor::Motor(const int& iport, const okapi::AbstractMotor::gearset& cartridge, const double& ia, const MotorControllerGain& constant):
okapi::Motor((std::uint8_t)(std::abs(iport)), iport < 0, cartridge, okapi::AbstractMotor::encoderUnits::degrees)
{
    velController = std::move(std::make_unique<MotorController>(constant));
    velFilter = std::move(std::make_unique<EmaFilter>(ia));
}

void Motor::setRPM(const std::int16_t& ivelocity){
    double power = velController->calcPower(ivelocity, 0, getFilteredVelocity());
    moveVoltage(power);
}

void Motor::setRPM(const std::int16_t& ivelocity, const std::int16_t& iacceleration){
    double power = velController->calcPower(ivelocity, iacceleration, getFilteredVelocity());
    moveVoltage(power);
}

double Motor::getFilteredVelocity(){
    double vel = getActualVelocity();
    return velFilter->filter(vel);
}

}