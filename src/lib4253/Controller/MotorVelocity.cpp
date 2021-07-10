#include "lib4253/Controller/MotorVelocity.hpp"
namespace lib4253{

MotorController::MotorController(){
    this->gain = {0,0,0};
}

MotorController::MotorController(const MotorControllerGain& gain){
    setGain(gain);
}

void MotorController::setGain(const MotorControllerGain& gain){
    this->gain = gain;
}

double MotorController::calcPower(const double& velocity, const double& acceleration, const double& currentRPM) const {
    return gain.kV * velocity + gain.kA * acceleration + gain.kP * (velocity - currentRPM);
}
}