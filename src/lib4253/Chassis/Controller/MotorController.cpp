#include "MotorController.hpp"
namespace lib4253{

MotorController::MotorController(const MotorControllerGain& gain){
    setGain(gain);
}

void MotorController::setGain(const MotorControllerGain& gain){
    this->gain = gain;
}

double MotorController::step(const okapi::QSpeed& velocity, const okapi::QAcceleration& acceleration, const okapi::QSpeed& currentSpeed) const {
    return gain.kV * velocity.convert(okapi::mps)
         + gain.kA * acceleration.convert(okapi::mps2)
         + gain.kP * (velocity - currentSpeed).convert(okapi::mps);
}
}