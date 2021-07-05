#include "main.h"

MotorVelocityController::MotorVelocityController() {
    this->gain = {0, 0, 0};
}

MotorVelocityController::MotorVelocityController(MotorVelocityControllerGain gain) {
    this->gain = gain;
}

void MotorVelocityController::setGain(MotorVelocityControllerGain gain){
    this->gain = gain;
}

double MotorVelocityController::calcPower(TrajectoryPoint v, double currentRPM){
    return gain.kV * v.velocity + gain.kA * v.acceleration + gain.kP * (v.velocity - currentRPM);
}
