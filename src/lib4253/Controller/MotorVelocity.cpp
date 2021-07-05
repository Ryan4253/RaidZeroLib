#include "lib4253/Controller/MotorVelocity.hpp"
namespace lib4253{

MotorVelocityController::MotorVelocityController(){
    this->gain = {0,0,0};
}

MotorVelocityController::MotorVelocityController(const MotorVelocityControllerGain& gain){
    setGain(gain);
}

void MotorVelocityController::setGain(const MotorVelocityControllerGain& gain){
    this->gain = gain;
}

double MotorVelocityController::calcPower(const double& velocity, const double& acceleration, const double& currentRPM) const {
    return gain.kV * velocity + gain.kA * acceleration + gain.kP * (velocity - currentRPM);
}

double MotorVelocityController::calcPower(const TrajectoryPoint& v, const double& currentRPM) const {
    return gain.kV * v.velocity + gain.kA * v.acceleration + gain.kP * (v.velocity - currentRPM);
}
}