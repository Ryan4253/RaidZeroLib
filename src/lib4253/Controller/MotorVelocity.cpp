#include "lib4253/Controller/MotorVelocity.hpp"
namespace lib4253{

MotorVelocityController::MotorVelocityController(const double& _kV, const double& _kA, const double& _kP){
    setGain(_kV, _kA, _kP);
}

void MotorVelocityController::setGain(const double& _kV, const double& _kA, const double& _kP){
    kV = _kV, kA = _kA, kP = _kP;
}

double MotorVelocityController::calcPower(const double& velocity, const double& acceleration, const double& currentRPM) const {
    return kV * velocity + kA * acceleration + kP * (velocity - currentRPM);
}

double MotorVelocityController::calcPower(const TrajectoryPoint& v, const double& currentRPM) const {
    return kV * v.velocity + kA * v.acceleration + kP * (v.velocity - currentRPM);
}
}