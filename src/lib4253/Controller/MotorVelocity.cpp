#include "main.h"

void MotorVelocityController::setGain(double a, double b, double c){
  kV = a, kA = b, kP = c;
}

double MotorVelocityController::calcPower(TrajectoryPoint v, double currentRPM){
  return kV * v.velocity + kA * v.acceleration + kP * (v.velocity - currentRPM);
}
