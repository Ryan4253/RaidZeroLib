#include "main.h"

PID::PID(double a, double b, double c){
  kP = a, kI = b, kD = c;
}

void PID::setGain(double a, double b, double c){
  kP = a, kI = b, kD = c;
}

void PID::initialize() {
  prevError = 0, integral = 0;
  dEMA.reset();
  time.getDt();
}

double PID::update(double err) {
  error = err;

  derivative = (dEMA.filter(error - prevError));// / (time.getDt().convert(millisecond));
  prevError = error;
  integral += error;
  integral *= !(((int)error ^ (int)prevError) < 0);

  output = error * kP + integral * kI + derivative * kD;

  return output;
}

bool PID::settleUtil(double errorThresh, int timeThresh) {
  return !((std::abs(error) <= errorThresh));
};
