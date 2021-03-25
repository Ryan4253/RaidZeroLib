#include "main.h"

PID::PID(double a, double b, double c){
  kP = a, kI = b, kD = c;
  maxIntegral = 1000000000, minDist = 1000000000;
}

PID::PID(){
  kP = 0, kI = 0, kD = 0;
  maxIntegral = 1000000000, minDist = 1000000000;
}

PID& PID::withGain(double a, double b, double c){
  kP = a, kI = b, kD = c;
  return *this;
}

PID& PID::withIGain(double windup, double dist){
  maxIntegral = windup, minDist = dist;
  return *this;
}

PID& PID::withEMAGain(double alpha){
  dEMA.setGains(alpha);
  return *this;
}

void PID::initialize() {
  prevError = 0, integral = 0;
  dEMA.reset();
  prevTime = pros::millis();
}

double PID::update(double err) {
  // P calculation
  error = err; // error

  // D calculation
  time = pros::millis();
  derivative = dEMA.filter((error - prevError) / (time - prevTime)); // dE / dT, filtered with an EMA filter
  prevTime = time;
  prevError = error;

  // I calculation
  integral += error * (error < minDist); // where to start collecting
  integral *= (((int)error ^ (int)prevError) >= 0); // set to 0 once passes setpoint
  integral = fmin(integral, maxIntegral); // cap integral to a limit

  return error * kP + integral * kI + derivative * kD; // final power output
}

bool PID::settleUtil(double errorThresh, int timeThresh) {
  return !((std::abs(error) <= errorThresh));
};

FPID& FPID::withFGain(double f){
  kF = f;
  return *this;
}

void FPID::setTarget(double t){
  target = t;
}

double FPID::fUpdate(double error){
  return update(error) + kF * target;
}
