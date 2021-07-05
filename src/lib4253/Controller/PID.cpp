#include "lib4253/Controller/PID.hpp"
namespace lib4253{

PID::PID(){
    this->gain = {0,0,0};
    maxIntegral = 1000000000, minDist = 1000000000;
}

PID::PID(const PIDGain& gain){
    this->gain = gain;
    maxIntegral = 1000000000, minDist = 1000000000;
}

void PID::setGain(const PIDGain& gain){
    this->gain = gain;
}

void PID::setIGain(const double& windup, const double& dist){
    maxIntegral = windup, minDist = dist;
}

void PID::setEMAGain(const double& alpha){
    dEMA.setGain(alpha);
}

void PID::initialize() {
    prevError = 0, integral = 0;
    dEMA.reset();
    prevTime = pros::millis();
}

double PID::step(const double& val) {
    // P calculation
    error = val; // error
    // D calculation
    time = pros::millis();
    derivative = dEMA.filter((error - prevError) / (time - prevTime)); // dE / dT, filtered with an EMA filter
    prevTime = time;
    prevError = error;
    // I calculation
    integral += error * (error < minDist); // where to start collecting
    integral *= (((int)error ^ (int)prevError) >= 0); // set to 0 once passes setpoint
    integral = fmin(integral, maxIntegral); // cap integral to a limit
    return error * gain.kP + integral * gain.kI + derivative * gain.kD; // final power output
}   

double PID::getError() const {
    return error;
}

void FPID::setFGain(const double& f){
    kF = f;
}   

void FPID::setTarget(const double& t){
    target = t;
}

double FPID::fUpdate(const double& error) {
    return step(error) + kF * target;
}
}