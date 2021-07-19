#include "lib4253/Controller/PID.hpp"
namespace lib4253{

PID::PID(){
    this->gain = {0,0,0, 1000000000, 1000000000, 1};
    this->derivEMA.setGain(gain.emaGain);
}

PID::PID(const PIDGain& gain){
    this->gain = gain;
    this->derivEMA.setGain(gain.emaGain);
}

void PID::setPIDGain(const double& kP, const double& kI, const double& kD){
    gain.kP = kP, gain.kI = kI, gain.kD = kD;
}

void PID::setIGain(const double& windup, const double& dist){
    gain.maxIntegral = windup, gain.minDist = dist;
}

void PID::setEMAGain(const double& alpha){
    gain.emaGain = alpha;
    derivEMA.setGain(alpha);
}

double PID::getDerivative() const{
    return derivative;
}

double PID::getIntegral() const{
    return integral;
}

void PID::initialize(){
    reset();
}

void PID::reset(){
    error = 0, prevError = 0, derivative = 0, integral = 0, output = 0;
    derivEMA.reset();
    timer.getTimer()->getDt();
}

double PID::step(const double& val) {
    // P calculation
    error = val; // error
    // D calculation
    derivative = derivEMA.filter((error - prevError) / (timer.getTimer()->getDt().convert(okapi::millisecond))); // dE / dT, filtered with an EMA filter
    prevError = error;
    // I calculation
    integral += error * (error <= gain.minDist); // where to start collecting
    integral *= (std::signbit(error) != std::signbit(prevError)); // set to 0 once passes setpoint
    integral = fmin(integral, gain.maxIntegral); // cap integral to a limit
    return output = error * gain.kP + integral * gain.kI + derivative * gain.kD; // final power output
}


///////////////////////////////////////////////

FFPID::FFPID(const FFPIDGain& iGain, const double& target){
    this->gain = gain;
    pid.setGain({gain.kP, gain.kI, gain.kD, gain.maxIntegral, gain.minDist, gain.emaGain});
    this->target = target;
}

void FFPID::setFFGain(const double& kF){
    gain.kF = kF;
}

void FFPID::setPIDGain(const double& kP, const double& kI, const double& kD){
    pid.setPIDGain(kP, kI, kD);
}

void FFPID::setIGain(const double& windup, const double& dist){
    pid.setIGain(windup, dist);
}

void FFPID::setEMAGain(const double& alpha){
    pid.setEMAGain(alpha);
}

double FFPID::getDerivative() const{
    return pid.getDerivative();
}

double FFPID::getIntegral() const{
    return pid.getIntegral();
}

void FFPID::initialize(){
    reset();
    output = 0;
}

void FFPID::reset(){
    pid.reset();
    output = 0;
}

double FFPID::step(const double& val) {
    return output = gain.kF * target + pid.step(val); // final power output
}

}