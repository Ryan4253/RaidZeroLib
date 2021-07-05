#include "lib4253/Controller/TakeBackHalf.hpp"
namespace lib4253{

TakeBackHalf::TakeBackHalf(){
    this->gain = {0};
}

TakeBackHalf::TakeBackHalf(const TBHGain& gain){
    this->gain = gain;
}

void TakeBackHalf::setGain(const TBHGain& gain){
    this->gain = gain;
}

void TakeBackHalf::setTargetVel(const double& target){
    targetVel = target;
}

void TakeBackHalf::setApproxVel(const double& approx){
    approxVel = approx;
}

void TakeBackHalf::initialize(){
    firstCross = true;
    output = 0, tbhVal = 0;
    error = targetVel, prevError = targetVel;
}

double TakeBackHalf::step(const double& val){
    error = targetVel - val;
    output += error * gain.gain;
    output = Math::clamp(output, 0, 127);

    if(signbit(error) != signbit(prevError)){
        if(firstCross){
            output = approxVel;
            firstCross = false;
        }
        else{
            output = 0.5 * (output + tbhVal);
        }
        tbhVal = output;
    }
    prevError = error;
    return output;
}   
}