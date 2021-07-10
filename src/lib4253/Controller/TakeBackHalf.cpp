#include "lib4253/Controller/TakeBackHalf.hpp"
#include <algorithm>
#include <cmath>
namespace lib4253{

TakeBackHalf::TakeBackHalf(const TBHGain& gain){
    this->gain = gain;
}

void TakeBackHalf::reset(){
    firstCross = true;
    output = 0, tbh = 0;
    error = 0, prevError = 0;
}

void TakeBackHalf::initialize(){
    reset();
}

double TakeBackHalf::step(const double& val){
    error = val;
    output += error * gain.gain;

    if(signbit(error) != signbit(prevError)){
        if(firstCross){
            output = gain.approxVel;
            firstCross = false;
        }
        else{
            output = 0.5 * (output + tbh);
        }
        tbh = output;
    }
    prevError = error;
    return output;
}   
}