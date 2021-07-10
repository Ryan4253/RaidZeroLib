#include "lib4253/Controller/TakeBackHalf.hpp"
#include <algorithm>
#include <cmath>
namespace lib4253{

TakeBackHalf::TakeBackHalf(const double& g){
    gain = g;
}

void TakeBackHalf::setGain(const double& g){
    gain = g;
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

double TakeBackHalf::step(const double& rpm){
    error = targetVel - rpm;
    output += error * gain;
    if(output > 127){
        output = 127;
    }
    else if(output < 0){
        output = 0;
    }

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