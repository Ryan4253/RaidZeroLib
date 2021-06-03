#include "main.h"
#include "lib4253/Controller/LinearMotionProfile.hpp"
namespace lib4253{

LinearMotionProfileController::LinearMotionProfileController(double a, double maxV){
    maxAcc = a,  maxVel = maxV;
}

void LinearMotionProfileController::setDistance(double d){
    dist = d;
    tAcc = maxVel / maxAcc;
    tCruise = (d-(maxAcc*tAcc*tAcc))/maxVel;
    dAcc = tAcc * maxVel / 2;
    dCruise = dAcc + (maxVel * tCruise);
}

double LinearMotionProfileController::getVelocityTime(double t){
    if(t < tAcc){
        return maxAcc * t;
    }
    else if(t > tCruise){
        double tDec = (t - tCruise);
        return maxVel - maxAcc * tDec;
    }
    else{
        return maxVel;
    }
}

double LinearMotionProfileController::  getVelocityDist(double d){
    if(d < dAcc){
        return maxAcc * (sqrt(2 * d / maxAcc));
    }
    else if(d > dCruise){
        double dDec = dAcc - (dCruise - d);
    }
    else{
        return maxVel;
    }
}

double LinearMotionProfileController::getTotalTime(){
    return tAcc * 2 + tCruise;
}
}