#include "lib4253/Controller/LinearMotionProfile.hpp"
namespace lib4253{

LinearMotionProfileController::LinearMotionProfileController(const double& a, const double& maxV){
    maxAcc = a,  maxVel = maxV;
}

void LinearMotionProfileController::setDistance(const double& d){
    dist = d;
    tAcc = maxVel / maxAcc;
    tCruise = (d-(maxAcc*tAcc*tAcc))/maxVel;
    dAcc = tAcc * maxVel / 2;
    dCruise = dAcc + (maxVel * tCruise);
}

double LinearMotionProfileController::getVelocityTime(const double& t) const {
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

double LinearMotionProfileController::  getVelocityDist(const double& d) const {
    if(d < dAcc){
        return maxAcc * (sqrt(2 * d / maxAcc));
    }
    else if(d > dCruise){
        double dDec = (dCruise + dAcc * 2) - d;
        return maxAcc * (sqrt(2 * d / maxAcc));
    }
    else{
        return maxVel;
    }
}

double LinearMotionProfileController::getTotalTime() const {
    return tAcc * 2 + tCruise;
}
}