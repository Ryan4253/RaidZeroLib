#include "ExpandedSkidSteerModel.hpp"

namespace lib4253{

void ExpandedSkidSteerModel::curvature(double iThrottle, double iCurvature, double iThreshold){
    if(abs(iThrottle) <= iThreshold){
        arcade(0, iCurvature, iThreshold);
        return;
    }

    double left = iThrottle + std::abs(iThrottle) * iCurvature;
    double right = iThrottle - std::abs(iThrottle) * iCurvature;

    double mag = std::max(abs(left), abs(right));
    if(mag > 1.0){
        left /= mag;
        right /= mag;
    }

    leftSideMotor->moveVoltage(left * maxVoltage);
    rightSideMotor->moveVoltage(right * maxVoltage);
}

}