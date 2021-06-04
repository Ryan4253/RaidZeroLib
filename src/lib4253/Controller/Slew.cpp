#include "lib4253/Controller/Slew.hpp"
namespace lib4253{

// Constructor, takes in acceleration and decelaration steps
SlewController::SlewController(const double& accel, const double& decel){
    accStep = accel, decStep = decel, speed = 0;
}

SlewController::SlewController(){
    accStep = 0; decStep = 0; speed =0 ;
}

void SlewController::setStep(const double& a, const double& d){
    accStep = a, decStep = d;
}

// To put simply, slew sets a threshold on how fast your motors can go.
// The steps increase / decreasing over time, making your robot able to accelerate smoothly
void SlewController::reset(){
    speed = 0;
}

double SlewController::step(const double& target) {
    double step;

    if(std::fabs(speed) < std::fabs(target)) {
        step = accStep;
    }
    else {
        step = decStep;
    }

    if(target > speed + step) {
        speed += step;
    }
    else if(target < speed - step) {
        speed -= step;
    }
    else {
        speed = target;
    }

    //return fmin(speed, 127);
    return fmin(target, 127);
}
}