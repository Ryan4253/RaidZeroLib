#include "main.h"
#include "lib4253/Controller/Slew.hpp"

// Constructor, takes in acceleration and decelaration steps
SlewController::SlewController(SlewGain gain){
    this->gain = gain, speed = 0;
}

SlewController::SlewController(){
    this->gain = {0, 0}; speed = 0;
}

void SlewController::setGain(SlewGain gain){
    this->gain = gain;
}

// To put simply, slew sets a threshold on how fast your motors can go.
// The steps increase / decreasing over time, making your robot able to accelerate smoothly
void SlewController::reset(){
    speed = 0;
}

double SlewController::step(double target) {
    double step;

    if(std::fabs(speed) < std::fabs(target)) {
        step = gain.accStep;
    }
    else {
        step = gain.decStep;
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
