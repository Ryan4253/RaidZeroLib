#include "lib4253/Controller/Slew.hpp"
namespace lib4253{

SlewController::Gains::Gains(double accStep, double decStep):
    accStep(accStep), decStep(decStep){}

SlewController::SlewController(double iAccStep, double iDecStep): SlewController(Gains(iAccStep, iDecStep)){}

SlewController::SlewController(const Gains& iGain){
    gain = iGain, speed = 0;
}

void SlewController::reset(){
    speed = 0, output = 0;
}

double SlewController::step(const double& val){
    double step;

    if(std::fabs(speed) < std::fabs(val)){
        step = gain.accStep;
    }
    else{
        step = gain.decStep;
    }

    if(target > speed + step){
        speed += step;
    }
    else if(target < speed - step){
        speed -= step;
    }
    else{
        speed = val;
    }

    //return fmin(speed, 127);
    return output = std::clamp(target, -12000.0, 12000.0);
}
}