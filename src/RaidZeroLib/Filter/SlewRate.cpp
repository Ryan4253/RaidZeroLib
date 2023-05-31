#include "RaidZeroLib/Filter/SlewRate.hpp"

namespace rz{

SlewRate::SlewRate(double iAccStep, double iDecStep) : accStep(iAccStep), decStep(iDecStep){}

SlewRate::SlewRate(double iStep) : SlewRate(iStep, iStep){}

double SlewRate::filter(double iInput){
    double step;

    if(std::abs(speed) < std::abs(iInput)){
        step = accStep;
    }
    else{
        step = decStep;
    }

    if(iInput > speed + step){
        speed += step;
    }
    else if(iInput < speed - step){
        speed -= step;
    }
    else{
        speed = iInput;
    }

    return speed = std::clamp(speed, -12000.0, 12000.0);
}

double SlewRate::getOutput() const{
    return speed;
}

}