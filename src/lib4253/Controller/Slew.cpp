#include "lib4253/Controller/Slew.hpp"
namespace lib4253{

Slew::Slew(const SlewGain& gain){
    this->gain = gain, speed = 0;
}

void Slew::reset(){
    speed = 0, output = 0;
}

void Slew::initialize(){
    reset();
}

double Slew::step(const double& val){
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
    return output = Math::clamp(target, -12000, 12000);
}
}