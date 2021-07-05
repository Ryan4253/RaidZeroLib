#include "main.h"
#include "lib4253/Controller/BangBang.hpp"

BangBang::BangBang(BangBangGain gain){
    this->gain = gain;
}

void BangBang::setGain(BangBangGain gain){
    this->gain = gain;
}

void BangBang::setTargetVel(double t){
    gain.targetVel = t;
}

double BangBang::step(double v){
    return v > gain.targetVel ? gain.highPower : gain.lowPower;
}
