#include "lib4253/Controller/BangBang.hpp"
namespace lib4253{

BangBang::BangBang(double h, double l, double t){
    highPower = h, lowPower = l, targetVel = t;
}

void BangBang::setTargetVel(double t){
    targetVel = t;
}

double BangBang::step(double v){
    return v > targetVel ? highPower : lowPower;
}
}