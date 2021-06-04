#include "lib4253/Controller/BangBang.hpp"
namespace lib4253{

BangBang::BangBang(const double& h, const double& l, const double& t){
    highPower = h, lowPower = l, targetVel = t;
}

void BangBang::setTargetVel(const double& t){
    targetVel = t;
}

double BangBang::step(const double& v) const {
    return v > targetVel ? highPower : lowPower;
}
}